#include "Common.h"
#include "Globals.h"
#include "hid/HIDTypes.h"
#include "hid/CUSBDeviceFinder.h"
#include "hid/CQuadcast2SHandshaker.h"
#include "communicator/CQuadcast2SCommunicator.h"
#include "display/CQC2SDisplayFactory.h"
#include "util/ArgParsing.h"
#include "util/ConfigParser.h"

#include <csignal>
#include <iostream>

UniquePtr<CQC2SDisplay>  CreateDisplay(int p_argc, char *p_pArgv[], AtomicBool& p_outVerbosity, Option<Set<WString>>& p_outAllowedSerials)
{
    // Check if we have a config defined
    auto configPathOpt = CConfigParser::ParseConfigPathArg(p_argc, p_pArgv);
    if (configPathOpt.has_value())
    {
        try
        {
            CConfigParser configParser(configPathOpt.value());
            auto parseResult = configParser.Parse();
            if (!parseResult.m_pDisplay)
            {
                std::cerr << "Config parsing failed: no startup display defined." << std::endl;
                return nullptr;
            }
            // if either option (arg or config) is verbose or config verbose is true, we want verbose on
            p_outVerbosity.store(parseResult.m_verbose | p_outVerbosity.load());
            // dont overwrite if they were passed via args
            if (!p_outAllowedSerials.has_value() && parseResult.m_allowedSerials.has_value() && !parseResult.m_allowedSerials->empty())
                p_outAllowedSerials = parseResult.m_allowedSerials;
            else if(parseResult.m_allowedSerials.has_value())
                std::cerr << "[Main] Ignoring allowed serials from config because they were also passed via command line arguments." << std::endl;
            return std::move(parseResult.m_pDisplay);
        }
        catch (const std::exception &e)
        {
            std::cerr << "Failed to parse config: " << e.what() << std::endl;
            return nullptr;
        }
    }
    else {
        return CQC2SDisplayFactory::CreateFromArgs(p_argc, p_pArgv);
    }
}

int main(int p_argc, char *p_pArgv[])
{
    /* minimize the communication breaks by splitting the logic into 3 separate threads
    - device finder
    - device connector (handshake/verify correct interface)
    - device communicator
    this way we can split device array into the pipeline and only create a possible minor hiccup upon connecting to a new device
    keep in mind, we do have the ability to disconnect a device inside the communicator thread, meaning that the finder thread
    needs to have read access to the communicator device list

    finder --> connector --> communicator
     ^                            /
     '---------------------------'

    */
    // "finder" logic
    auto allowedSerials = ParseSerialArgs(p_argc, p_pArgv);
    g_verbosity =
#ifdef DEBUG
        true;
#else
        ParseVerbose(p_argc, p_pArgv);
#endif

    UniquePtr<CQC2SDisplay> pDisplay = CreateDisplay(p_argc, p_pArgv,g_verbosity, allowedSerials);
    

    // Finder -> handshake thread pipeline
    CQuadcast2SHandshaker handshaker;
    HIDDeviceContainer finderHandshakeDevicesPass;
    const auto POLLING_FREQUENCY = 2s; // poll every 2
    ConditionVariable cvNewDevicesFound;
    Mutex finderHandshakeMtx;
    bool handshakeBatchDone = true; // true when handshake is idle/done with current batch

    // CV for notification to main thread that devices have been connected to set up sender thread
    ConditionVariable cvConnectedDevicesUpdated;
    Mutex connectedDevicesUpdatedMtx;

    // CV for notification to finder thread that handshake is complete and finding may resume
    ConditionVariable cvHandshakeDone;
    Mutex handshakeDoneMtx;

    // handshake -> sender thread pipeline
    CQuadcast2SCommunicator communicator;
    HIDDeviceContainer connectedHandshakeDevicesPass;
    Mutex handshakeSenderMtx;
    bool senderBatchDone = true; // true when sender is idle/done with current batch

    ConditionVariable cvSenderDone;
    Mutex senderDoneMtx;

    auto finderThread = [&]()
    {
        CUSBDeviceFinder finder;
        // lets do that again
        while (!g_signalStopRequest)
        {
            HIDDeviceContainer devices;
            Set<WString> connectedSerials = communicator.GetOpenSerials();

            if (g_verbosity)
            {
                std::wcout << L"[Finder] Current connected serials: ";
                for (const auto &s : connectedSerials)
                    std::wcout << s << L" ";
                std::wcout << std::endl;
            }

            devices = finder.FindDevices(g_QUADCAST2S_USB_ID,
                                         allowedSerials,
                                         connectedSerials);
            // new devices found, pass to connector thread
            if (!devices.empty())
            {
                // pass new devices to the pipe
                std::wcout << L"[Finder] Preparing new devices to connector thread..." << std::endl;
                {
                    LockGuard lg(finderHandshakeMtx);
                    finderHandshakeDevicesPass = devices;
                }
                {
                    LockGuard lg(handshakeDoneMtx);
                    handshakeBatchDone = false; // new batch submitted
                }
                std::wcout << L"[Finder] Signaling connector thread..." << std::endl;
                cvNewDevicesFound.notify_one();
                // wait for signal from connector thread that it is done completely
                std::wcout << L"[Finder] Waiting for connector thread to finish processing..." << std::endl;
                UniqueLock lock(handshakeDoneMtx);
                cvHandshakeDone.wait(lock, [&]()
                                     { return handshakeBatchDone || g_signalStopRequest.load(); });
                std::wcout << L"[Finder] Continuing..." << std::endl;
            }
            std::this_thread::sleep_for(POLLING_FREQUENCY);
        }
        // leaving is only possible if we do not wait. (potential dead lock risk)
        // Sender should not be able to block complete handshake pipeline at this time too because of this predicate
        cvNewDevicesFound.notify_all();

        std::wcout << L"[Finder] Finder thread exiting..." << std::endl;
    };

    auto connectorThread = [&]()
    {
        // lets do this again
        while (!g_signalStopRequest)
        {
            if (g_verbosity)
                std::wcout << L"[Handshake] Waiting for new devices from finder thread..." << std::endl;
            UniqueLock lock(finderHandshakeMtx);
            // wait for signal to process new devices (or if we cancel process)
            cvNewDevicesFound.wait(lock,
                                   [&finderHandshakeDevicesPass]()
                                   {
                                       return !finderHandshakeDevicesPass.empty() || g_signalStopRequest;
                                   });

            if (g_verbosity)
                std::wcout << L"[Handshake] Received signal from finder thread, processing devices..." << std::endl;
            // if new devices are to be passed, work them through (this is false if its a signal stop request)
            while (!finderHandshakeDevicesPass.empty())
            {
                // Work off the devices
                auto next = finderHandshakeDevicesPass.back();
                finderHandshakeDevicesPass.pop_back();

                // print
                if (g_verbosity)
                {
                    auto pInfo = hid_get_device_info(next.get());
                    if (pInfo)
                    {
                        std::wcout << L"[Handshake] Processing device: "
                                   << (pInfo->manufacturer_string ? pInfo->manufacturer_string : L"(unknown)")
                                   << L" " << (pInfo->product_string ? pInfo->product_string : L"(unknown)")
                                   << L" serial=" << (pInfo->serial_number ? pInfo->serial_number : L"?")
                                   << L" path=" << (pInfo->path ? pInfo->path : "(unknown)")
                                   << std::endl;
                    }
                }
                // only returns viable ptr on successful response from device
                auto addRet = handshaker.ConnectOne(next);
                if (addRet)
                {
                    WString addedSerial;
                    {
                        auto pInfo = hid_get_device_info(addRet.get());
                        if (pInfo)
                            addedSerial = WString(pInfo->serial_number);
                    }

                    if (g_verbosity)
                    {
                        std::wcout << L"[Handshake] Successfully connected to device, adding to communicator pipeline. Serial: " << addedSerial << std::endl;
                    }
                    // add to pipe
                    {
                        LockGuard lg(handshakeSenderMtx);
                        connectedHandshakeDevicesPass.push_back(addRet);
                    }
                    // remove same serials
                    if (!addedSerial.empty())
                    {
                        auto originalSize = finderHandshakeDevicesPass.size();
                        auto updtEnd = std::remove_if(finderHandshakeDevicesPass.begin(), finderHandshakeDevicesPass.end(),
                                       [&addedSerial](const auto &p_device)
                                       {
                                           auto pInfo = hid_get_device_info(p_device.get());
                                           if (!pInfo)
                                               return false;
                                           auto serial = WString(pInfo->serial_number);
                                           auto isDuplicate = serial == addedSerial;
                                           return isDuplicate;
                                       });
                        finderHandshakeDevicesPass.erase(updtEnd, finderHandshakeDevicesPass.end());
                        if (g_verbosity)
                        {
                            auto newSize = finderHandshakeDevicesPass.size();
                            std::wcout << L"[Handshake] Removed " << (originalSize - newSize) << L" duplicate devices with serial " << addedSerial << L" from handshake pipeline due to same serial." << std::endl;
                        }
                    }
                }
                else if (g_verbosity)
                {
                    std::wcout << L"[Handshake] Failed handshake for device, skipping" << std::endl;
                }
            }

            // Notify that we are done
            if (g_verbosity)
                std::wcout << L"[Handshake] Done processing devices, notifying main..." << std::endl;

            // Mark handshake batch as complete before notifying finder
            {
                LockGuard lg(handshakeDoneMtx);
                handshakeBatchDone = true;
            }
            cvHandshakeDone.notify_one();

            // notify main in case we are at init state
            {
                UniqueLock lock2(connectedDevicesUpdatedMtx);
                cvConnectedDevicesUpdated.notify_one();
            }

            // No need to wait for sender anymore - finder will wait on handshakeBatchDone
        }

        std::wcout << L"[Handshake] Connector thread exiting..." << std::endl;
    };


    auto handleIncomingNewDevices = [&](CQuadcast2SCommunicator &p_communicator)
    {
        {
            LockGuard lg(handshakeSenderMtx);
            // could intersect with the while loop from handshake,
            // but this just means we have less devices for a irrelevant amount of ticks
            if (connectedHandshakeDevicesPass.empty())
                return true;

            if (g_verbosity)
                std::wcout << L"[Sender] New devices received from handshake thread, adding to communicator..." << std::endl;
            while (!connectedHandshakeDevicesPass.empty())
            {
                auto currentDevicePaths = p_communicator.GetOpenPaths();
                // pop device from queue
                auto device = connectedHandshakeDevicesPass.back();
                connectedHandshakeDevicesPass.pop_back();
                // do an extra check if device already connected (prevent duplicates from handshake and possible race condition)
                // any double connected device (device, not interface) will not light up the LEDs, so just to be sure
                auto pInfo = hid_get_device_info(device.get());
                if (!pInfo)
                    continue;
                String path(pInfo->path);
                if (std::find(currentDevicePaths.begin(), currentDevicePaths.end(), path) != currentDevicePaths.end())
                    continue; // already connected, skip
                // add new device to communicate with

                if (g_verbosity)
                {
                    WString serial = pInfo->serial_number ? WString(pInfo->serial_number) : L"(unknown)";
                    std::wcout << L"[Sender] Adding device " << serial << L" to communicator." << std::endl;
                }
                p_communicator.AddDevice(device);
            }
        }

        if (g_verbosity)
            std::wcout << L"[Sender] Done processing new devices." << std::endl;
        // No need to signal back - handshake already marked itself done
        // return value handles cancellation of display function, we don't really need that.
        return true;
    };

    auto senderThread = [&]()
    {
        pDisplay->Display(
            communicator,
            g_signalStopRequest,
            handleIncomingNewDevices);

        std::wcout << L"[Communicator] Display function ended, signaling other threads to stop..." << std::endl;
        g_signalStopRequest = true; // signal other threads to stop as well, in case display ends on its own (e.g. video end condition met)
    };

    std::signal(SIGINT, [](int)
                { g_signalStopRequest = true; });
    std::signal(SIGTERM, [](int)
                { g_signalStopRequest = true; });

    Thread tFinder(finderThread);
    Thread tConnector(connectorThread);

    // Await initial start to have at least one device connected
    {
        UniqueLock lock(connectedDevicesUpdatedMtx);
        cvConnectedDevicesUpdated.wait(lock, [&]()
                                       { return !connectedHandshakeDevicesPass.empty() || g_signalStopRequest.load(); });
    }

    if (!pDisplay->Initialize())
        throw std::runtime_error("Failed to initialize display: " + pDisplay->GetName());

    if (g_signalStopRequest)
    {
        if (g_verbosity)
            std::wcout << L"[Main] Stop signal received before starting sender thread, exiting..." << std::endl;
        tFinder.join();
        tConnector.join();
        return 0;
    }

    if (g_verbosity)
        std::wcout << L"[Main] Found device. Starting sender thread..." << std::endl;
    Thread tSender(senderThread);

    // joins here
    tFinder.join();
    tConnector.join();
    tSender.join();

    // shutdown
    pDisplay->Shutdown(communicator);
}
