// Copyright (c) 2026 Mario T
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#include "Common.h"
#include "Globals.h"
#include "config/CConfigBuilder.h"
#include "hid/HIDTypes.h"
#include "hid/CUSBDeviceFinder.h"
#include "hid/CQuadcast2SHandshaker.h"
#include "communicator/CQuadcast2SCommunicator.h"
#include "audio/CAudioProcessor.h"
#include "util/ArgParsing.h"

#include <csignal>
#include <iostream>

#ifdef USE_SYSTEMD
#include <systemd/sd-daemon.h>
void NotifySystemdReady()
{
    LOG_VERBOSE(L"READY=1 NOTIFY...");
    sd_notify(0, "READY=1");
}
void NotifySystemdWatchdog()
{
    sd_notify(0, "WATCHDOG=1");
}
void NotifySystemdStopping()
{
    LOG_VERBOSE(L"STOPPING=1 NOTIFY...");
    sd_notify(0, "STOPPING=1");
}
#define SYSTEMD_NOTIFY_READY NotifySystemdReady()
#define SYSTEMD_NOTIFY_WATCHDOG NotifySystemdWatchdog()
#define SYSTEMD_NOTIFY_STOPPING NotifySystemdStopping()
#define SYSTEMD_WATCHDOG_DECL_USEC(lastWatchdogNotify,varname) \
    auto lastWatchdogNotify = std::chrono::steady_clock::now(); \
    uint64_t varname = 0;                \
    sd_watchdog_enabled(0, &varname)
#define SYSTEMD_WATCHDOG_INTERVAL(varname) std::chrono::microseconds(varname / 1000000)
#define SYSTEMD_NOTIFY_WATCHDOG_IF_DUE(interval, lastNotify) \
    do { \
        if ((interval).count() > 0) { \
            auto now = std::chrono::steady_clock::now(); \
            if (now - (lastNotify) >= (interval)) { \
                SYSTEMD_NOTIFY_WATCHDOG; \
                (lastNotify) = now; \
            } \
        } \
    } while(0)
#else
#define SYSTEMD_NOTIFY_READY
#define SYSTEMD_NOTIFY_WATCHDOG
#define SYSTEMD_NOTIFY_STOPPING
#define SYSTEMD_WATCHDOG_DECL_USEC(lastWatchdogNotify,varname)
#define SYSTEMD_WATCHDOG_INTERVAL(varname) std::chrono::microseconds(0);
#define SYSTEMD_NOTIFY_WATCHDOG_IF_DUE(interval, lastNotify)
#endif

// CreateDisplay has been replaced by CConfigBuilder::Build()
// See src/config/CConfigBuilder.h

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
    // ── Handle early-exit flags ───────────────────────────────────────
    if (ParseFlag(p_argc, p_pArgv, {"--help", "-h"}))
    {
        PrintHelp(p_argc > 0 ? p_pArgv[0] : "qc2srgb");
        return EXIT_SUCCESS;
    }
    if (ParseFlag(p_argc, p_pArgv, {"--list-audio-devices"}))
    {
        CAudioProcessor::PrintDevices();
        return EXIT_SUCCESS;
    }

    // ── Build unified configuration (CLI args + optional config file) ──
    SProgramConfig cfg = CConfigBuilder::Build(p_argc, p_pArgv);

    // Apply config to globals
    g_verbosity =
#ifdef DEBUG
        true;
#else
        cfg.m_verbose;
#endif
    g_noWaitForRead.store(cfg.m_noWaitForRead);

    if (!cfg.m_pDisplay)
    {
        LOG_ERROR(L"No startup display could be created. Exiting.");
        return EXIT_FAILURE;
    }

    // ── Audio capture ───────────────────────────────────────────────────
    CAudioProcessor audioProcessor;
    if (cfg.m_enableAudio)
    {
        if (!audioProcessor.Initialize(1024, cfg.m_audioDeviceId, cfg.m_audioChannel))
        {
            LOG("[Main] Failed to initialize audio processor.");
            return EXIT_FAILURE;
        }
        else
        {
            audioProcessor.SetInputGain(cfg.m_inputGain);
            audioProcessor.SetSmoothing(cfg.m_audioSmoothing, cfg.m_audioSmoothingAlpha);
            cfg.m_pDisplay->SetAudioProcessor(&audioProcessor);
        }
    }
    // ─────────────────────────────────────────────────────────────────────

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

    auto finderThread = [&]()
    {
        CUSBDeviceFinder finder;
        // lets do that again
        while (!g_signalStopRequest)
        {
            HIDDeviceContainer devices;
            Set<WString> connectedSerials = communicator.GetOpenSerials();

            devices = finder.FindDevices(g_QUADCAST2S_USB_ID,
                                         cfg.m_allowedSerials,
                                         connectedSerials);
            // new devices found, pass to connector thread
            if (!devices.empty())
            {
                // pass new devices to the pipe
                LOG_VERBOSE(L"[Finder] Preparing new devices to connector thread..." );
                {
                    LockGuard lg(finderHandshakeMtx);
                    finderHandshakeDevicesPass = devices;
                }
                {
                    LockGuard lg(handshakeDoneMtx);
                    handshakeBatchDone = false; // new batch submitted
                }
                LOG_VERBOSE(L"[Finder] Signaling connector thread..." );
                cvNewDevicesFound.notify_one();
                // wait for signal from connector thread that it is done completely
                LOG_VERBOSE(L"[Finder] Waiting for connector thread to finish processing..." );
                UniqueLock lock(handshakeDoneMtx);
                cvHandshakeDone.wait(lock, [&]()
                                     { return handshakeBatchDone || g_signalStopRequest.load(); });
                LOG_VERBOSE(L"[Finder] Continuing..." );
            }
            std::this_thread::sleep_for(POLLING_FREQUENCY);
        }
        // leaving is only possible if we do not wait. (potential dead lock risk)
        // Sender should not be able to block complete handshake pipeline at this time too because of this predicate
        cvNewDevicesFound.notify_all();

        LOG(L"[Finder] Finder thread exiting..." );
    };

    auto connectorThread = [&]()
    {
        // lets do this again
        while (!g_signalStopRequest)
        {
            LOG_VERBOSE(L"[Handshake] Waiting for new devices from finder thread..." );
            UniqueLock lock(finderHandshakeMtx);
            // wait for signal to process new devices (or if we cancel process)
            cvNewDevicesFound.wait(lock,
                                   [&finderHandshakeDevicesPass]()
                                   {
                                       return !finderHandshakeDevicesPass.empty() || g_signalStopRequest;
                                   });

            LOG_VERBOSE(L"[Handshake] Received signal from finder thread, processing devices..." );
            // if new devices are to be passed, work them through (this is false if its a signal stop request)
            while (!finderHandshakeDevicesPass.empty())
            {
                // Work off the devices
                auto next = finderHandshakeDevicesPass.back();
                finderHandshakeDevicesPass.pop_back();

                // print
                {
                    auto pInfo = hid_get_device_info(next.get());
                    if (pInfo)
                    {
                        LOG(L"[Handshake] Processing device: "
                                   << (pInfo->manufacturer_string ? pInfo->manufacturer_string : L"(unknown)")
                                   << L" " << (pInfo->product_string ? pInfo->product_string : L"(unknown)")
                                   << L" serial=" << (pInfo->serial_number ? pInfo->serial_number : L"?")
                                   << L" path=" << (pInfo->path ? pInfo->path : "(unknown)"));
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
                    LOG(L"[Handshake] Successfully connected to device, adding to communicator pipeline. Serial: " << addedSerial );

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
                        {
                            auto newSize = finderHandshakeDevicesPass.size();
                            LOG_VERBOSE(L"[Handshake] Removed " << (originalSize - newSize) << L" duplicate devices with serial " << addedSerial << L" from handshake pipeline due to same serial." );
                        }
                    }
                }
                else
                {
                    LOG(L"[Handshake] Failed handshake for device, skipping" );
                }
            }

            // Notify that we are done
            LOG_VERBOSE(L"[Handshake] Done processing devices, notifying main..." );

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

        LOG(L"[Handshake] Connector thread exiting..." );
    };

    SYSTEMD_WATCHDOG_DECL_USEC(lastWatchdogNotify,watchdogIntervalVar);
    const auto WATCHDOG_INTERVAL = SYSTEMD_WATCHDOG_INTERVAL(watchdogIntervalVar);
    auto handleIncomingNewDevices = [&](CQuadcast2SCommunicator &p_communicator)
    {
        SYSTEMD_NOTIFY_WATCHDOG_IF_DUE(WATCHDOG_INTERVAL, lastWatchdogNotify);
        {
            LockGuard lg(handshakeSenderMtx);
            // could intersect with the while loop from handshake,
            // but this just means we have less devices for a irrelevant amount of ticks
            if (connectedHandshakeDevicesPass.empty())
                return true;

            LOG_VERBOSE(L"[Sender] New devices received from handshake thread, adding to communicator..." );
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

                {
                    WString serial = pInfo->serial_number ? WString(pInfo->serial_number) : L"(unknown)";
                    LOG_VERBOSE(L"[Sender] Adding device " << serial << L" to communicator." );
                }
                p_communicator.AddDevice(device);
            }
        }

        LOG_VERBOSE(L"[Sender] Done processing new devices." );
        // No need to signal back - handshake already marked itself done
        // return value handles cancellation of display function, we don't really need that.
        return true;
    };

    auto senderThread = [&]()
    {
        cfg.m_pDisplay->Display(
            communicator,
            g_signalStopRequest,
            handleIncomingNewDevices);

        LOG_VERBOSE(L"[Communicator] Display function ended, signaling other threads to stop..." );
        g_signalStopRequest = true; // signal other threads to stop as well, in case display ends on its own (e.g. video end condition met)
    };

    std::signal(SIGINT, [](int)
                { g_signalStopRequest = true; });
    std::signal(SIGTERM, [](int)
                { g_signalStopRequest = true; });

    if (!cfg.m_pDisplay->Initialize())
    {
        LOG_ERROR(L"Failed to initialize display: " + WStr(cfg.m_pDisplay->GetName()));
        return EXIT_FAILURE;
    }

    SYSTEMD_NOTIFY_READY;

    Thread tFinder(finderThread);
    Thread tConnector(connectorThread);

    // Await initial start to have at least one device connected
    {
        UniqueLock lock(connectedDevicesUpdatedMtx);
        cvConnectedDevicesUpdated.wait(lock, [&]()
                                       { return !connectedHandshakeDevicesPass.empty() || g_signalStopRequest.load(); });
    }

    if (g_signalStopRequest)
    {
        LOG(L"[Main] Stop signal received before starting sender thread, exiting..." );
        tFinder.join();
        tConnector.join();

        SYSTEMD_NOTIFY_STOPPING;
        return EXIT_SUCCESS;
    }

    LOG(L"[Main] Found device. Starting sender thread..." );
    cfg.m_pDisplay->Reset();
    Thread tSender(senderThread);

    // joins here
    tFinder.join();
    tConnector.join();
    tSender.join();

    // ── Shut down audio capture ──────────────────────────────────────────
    audioProcessor.Shutdown();
    // ─────────────────────────────────────────────────────────────────────

    // shutdown
    SYSTEMD_NOTIFY_STOPPING;
    cfg.m_pDisplay->Shutdown(communicator);
    return EXIT_SUCCESS;
}
