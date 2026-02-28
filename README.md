# Quadcast 2S RGB
Tool to communicate with the HyperX Quadcast 2S's 108 RGB diodes.
HEAVY WIP! Yet, functional. 
Just check the main() function inside the yet not organized Main.cpp,
```cpp
    CUSBDeviceFinder finder;
    HIDDeviceContainer devices;
    do
    {
        devices = finder.FindDevices(g_QUADCAST2S_USB_ID);
    } while (devices.empty());

    CQuadcast2SCommunicator communicator(devices);
    devices.clear();
    if (!communicator.Connect())
    {
        std::cerr << "No viable device interface found after handshake. Exiting." << std::endl;
        return 1;
    }

    UniquePtr<CQC2SDisplay> pDisplay = pMultiDisplay->AddDisplay(
        CQC2SDisplayFactory::CreateSolidColor({0x00, 0xff, 0xff}, "solid1", std::make_unique<CTimeEndCondition>(3s),"solid2"));
    pMultiDisplay->AddDisplay(
        CQC2SDisplayFactory::CreateSolidColor({0x3c, 0x10, 0xff}, "solid2", std::make_unique<CTimeEndCondition>(5s),"pulse1"));
    pMultiDisplay->AddDisplay(
        CQC2SDisplayFactory::CreatePulseColor({0x10, 0x10, 0xc5},0.025,"pulse1",nullptr));

    if (!pDisplay->Initialize(communicator))
        throw std::runtime_error("Failed to initialize display: " + pDisplay->GetName());
    pDisplay->Display(communicator);
    pDisplay->Shutdown(communicator);
```
This example finds all usb device interfaces for the quadcast 2s and communicates with them synchronously and at the same time. The colors displayed here will be #00ffff (aqua) for 3 seconds, #3c10ff (violet/blue tone) and then pulsing permanently on #1010c5 (deep blue) 