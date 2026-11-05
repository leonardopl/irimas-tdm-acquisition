#include "camera_discovery.h"

#include <cassert>
#include <vector>

namespace {

void check_discovery(unsigned int usb_count, unsigned int network_count,
                     unsigned int expected_count, const std::vector<bool>& expected_calls)
{
    std::vector<bool> calls;
    const unsigned int count = DiscoverCameras([&](bool network) -> unsigned int {
        calls.push_back(network);
        return network ? network_count : usb_count;
    });
    assert(count == expected_count);
    assert(calls == expected_calls);
}

void check_selection()
{
    using namespace camera_discovery;
    const std::vector<Device> devices = {
        {"other", "SERIAL0", "USB3Vision"},
        {"alias", "SERIAL1", "USB3Vision"},
        {"wanted-id", "SERIAL1", "USB3Vision"}, // Same camera, alternate device id.
        {"network", "SERIAL1", "GigEVision"},  // Another protocol is retained.
        {"no-serial-a", "", "USB3Vision"},
        {"no-serial-b", "", "USB3Vision"}
    };
    const auto defaults = deduplicateDevices(devices);
    assert((defaults == std::vector<unsigned int>{0, 1, 3, 4, 5}));
    for (const std::string wanted : {"wanted-id", "alias", "SERIAL1", "network", "no-serial-b"}) {
        const auto retained = deduplicateDevices(devices, wanted);
        unsigned int selected = 99;
        std::string error = "old error";
        assert(selectDevice(devices, retained, wanted, selected, error) && error.empty());
        assert(devices[retained[selected]].matches(wanted));
        assert(selected != 0); // Never open the unrelated first camera.
        if (wanted == "wanted-id") {
            assert((retained == std::vector<unsigned int>{0, 2, 3, 4, 5}));
            assert(retained[selected] == 2);
        } else {
            assert(retained == defaults);
        }
    }
    // An entry matching before dedup is insufficient: if it wasn't retained, fail explicitly.
    unsigned int selected = 99;
    std::string error;
    assert(!selectDevice(devices, defaults, "wanted-id", selected, error));
    assert(selected == 99 && error == "selected camera wanted-id not found");
    const auto retained = deduplicateDevices(devices, "missing");
    assert(retained == defaults);
    assert(!selectDevice(devices, retained, "missing", selected, error));
    assert(selected == 99 && error == "selected camera missing not found");
    assert(!selectDevice({}, {}, "missing", selected, error));
    assert(selected == 99 && error == "selected camera missing not found");
}

} // namespace

int main()
{
    check_selection();
    check_discovery(1, 2, 1, {false});       // USB camera: skip network discovery.
    check_discovery(0, 2, 2, {false, true}); // No USB camera: return the network pass's count.
    check_discovery(0, 0, 0, {false, true}); // No cameras on either pass.
    for (bool found_on_usb : {false, true}) {
        std::vector<bool> calls;
        bool found = false;
        const unsigned int count = DiscoverCameras([&](bool network) -> unsigned int {
            calls.push_back(network);
            found = network || found_on_usb;
            return network ? 2 : 1;
        }, [&]() { return found; });
        assert(count == (found_on_usb ? 1u : 2u));
        assert(calls == (found_on_usb ? std::vector<bool>{false} : std::vector<bool>{false, true}));
    }
    return 0;
}
