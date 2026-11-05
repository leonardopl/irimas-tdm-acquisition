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

} // namespace

int main()
{
    check_discovery(1, 2, 1, {false});       // USB camera: skip network discovery.
    check_discovery(0, 2, 2, {false, true}); // No USB camera: return the network pass's count.
    check_discovery(0, 0, 0, {false, true}); // No cameras on either pass.
    return 0;
}
