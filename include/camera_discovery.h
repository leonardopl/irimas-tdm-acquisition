#pragma once

#include <map>
#include <string>
#include <vector>

namespace camera_discovery {

// Hardware-free discovery fields; positions correspond to entries in the current discovery pass.
struct Device {
    std::string id, serial, protocol;
    bool matches(const std::string& wanted) const {
        return !wanted.empty() && (id == wanted || (!serial.empty() && serial == wanted));
    }
};

// Preserve first-entry order and the serial+protocol dedup rule; serial-less entries stay separate.
// If a duplicate matches the wanted device id, retain that entry instead of its nonmatching alias.
inline std::vector<unsigned int> deduplicateDevices(const std::vector<Device>& devices,
                                                   const std::string& wanted = "")
{
    std::map<std::string, std::size_t> positions;
    std::vector<unsigned int> retained;
    for (unsigned int i = 0; i < devices.size(); ++i) {
        const Device& device = devices[i];
        const std::string key = device.serial.empty() ? "index:" + std::to_string(i)
            : "serial:" + device.serial + "|" + device.protocol;
        const auto inserted = positions.emplace(key, retained.size());
        if (inserted.second) retained.push_back(i);
        else if (device.matches(wanted) && !devices[retained[inserted.first->second]].matches(wanted))
            retained[inserted.first->second] = i;
    }
    return retained;
}

// Select a matching retained entry, returning its position in retained (not its discovery index).
// On failure selected is unchanged and error names the requested camera; never select a fallback.
inline bool selectDevice(const std::vector<Device>& devices, const std::vector<unsigned int>& retained,
                         const std::string& wanted, unsigned int& selected, std::string& error)
{
    error.clear();
    for (unsigned int i = 0; i < retained.size(); ++i) {
        if (devices[retained[i]].matches(wanted)) {
            selected = i;
            return true;
        }
    }
    error = "selected camera " + wanted + " not found";
    return false;
}

} // namespace camera_discovery

// B252: avoid the roughly 1 s GigE discovery wait when USB3 Vision finds a camera.
// With no requested identity, USB3 cameras take precedence over GigE cameras.
// Each pass returns its deduplicated camera count; network=true includes GigE Vision.
// wanted_found tests the latest pass: an absent requested identity also requires full discovery.
template <class Pass, class Match>
unsigned int DiscoverCameras(Pass pass, Match wanted_found)
{
    const unsigned int count = pass(false);
    return count == 0 || !wanted_found() ? pass(true) : count;
}

template <class Pass>
unsigned int DiscoverCameras(Pass pass)
{
    return DiscoverCameras(pass, []() { return true; });
}
