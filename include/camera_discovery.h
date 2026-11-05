#pragma once

// B252: avoid the roughly 1 s GigE discovery wait when USB3 Vision finds a camera.
// With both USB3 and GigE cameras connected, use USB3 and do not list the GigE cameras.
// Each pass returns its deduplicated camera count; network=true includes GigE Vision.
template <class Pass>
unsigned int DiscoverCameras(Pass pass)
{
    const unsigned int count = pass(false);
    return count == 0 ? pass(true) : count;
}
