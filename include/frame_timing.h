#pragma once

// Per-frame timing of an acquisition (qwlsi-gui B192): the host time of each step of the frame loop, in ms from the
// first DAC command, and the camera's own buffer timestamps. Written as acquis/frame_timing.csv after the loop (also
// after a failed frame), for the timing breakdown; nothing reads it back.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <string>
#include <vector>

struct FrameTiming {
    std::size_t image_index = 0;
    double dac_start_ms = 0;        // before the first DAC write
    double dac_done_ms = 0;         // after the second DAC write
    double trigger_start_ms = 0;    // after the settle sleep, before the software trigger
    double trigger_done_ms = 0;     // the software trigger call returned
    double buffer_ms = -1;          // the frame's buffer popped from the stream (-1: none)
    double written_ms = -1;         // the image file written (-1: not written)
    std::uint64_t frame_id = 0;               // the camera's frame id of the buffer
    std::uint64_t camera_timestamp_ns = 0;    // camera clock (arv_buffer_get_timestamp)
    std::uint64_t system_timestamp_ns = 0;    // host clock when the buffer arrived (arv_buffer_get_system_timestamp)
};

inline bool write_frame_timing_csv(const std::string& path, const std::vector<FrameTiming>& rows, int settle_ms,
                                   std::string& error)
{
    std::FILE* file = std::fopen(path.c_str(), "w");
    if (!file) {
        error = path + ": " + std::strerror(errno);
        return false;
    }
    std::fprintf(file, "schema_version,image_index,settle_ms,dac_start_ms,dac_done_ms,trigger_start_ms,"
                       "trigger_done_ms,buffer_ms,written_ms,frame_id,camera_timestamp_ns,system_timestamp_ns\n");
    for (const FrameTiming& row : rows)
        std::fprintf(file, "1,%zu,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%llu,%llu,%llu\n", row.image_index, settle_ms,
                     row.dac_start_ms, row.dac_done_ms, row.trigger_start_ms, row.trigger_done_ms, row.buffer_ms,
                     row.written_ms, (unsigned long long)row.frame_id, (unsigned long long)row.camera_timestamp_ns,
                     (unsigned long long)row.system_timestamp_ns);
    const bool ok = !std::ferror(file);
    if (std::fclose(file) != 0 || !ok) {
        error = path + ": write failed";
        return false;
    }
    return true;
}
