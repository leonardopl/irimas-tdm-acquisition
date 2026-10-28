#pragma once

#include "frame_timing.h"

#include <sstream>

struct FrameLoopOptions {
    bool overlap = false;
    bool requested_overlap = false;
    std::string fallback_reason;
    int margin_us = 500;
    int settle_ms = 0;
    double exposure_us = 0;
};

inline FrameTimingSummary check_frame_timing(std::vector<FrameTiming>& rows, const FrameLoopOptions& options,
                                             const FrameClockMapping& clock = FrameClockMapping(),
                                             bool scan_complete = true)
{
    FrameTimingSummary summary = check_frame_timing(rows, options.overlap, options.settle_ms,
                                                    options.exposure_us, options.margin_us, clock, scan_complete);
    summary.requested_overlap = options.requested_overlap || options.overlap;
    summary.fallback_reason = options.fallback_reason;
    return summary;
}

inline std::string frame_exposure_diagnostic(bool overlap_requested, const std::string& reason)
{
    return (overlap_requested ? "Warning: " : "Note: ") + reason +
           (overlap_requested ? "; using sequential frame loop" : "; exposure not fixed: slack checks skipped");
}

inline bool parse_frame_loop_options(const std::string& overlap, const std::string& margin,
                                     FrameLoopOptions& options, std::string& error)
{
    options.overlap = false;
    options.requested_overlap = false;
    options.fallback_reason.clear();
    options.margin_us = 500;
    error.clear();
    // get_string returns the rest of the config line: drop an inline "# comment" as the numeric keys' atoi does.
    const std::string values[] = {overlap.substr(0, overlap.find('#')), margin.substr(0, margin.find('#'))};
    const char* messages[] = {"FSM_OVERLAP must be 0 or 1",
                             "FSM_OVERLAP_MARGIN_US must be between 0 and 100000"};
    for (int i = 0; i < 2; ++i) {
        if (values[i].empty())
            continue;
        std::istringstream input(values[i]);
        int value;
        if (!(input >> value) || !(input >> std::ws).eof() || value < 0 || value > (i == 0 ? 1 : 100000)) {
            error = messages[i];
            return false;
        }
        if (i == 0) options.overlap = value == 1;
        else options.margin_us = value;
    }
    options.requested_overlap = options.overlap;
    return true;
}

// IO owns the hardware and clock. Buffer is a nullable handle, kept until write returns.
// finish saves timing and stops acquisition after every handle is returned, including on failure.
template<class IO>
bool run_frame_loop(std::size_t count, FrameLoopOptions options, IO& io,
                    std::vector<FrameTiming>& rows)
{
    options.requested_overlap = options.requested_overlap || options.overlap;
    if (options.overlap && (!std::isfinite(options.exposure_us) || options.exposure_us <= 0)) {
        options.overlap = false;
        options.fallback_reason = "invalid exposure: overlap requires a finite positive exposure_us";
    }
    typedef typename IO::Buffer Buffer;
    Buffer held = Buffer();
    Buffer current = Buffer();
    rows.clear();
    rows.reserve(count);
    auto finish = [&](bool success) {
        if (current) io.push(current);
        if (held) io.push(held);
        current = held = Buffer();
        io.finish(rows, success, options);
        return success;
    };
    auto command = [&](std::size_t k) {
        rows.push_back(FrameTiming());
        rows[k].image_index = k;
        rows[k].dac_start_ms = io.now_ms();
        io.command(k);
        rows[k].dac_done_ms = io.now_ms();
    };
    auto write = [&](std::size_t k, Buffer buffer) {
        if (!io.write(k, buffer))
            return false;
        rows[k].written_ms = io.now_ms();
        io.progress(k, buffer);
        return true;
    };
    if (options.overlap && count > 0)
        command(0);
    for (std::size_t k = 0; k < count; ++k) {
        if (options.overlap)
            io.sleep_until_ms(rows[k].dac_done_ms + options.settle_ms);
        else {
            command(k);
            io.sleep_for_ms(options.settle_ms); // Preserve the sequential mode's relative wait.
        }
        rows[k].trigger_start_ms = io.now_ms();
        const bool triggered = io.trigger(k);
        rows[k].trigger_done_ms = io.now_ms();
        if (!triggered) {
            if (held)
                write(k - 1, held); // Preserve the previous complete frame even when this trigger fails.
            return finish(false);
        }
        if (options.overlap) {
            io.sleep_until_ms(rows[k].trigger_done_ms + (options.exposure_us + options.margin_us) / 1000.0);
            if (k + 1 < count)
                command(k + 1);
            if (held) {
                if (!write(k - 1, held))
                    return finish(false);
                io.push(held);
                held = Buffer();
            }
        }
        current = io.pop(k);
        if (!current)
            return finish(false);
        rows[k].buffer_ms = io.now_ms();
        io.describe(current, rows[k]);
        if (!io.complete(current))
            return finish(false);
        if (options.overlap) {
            held = current;
            current = Buffer();
        } else {
            if (!write(k, current))
                return finish(false);
            io.push(current);
            current = Buffer();
        }
    }
    if (held && !write(count - 1, held))
        return finish(false);
    return finish(true);
}
