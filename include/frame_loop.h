#pragma once

#include "frame_timing.h"

#include <algorithm>
#include <cstdio>
#include <sstream>

struct FrameLoopOptions {
    bool overlap = false;
    bool requested_overlap = false;
    std::string fallback_reason;
    int margin_us = 500;
    int settle_ms = 0;
    // An FSM schedule (B199): frame k waits frame_settle_ms[k] after its DAC write instead of settle_ms, and its timing
    // row records it. The loop is then sequential whatever overlap says (fsm_schedule_sequential_reason). Empty: every
    // frame waits settle_ms.
    std::vector<int> frame_settle_ms;
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

// In overlap the trigger also waits for the previous frame's transfer, so a short settle would not be the real wait: the
// probe of B199 measures settles of 2-20 ms. The summary's fallback_reason when overlap was requested.
constexpr const char* fsm_schedule_sequential_reason =
    "FSM schedule: sequential loop, each frame waits its own settle after its DAC write";

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
    // get_string already drops an inline "# comment" from config lines; a command-line value is passed as given, so
    // drop one here too.
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
// command returns false when a DAC write failed (it is then repeated once).
// finish saves timing and stops acquisition after every handle is returned, including on failure.
// A settle per frame (options.frame_settle_ms) must have one value >= 0 per frame: otherwise nothing is commanded. It runs
// the sequential loop: requested overlap falls back (fsm_schedule_sequential_reason).
template<class IO>
bool run_frame_loop(std::size_t count, FrameLoopOptions options, IO& io,
                    std::vector<FrameTiming>& rows)
{
    options.requested_overlap = options.requested_overlap || options.overlap;
    if (options.overlap && !options.frame_settle_ms.empty()) {
        options.overlap = false;
        options.fallback_reason = fsm_schedule_sequential_reason;
    }
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
    const bool per_frame = !options.frame_settle_ms.empty();
    if (per_frame && options.frame_settle_ms.size() != count) {
        std::fprintf(stderr, "FSM schedule: %zu settle values for %zu frames: nothing moved\n",
                     options.frame_settle_ms.size(), count);
        return finish(false);
    }
    if (per_frame && *std::min_element(options.frame_settle_ms.begin(), options.frame_settle_ms.end()) < 0) {
        std::fprintf(stderr, "FSM schedule: a negative settle: nothing moved\n");
        return finish(false);
    }
    auto settle = [&](std::size_t k) { return per_frame ? options.frame_settle_ms[k] : options.settle_ms; };
    // A failed DAC write is repeated once, immediately. dac_start_ms stays the first attempt's start; dac_done_ms
    // follows the last attempt, so the settle wait counts from the write that landed. A second failure stops the scan.
    auto command = [&](std::size_t k) {
        rows.push_back(FrameTiming());
        rows[k].image_index = k;
        if (per_frame)
            rows[k].settle_ms = options.frame_settle_ms[k];
        rows[k].dac_start_ms = io.now_ms();
        bool written = io.command(k);
        if (!written) {
            rows[k].dac_retried = true;
            written = io.command(k);
        }
        rows[k].dac_done_ms = io.now_ms();
        if (!written) {
            rows[k].dac_failed = true;
            std::fprintf(stderr, "FSM DAC write failed twice at image %zu: stopping; "
                                 "check the LabJack U3 USB connection\n", k);
        }
        return written;
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
        if (options.overlap) {
            // Command k failed after frame k - 1's exposure: keep that frame, as a failed trigger k would.
            if (rows[k].dac_failed) {
                if (held)
                    write(k - 1, held);
                return finish(false);
            }
            io.sleep_until_ms(rows[k].dac_done_ms + settle(k));
        } else {
            if (!command(k))
                return finish(false);
            io.sleep_for_ms(settle(k)); // Preserve the sequential mode's relative wait.
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
                command(k + 1); // A failed write stops the scan at the next iteration, after frame k is saved.
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
