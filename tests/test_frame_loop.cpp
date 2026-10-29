#include "frame_loop.h"

#include <cassert>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <set>

namespace {

struct FakeIO {
    struct Image { std::size_t index; };
    typedef Image* Buffer;
    std::vector<Image> images;
    std::vector<std::string> events;
    std::vector<std::size_t> written;
    std::set<Buffer> held;
    std::vector<int> pushes;
    std::vector<FrameTiming> saved;
    FrameLoopOptions saved_options;
    double time = 0;
    double transfer_ms = 12;
    double write_ms = 1.4;
    double ready = 0;
    int timeout_at = -1, write_failure_at = -1, trigger_failure_at = -1, incomplete_at = -1;
    int finishes = 0;
    bool finished_ok = false;

    explicit FakeIO(std::size_t count) : images(count), pushes(count, 0) {
        for (std::size_t k = 0; k < count; ++k) images[k].index = k;
    }
    void event(const char* name, std::size_t k) { events.push_back(std::string(name) + std::to_string(k)); }
    double now_ms() { return time; }
    void sleep_for_ms(int delay) { events.push_back("sleep_for"); time += delay; }
    void sleep_until_ms(double deadline) {
        assert(std::isfinite(deadline));
        events.push_back("sleep_until");
        time = std::max(time, deadline);
    }
    void command(std::size_t k) { event("dac", k); time += 1.18; }
    bool trigger(std::size_t k) {
        event("trigger", k);
        time += 0.28;
        ready = time + transfer_ms;
        return static_cast<int>(k) != trigger_failure_at;
    }
    Buffer pop(std::size_t k) {
        event("pop", k);
        time = std::max(time, ready);
        if (static_cast<int>(k) == timeout_at) return NULL;
        Buffer buffer = &images[k];
        assert(held.insert(buffer).second);
        return buffer;
    }
    void describe(Buffer buffer, FrameTiming& row) {
        row.frame_id = buffer->index + 10;
        row.camera_timestamp_ns = static_cast<std::uint64_t>((ready - transfer_ms) * 1000000);
    }
    bool complete(Buffer buffer) { return static_cast<int>(buffer->index) != incomplete_at; }
    bool write(std::size_t k, Buffer buffer) {
        assert(buffer->index == k && held.count(buffer));
        event("write", k);
        time += write_ms;
        if (static_cast<int>(k) == write_failure_at) return false;
        assert(std::find(written.begin(), written.end(), k) == written.end());
        written.push_back(k);
        return true;
    }
    void progress(std::size_t k, Buffer) { event("progress", k); }
    void push(Buffer buffer) {
        assert(held.erase(buffer) == 1);
        assert(++pushes[buffer->index] == 1);
        event("push", buffer->index);
    }
    void finish(std::vector<FrameTiming>& rows, bool success, const FrameLoopOptions& effective_options) {
        assert(held.empty());
        events.push_back("save_timing");
        saved = rows;
        saved_options = effective_options;
        events.push_back("stop");
        ++finishes;
        finished_ok = success;
    }
};

std::size_t position(const FakeIO& io, const std::string& event)
{
    const auto found = std::find(io.events.begin(), io.events.end(), event);
    assert(found != io.events.end());
    return found - io.events.begin();
}

FrameLoopOptions options(bool overlap)
{
    FrameLoopOptions result;
    result.overlap = overlap;
    result.settle_ms = 5;
    result.exposure_us = 244;
    return result;
}

void check_success(bool overlap, std::size_t count, double transfer = 12, double write = 1.4)
{
    FakeIO io(count);
    io.transfer_ms = transfer;
    io.write_ms = write;
    std::vector<FrameTiming> rows;
    const FrameLoopOptions config = options(overlap);
    assert(run_frame_loop(count, config, io, rows));
    assert(io.saved_options.overlap == overlap && io.saved_options.requested_overlap == overlap);
    assert(io.saved_options.fallback_reason.empty());
    assert(io.finishes == 1 && io.finished_ok && io.saved.size() == count);
    assert(io.written.size() == count && rows.size() == count);
    for (std::size_t k = 0; k < count; ++k) {
        const std::string n = std::to_string(k);
        assert(io.written[k] == k && io.pushes[k] == 1);
        assert(rows[k].trigger_start_ms >= rows[k].dac_done_ms + config.settle_ms - 1e-9);
        assert(rows[k].written_ms >= rows[k].buffer_ms);
        assert(position(io, "dac" + n) < position(io, "trigger" + n));
        assert(position(io, "trigger" + n) < position(io, "pop" + n));
        assert(position(io, "pop" + n) < position(io, "write" + n));
        assert(position(io, "write" + n) < position(io, "push" + n));
        if (k + 1 < count) {
            const std::string next = std::to_string(k + 1);
            assert(position(io, "pop" + n) < position(io, "trigger" + next));
            assert(rows[k + 1].trigger_start_ms >= rows[k].buffer_ms);
            if (overlap) {
                assert(rows[k + 1].dac_start_ms >= rows[k].trigger_done_ms + 0.744 - 1e-9);
                assert(position(io, "trigger" + n) < position(io, "dac" + next));
                assert(position(io, "dac" + next) < position(io, "pop" + n));
                assert(position(io, "trigger" + next) < position(io, "write" + n));
                assert(position(io, "write" + n) < position(io, "pop" + next));
                if (k + 2 < count)
                    assert(position(io, "dac" + std::to_string(k + 2)) < position(io, "write" + n));
            } else {
                assert(position(io, "push" + n) < position(io, "dac" + next));
            }
        }
    }
    if (!overlap) {
        std::vector<std::string> expected;
        for (std::size_t k = 0; k < count; ++k) {
            const std::string n = std::to_string(k);
            const std::string per_frame[] = {"dac" + n, "sleep_for", "trigger" + n, "pop" + n,
                                              "write" + n, "progress" + n, "push" + n};
            expected.insert(expected.end(), per_frame, per_frame + 7);
        }
        expected.push_back("save_timing");
        expected.push_back("stop");
        assert(io.events == expected);
    }
}

void check_failure(bool overlap, int kind)
{
    FakeIO io(5);
    if (kind == 0) io.timeout_at = 2;
    if (kind == 1) io.write_failure_at = 1;
    if (kind == 2) io.trigger_failure_at = 2;
    if (kind == 3) io.incomplete_at = 2;
    if (kind == 4) io.write_failure_at = 4; // final flush
    if (kind == 5) { io.trigger_failure_at = 2; io.write_failure_at = 1; }
    if (kind == 6) io.trigger_failure_at = 0;
    std::vector<FrameTiming> rows;
    assert(!run_frame_loop(5, options(overlap), io, rows));
    assert(io.finishes == 1 && !io.finished_ok && io.held.empty());
    assert(io.saved.size() == rows.size());
    if (kind == 2) {
        assert((io.written == std::vector<std::size_t>{0, 1}));
        assert(std::count(io.events.begin(), io.events.end(), "write1") == 1);
        assert(io.saved[1].written_ms >= io.saved[1].buffer_ms);
        if (overlap) {
            assert(position(io, "trigger2") < position(io, "write1"));
            assert(position(io, "write1") < position(io, "push1"));
            assert(position(io, "push1") < position(io, "save_timing"));
        }
    }
    if (kind == 5 && overlap) {
        assert(io.written == std::vector<std::size_t>{0});
        assert(position(io, "trigger2") < position(io, "write1"));
        assert(std::count(io.events.begin(), io.events.end(), "write1") == 1);
        assert(io.saved[1].written_ms == -1 && io.pushes[1] == 1);
    }
    if (kind == 6)
        assert(io.written.empty() && rows.size() == 1);
    for (std::size_t k = 0; k < rows.size(); ++k) {
        assert(io.pushes[k] == (rows[k].buffer_ms >= 0 ? 1 : 0));
        assert(io.saved[k].written_ms == rows[k].written_ms);
        const bool written = std::find(io.written.begin(), io.written.end(), k) != io.written.end();
        assert(written == (rows[k].written_ms >= 0));
    }
    assert(position(io, "save_timing") + 1 == position(io, "stop"));
}

void check_config()
{
    FrameLoopOptions config;
    std::string error;
    assert(parse_frame_loop_options("", "", config, error));
    assert(!config.overlap && !config.requested_overlap && config.margin_us == 500 && error.empty());
    assert(parse_frame_loop_options("1", "0", config, error) && config.overlap && config.margin_us == 0);
    assert(config.requested_overlap);
    assert(parse_frame_loop_options("0", "100000", config, error) && !config.overlap && config.margin_us == 100000);
    assert(!config.requested_overlap);
    // An inline comment after the value (a command-line value is passed as given) is ignored.
    assert(parse_frame_loop_options("1               # B192 test", "250 # us", config, error));
    assert(config.overlap && config.margin_us == 250);
    const char* invalid_modes[] = {"2", "-1", "0.5", "true", "1x", "999999999999999999999999", " "};
    for (const char* value : invalid_modes) {
        assert(!parse_frame_loop_options(value, "", config, error));
        assert(error == "FSM_OVERLAP must be 0 or 1");
    }
    const char* invalid_margins[] = {"-1", "100001", "1.5", "abc", "500us", "999999999999999999999999", " "};
    for (const char* value : invalid_margins) {
        assert(!parse_frame_loop_options("", value, config, error));
        assert(error == "FSM_OVERLAP_MARGIN_US must be between 0 and 100000");
    }
}

void check_exposure_diagnostics()
{
    const char* reasons[] = {"cannot read camera exposure: invalid ExposureTime",
                             "camera exposure auto is not known to be off: ExposureAuto enabled",
                             "camera sequencer is not known to be off: On",
                             "camera sequencer is not known to be off: unreadable SequencerMode",
                             "camera TriggerDelay is not known to be zero: 100.000000"};
    for (const char* reason : reasons) {
        const std::string fallback = frame_exposure_diagnostic(true, reason);
        assert(fallback == std::string("Warning: ") + reason + "; using sequential frame loop");
        const std::string sequential = frame_exposure_diagnostic(false, reason);
        assert(sequential == std::string("Note: ") + reason + "; exposure not fixed: slack checks skipped");
        assert(sequential.find("using sequential") == std::string::npos);
    }
}

bool near(double a, double b) { return std::abs(a - b) < 1e-8; }

std::vector<FrameTiming> synthetic_rows()
{
    std::vector<FrameTiming> rows(3);
    for (std::size_t k = 0; k < rows.size(); ++k) {
        rows[k].image_index = k;
        rows[k].frame_id = 100 + k;
        rows[k].camera_timestamp_ns = 1006000000ULL + k * 20000000ULL;
        rows[k].dac_start_ms = k == 0 ? 0 : 7 + (k - 1) * 20;
        rows[k].dac_done_ms = rows[k].dac_start_ms + 1;
        rows[k].trigger_start_ms = 5.8 + k * 20;
        rows[k].trigger_done_ms = 6 + k * 20;
        rows[k].buffer_ms = 18 + k * 20;
        rows[k].written_ms = 28 + k * 20;
    }
    return rows;
}

std::string summary_json(const FrameTimingSummary& summary)
{
    const std::string path = "/tmp/tdm_frame_loop_context.json";
    std::string error;
    assert(write_frame_timing_summary(path, summary, error));
    std::ifstream input(path.c_str());
    const std::string json((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    std::remove(path.c_str());
    return json;
}

void check_invalid_overlap_exposure()
{
    FakeIO sequential(3);
    std::vector<FrameTiming> sequential_rows;
    assert(run_frame_loop(3, options(false), sequential, sequential_rows));
    const double invalid[] = {0, -1, NAN, INFINITY, -INFINITY};
    for (double exposure : invalid) {
        FakeIO io(3);
        auto config = options(true);
        config.exposure_us = exposure;
        std::vector<FrameTiming> rows;
        assert(run_frame_loop(3, config, io, rows));
        assert(io.events == sequential.events && io.written == sequential.written && io.pushes == sequential.pushes);
        assert(!io.saved_options.overlap && io.saved_options.requested_overlap);
        assert(io.saved_options.fallback_reason.find("finite positive exposure_us") != std::string::npos);
        auto summary = check_frame_timing(rows, io.saved_options);
        const std::string json = summary_json(summary);
        assert(json.find("\"requested_mode\": \"overlap\"") != std::string::npos);
        assert(json.find("\"mode\": \"sequential\"") != std::string::npos);
        assert(json.find("\"fallback_reason\": \"invalid exposure:") != std::string::npos);
        if (!std::isfinite(exposure))
            assert(json.find("\"exposure_us\": null") != std::string::npos);
    }
}

void check_summary_context()
{
    for (bool overlap : {false, true}) {
        auto config = options(overlap);
        config.settle_ms = 7;
        config.exposure_us = 321;
        config.margin_us = 1000;
        auto rows = synthetic_rows();
        auto summary = check_frame_timing(rows, config);
        const std::string json = summary_json(summary);
        const std::string mode = overlap ? "overlap" : "sequential";
        assert(json.find("\"mode\": \"" + mode + "\"") != std::string::npos);
        assert(json.find("\"requested_mode\": \"" + mode + "\"") != std::string::npos);
        assert(json.find("\"fallback_reason\": null") != std::string::npos);
        assert(json.find("\"settle_ms\": 7") != std::string::npos);
        assert(json.find("\"exposure_us\": 321.000000") != std::string::npos);
        assert(json.find("\"margin_us\": 1000") != std::string::npos);
    }
    auto config = options(false);
    config.requested_overlap = true;
    config.fallback_reason = "cannot read \"TriggerDelay\": device\\error\n";
    auto rows = synthetic_rows();
    const std::string json = summary_json(check_frame_timing(rows, config));
    assert(json.find("\"requested_mode\": \"overlap\"") != std::string::npos);
    assert(json.find("\"mode\": \"sequential\"") != std::string::npos);
    assert(json.find("\"fallback_reason\": \"cannot read \\\"TriggerDelay\\\": device\\\\error\\u000a\"") != std::string::npos);
}

void check_latch_quality()
{
    FrameClockLatch first, last;
    first.valid = last.valid = true;
    first.camera_ns = 1000000000ULL;
    last.camera_ns = 1100000000ULL;
    for (bool overlap : {false, true}) {
        for (double uncertainty : {0.249, 0.25, 0.251, 0.4, 0.499, 0.5, 0.501, 1.5}) {
            first.host_before_ms = -uncertainty;
            first.host_after_ms = uncertainty;
            last.host_before_ms = 100 - uncertainty;
            last.host_after_ms = 100 + uncertainty;
            const auto clock = frame_clock_mapping(first, last, true);
            auto rows = synthetic_rows();
            const auto summary = check_frame_timing(rows, overlap, 5, 244, 500, clock);
            const bool suspect = uncertainty > 0.5;
            assert(summary.latch_uncertain == suspect && summary.suspect == suspect);
            const std::string line = frame_timing_summary_line(summary);
            assert((line.find("note=latch_uncertain") != std::string::npos) == suspect);
            const std::string json = summary_json(summary);
            assert(json.find(std::string("\"latch_uncertain\": ") + (suspect ? "true" : "false")) != std::string::npos);
            assert((json.find("latch checks are inconclusive") != std::string::npos) == suspect);
            if (uncertainty > 0.5) {
                rows.resize(1); // The cap still applies when there is no next-DAC slack.
                assert(check_frame_timing(rows, true, 5, 244, 2000, clock).latch_uncertain);
            }
        }
    }
    auto rows = synthetic_rows();
    rows[0].dac_done_ms = 2; // A slow latch must not hide a 1 ms settle/slack violation behind an OK verdict.
    rows[1].dac_start_ms = 5.244;
    const auto wide_clock = frame_clock_mapping(first, last, true);
    auto summary = check_frame_timing(rows, false, 5, 244, 500, wide_clock);
    assert(summary.latch_uncertain && summary.suspect && near(summary.min_slack_ms, -1));
    assert(!summary.unsettled_frames && !summary.negative_slack_frames);

    const FrameClockLatch missing;
    const FrameClockMapping mappings[] = {frame_clock_mapping(missing, missing),
        frame_clock_mapping(missing, missing, true), frame_clock_mapping(first, missing, true),
        frame_clock_mapping(missing, last, true), frame_clock_mapping(first, first, true)};
    for (std::size_t i = 0; i < 5; ++i) {
        rows = synthetic_rows();
        summary = check_frame_timing(rows, false, 5, 244, 500, mappings[i]);
        const std::string status = i == 0 ? "absent" : "failed";
        assert(!mappings[i].valid && summary.suspect == (i != 0));
        assert(frame_timing_summary_line(summary).find(" latch=" + status) != std::string::npos);
        assert(summary_json(summary).find("\"latch\": \"" + status + "\"") != std::string::npos);
        assert(std::isnan(rows[0].exposure_start_host_ms));
    }
}

void check_slack_uncertainty()
{
    FrameClockMapping clock;
    clock.valid = true;
    clock.first.camera_ns = 1000000000ULL;
    clock.uncertainty_ms = 0.4;
    for (bool overlap : {false, true}) {
        for (double slack : {0.0, 0.1, 0.399, 0.401}) {
            auto rows = synthetic_rows();
            rows[1].dac_start_ms = 6.25 + slack;
            // Zero margin isolates latch quality from the loop's overlap rule.
            const auto summary = check_frame_timing(rows, overlap, 5, 250, 0, clock);
            assert(near(summary.min_slack_ms, slack));
            assert(summary.latch_uncertain == (slack < clock.uncertainty_ms));
            assert(summary.suspect == summary.latch_uncertain);
            assert(!summary.negative_slack_frames && !summary.overlap_violations && !summary.unsettled_frames);
        }
        // Equality is conclusive (binary-exact values avoid rounding across the boundary).
        auto rows = synthetic_rows();
        clock.uncertainty_ms = 0.5;
        rows[1].dac_start_ms = 6.75;
        const auto summary = check_frame_timing(rows, overlap, 5, 250, 500, clock);
        assert(near(summary.min_slack_ms, clock.uncertainty_ms));
        assert(!summary.latch_uncertain && !summary.suspect);
        rows[1].dac_start_ms = 5.75; // Exactly -u is inconclusive, not a proven violation.
        const auto boundary = check_frame_timing(rows, false, 5, 250, 500, clock);
        assert(near(boundary.min_slack_ms, -clock.uncertainty_ms));
        assert(boundary.latch_uncertain && boundary.suspect && !boundary.negative_slack_frames);
        clock.uncertainty_ms = 0.4;
    }
    const double invalid_exposures[] = {0, NAN, INFINITY};
    for (double exposure : invalid_exposures) {
        auto rows = synthetic_rows();
        const auto summary = check_frame_timing(rows, false, 5, exposure, 500, clock);
        assert(std::isnan(summary.min_slack_ms));
        assert(!summary.latch_uncertain && !summary.suspect);
    }
}

FrameClockLatch recorded_latch(std::uint64_t camera_ns, double before_ms, double after_ms)
{
    FrameClockLatch latch;
    latch.valid = true;
    latch.camera_ns = camera_ns;
    latch.host_before_ms = before_ms;
    latch.host_after_ms = after_ms;
    return latch;
}

void check_real_timing_replays()
{
    // Unmodified schema-2 CSVs and latch windows from frame_timing_summary.json in:
    // ~/Documents/acquisitions/20261029_094638/background/000_20261029_094812/acquis/
    // ~/Documents/acquisitions/20261029_095359/background/000_20261029_100403_633nm/acquis/
    // ~/Documents/acquisitions/20261029_095359/background/002_20261029_100423_488nm/acquis/
    struct Replay {
        const char* path;
        int settle_ms;
        double exposure_us;
        FrameClockLatch first, last;
    };
    const Replay replays[] = {
        {"tests/fixtures/frame_timing_20261029_094812.csv", 10, 380,
         recorded_latch(1796563079453673ULL, 0.001046, 0.680537),
         recorded_latch(1796568354389997ULL, 5274.974296, 5275.145752)},
        {"tests/fixtures/frame_timing_20261029_100403_633nm.csv", 5, 326,
         recorded_latch(1797514428185208ULL, 0.002628, 0.361268),
         recorded_latch(1797520817183166ULL, 6389.011181, 6389.225117)},
        {"tests/fixtures/frame_timing_20261029_100423_488nm.csv", 5, 4575,
         recorded_latch(1797534852169995ULL, 0.004805, 0.195373),
         recorded_latch(1797542750144007ULL, 7897.833191, 7898.049090)}
    };
    for (std::size_t i = 0; i < 3; ++i) {
        const Replay& replay = replays[i];
        std::ifstream input(replay.path);
        assert(input.good());
        std::string line;
        std::getline(input, line); // Header; derived CSV columns are deliberately recomputed.
        assert(line.find("schema_version,image_index,settle_ms,") == 0);
        std::vector<FrameTiming> rows;
        while (std::getline(input, line)) {
            FrameTiming row;
            int schema, settle;
            unsigned long long frame_id, camera_ns, system_ns;
            assert(std::sscanf(line.c_str(), "%d,%zu,%d,%lf,%lf,%lf,%lf,%lf,%lf,%llu,%llu,%llu",
                               &schema, &row.image_index, &settle, &row.dac_start_ms, &row.dac_done_ms,
                               &row.trigger_start_ms, &row.trigger_done_ms, &row.buffer_ms, &row.written_ms,
                               &frame_id, &camera_ns, &system_ns) == 12);
            assert(schema == 2 && settle == replay.settle_ms);
            row.frame_id = frame_id;
            row.camera_timestamp_ns = camera_ns;
            row.system_timestamp_ns = system_ns;
            rows.push_back(row);
        }
        assert(input.eof() && rows.size() == 400);
        const auto clock = frame_clock_mapping(replay.first, replay.last, true);
        assert(clock.valid);
        const auto summary = check_frame_timing(rows, true, replay.settle_ms, replay.exposure_us, 500, clock);
        assert(!summary.suspect && !summary.latch_uncertain && summary.frames == 400);
        assert(near(summary.spacing_dev_max_ms, 0) && summary.min_slack_ms > clock.uncertainty_ms);
        double old_spacing_max_ms = 0;
        double start_spacing_max_ms = 0;
        for (std::size_t k = 1; k < rows.size(); ++k) {
            const double deviation = camera_delta_ms(rows[k].camera_timestamp_ns, rows[k - 1].camera_timestamp_ns) -
                                     (rows[k].trigger_done_ms - rows[k - 1].trigger_done_ms);
            old_spacing_max_ms = std::max(old_spacing_max_ms, std::abs(deviation));
            const double start_deviation = camera_delta_ms(rows[k].camera_timestamp_ns, rows[k - 1].camera_timestamp_ns) -
                                           (rows[k].trigger_start_ms - rows[k - 1].trigger_start_ms);
            start_spacing_max_ms = std::max(start_spacing_max_ms, std::abs(start_deviation));
        }
        if (i == 0) {
            assert(clock.uncertainty_ms > 0.25 && clock.uncertainty_ms < 0.5);
            assert(old_spacing_max_ms < 0.5); // Previously suspect solely from the overlap latch rule.
        } else if (i == 1) {
            assert(clock.uncertainty_ms < 0.25);
            assert(old_spacing_max_ms > 17); // Previously suspect solely from delayed trigger returns.
        } else {
            assert(start_spacing_max_ms > 2.7); // Exposure moved inside a long trigger call.
        }
    }
}

void check_timing()
{
    assert(near(camera_delta_ms(1000000ULL, 2000000ULL), -1));
    assert(near(camera_delta_ms(2000000ULL, 1000000ULL), 1));
    FrameClockLatch first, last;
    first.valid = last.valid = true;
    first.camera_ns = 1000000000ULL;
    first.host_before_ms = -0.1;
    first.host_after_ms = 0.1;
    last.camera_ns = 1100000000ULL;
    last.host_before_ms = 99.8;
    last.host_after_ms = 100.2;
    const FrameClockMapping clock = frame_clock_mapping(first, last);
    assert(clock.valid && near(clock.offset_ms, -1000) && near(clock.drift_ppm, 0));
    assert(near(clock.uncertainty_ms, 0.2) && near(clock.host_ms(1020000000ULL), 20));
    assert(near(clock.host_ms(first.camera_ns - 1000000ULL), -1));
    auto rows = synthetic_rows();
    auto summary = check_frame_timing(rows, true, 5, 244, 500, clock);
    assert(!summary.suspect && summary.frames == 3 && summary.id_gaps == 0);
    assert(near(rows[0].exposure_start_host_ms, 6) && near(rows[1].exposure_start_host_ms, 26));
    assert(near(summary.min_slack_ms, 0.756) && near(rows[0].next_dac_slack_ms, 0.756));
    assert(std::isnan(rows.back().next_dac_slack_ms));
    assert(near(summary.spacing_dev_median_ms, 0) && near(summary.spacing_dev_max_ms, 0));
    assert(near(summary.trigger_start_delay.median_ms, 0.2) && near(summary.trigger_start_delay.min_ms, 0.2) &&
           near(summary.trigger_start_delay.max_ms, 0.2));
    assert(near(summary.trigger_done_delay.median_ms, 0) && near(summary.trigger_done_delay.min_ms, 0) &&
           near(summary.trigger_done_delay.max_ms, 0) && !summary.timestamp_meaning_suspect);

    rows[1].trigger_done_ms += 17;
    summary = check_frame_timing(rows, false, 5, 244, 500, clock);
    assert(!summary.suspect && near(summary.spacing_dev_max_ms, 0));
    // Exposure can move inside a long trigger call without a clock mapping.
    rows = synthetic_rows();
    rows[1].camera_timestamp_ns += 2700000;
    rows[1].trigger_done_ms += 4.75;
    summary = check_frame_timing(rows, false, 5, 244, 500);
    assert(!summary.suspect && near(summary.spacing_dev_max_ms, 0));
    // Camera spacing outside the trigger-call intervals still warns.
    rows = synthetic_rows();
    rows[1].camera_timestamp_ns += 800000;
    summary = check_frame_timing(rows, false, 5, 244, 500);
    assert(summary.suspect && near(summary.spacing_dev_max_ms, 0.6));

    rows = synthetic_rows();
    rows[1].frame_id += 1; // skipped id, followed by a duplicate
    summary = check_frame_timing(rows, true, 5, 244, 500, clock);
    assert(summary.suspect && summary.id_gaps == 2 && summary.id_gap_after_images[0] == 0);
    rows = synthetic_rows();
    rows[1].camera_timestamp_ns += 2000000;
    summary = check_frame_timing(rows, false, 5, 244, 500);
    assert(summary.suspect && near(summary.spacing_dev_max_ms, 1.8) && near(summary.spacing_dev_median_ms, 0));
    assert(std::isnan(summary.min_slack_ms) && std::isnan(rows[0].exposure_start_host_ms));
    assert(std::isnan(summary.trigger_start_delay.median_ms) && std::isnan(summary.trigger_done_delay.min_ms));
    rows[2].camera_timestamp_ns = rows[1].camera_timestamp_ns;
    assert(check_frame_timing(rows, false, 5, 244, 500).timestamp_errors == 1);
    rows = synthetic_rows();
    rows[1].camera_timestamp_ns = rows[0].camera_timestamp_ns - 1000000ULL;
    rows[2].camera_timestamp_ns = rows[1].camera_timestamp_ns + 20000000ULL;
    summary = check_frame_timing(rows, false, 5, 244, 500);
    assert(summary.timestamp_errors == 1 && near(summary.spacing_dev_max_ms, 20.8));
    assert(near(summary.spacing_dev_median_ms, -10.4));

    rows = synthetic_rows();
    rows[1].dac_start_ms = 6.2;
    summary = check_frame_timing(rows, true, 5, 244, 500, clock);
    assert(summary.suspect && summary.overlap_violations == 1 && summary.negative_slack_frames == 0);
    assert(summary.negative_slack_images.empty());
    assert(near(summary.min_slack_ms, -0.044));
    rows = synthetic_rows();
    rows[0].dac_done_ms = 1.3;
    summary = check_frame_timing(rows, true, 5, 244, 500, clock);
    assert(summary.suspect && summary.unsettled_frames == 1);
    assert(summary.unsettled_images == std::vector<std::size_t>{0});
    // Slack within uncertainty is inconclusive; a proven negative slack stays a violation.
    for (double deficit : {0.199, 0.201}) {
        rows = synthetic_rows();
        rows[0].dac_done_ms = 1 + deficit;
        summary = check_frame_timing(rows, false, 5, 244, 500, clock);
        assert(summary.unsettled_frames == (deficit > 0.2 ? 1U : 0U));
        assert(summary.suspect == (deficit > 0.2));
        rows = synthetic_rows();
        rows[1].dac_start_ms = 6.244 - deficit;
        summary = check_frame_timing(rows, false, 5, 244, 500, clock);
        assert(near(summary.min_slack_ms, -deficit));
        assert(summary.negative_slack_frames == (deficit > 0.2 ? 1U : 0U));
        assert(summary.latch_uncertain == (deficit <= 0.2) && summary.suspect);
        assert((frame_timing_summary_line(summary).find("note=latch_uncertain") != std::string::npos) ==
               (deficit <= 0.2));
    }
    rows = synthetic_rows();
    const double start_delays[] = {0.1, 0.4, 0.2};
    const double done_delays[] = {-0.1, -0.2, -0.3};
    for (std::size_t k = 0; k < rows.size(); ++k) {
        const double exposure_start = 6 + k * 20;
        rows[k].trigger_start_ms = exposure_start - start_delays[k];
        rows[k].trigger_done_ms = exposure_start - done_delays[k];
    }
    summary = check_frame_timing(rows, false, 5, 244, 500, clock);
    assert(!summary.suspect && near(summary.trigger_start_delay.median_ms, 0.2));
    assert(near(summary.trigger_start_delay.min_ms, 0.1) && near(summary.trigger_start_delay.max_ms, 0.4));
    assert(near(summary.trigger_done_delay.median_ms, -0.2) && near(summary.trigger_done_delay.min_ms, -0.3) &&
           near(summary.trigger_done_delay.max_ms, -0.1));
    assert(near(frame_delay_statistics(std::vector<double>{0.1, 0.3}).median_ms, 0.2));
    // Median delay from trigger start must be within [-uncertainty, median call duration + uncertainty + 1 ms].
    const int timestamp_shifts_ns[] = {-401000, -399000, 1199000, 1201000};
    for (int shift : timestamp_shifts_ns) {
        rows = synthetic_rows();
        rows.resize(1);
        rows[0].dac_done_ms = 0; // Isolate the timestamp-meaning check from settling and next-DAC slack.
        rows[0].camera_timestamp_ns = static_cast<std::uint64_t>(1006000000LL + shift);
        summary = check_frame_timing(rows, false, 5, 244, 500, clock);
        const bool suspect = shift == -401000 || shift == 1201000;
        assert(summary.timestamp_meaning_suspect == suspect && summary.suspect == suspect);
        assert(!summary.unsettled_frames && !summary.negative_slack_frames);
        assert((frame_timing_summary_line(summary).find("slack_checks_untrustworthy") != std::string::npos) == suspect);
    }
    rows = synthetic_rows();
    assert(check_frame_timing(rows, false, 5, 244, 500, clock, false).suspect);
    rows[2].buffer_ms = -1;
    summary = check_frame_timing(rows, true, 5, 244, 500, clock);
    assert(summary.suspect && summary.frames == 2 && summary.id_gaps == 0);
    std::vector<FrameTiming> empty;
    assert(!check_frame_timing(empty, true, 5, 244, 500).suspect);

    last.host_before_ms += 0.01;
    last.host_after_ms += 0.01;
    const auto drifting = frame_clock_mapping(first, last);
    assert(drifting.valid && near(drifting.drift_ppm, 100));
    assert(near(drifting.host_ms(1050000000ULL), 50.005));
    last.camera_ns = first.camera_ns;
    assert(!frame_clock_mapping(first, last).valid);
    last.valid = false;
    assert(!frame_clock_mapping(first, last).valid);

    rows = synthetic_rows();
    summary = check_frame_timing(rows, true, 5, 244, 500, clock);
    assert(frame_timing_summary_line(summary) ==
           "[frame-timing] mode=overlap frames=3 id_gaps=0 spacing_dev_max_ms=0.000000 "
           "min_slack_ms=0.756000 trigger_start_delay_median_ms=0.200000 trigger_start_delay_min_ms=0.200000 "
           "trigger_start_delay_max_ms=0.200000 trigger_done_delay_median_ms=0.000000 "
           "trigger_done_delay_min_ms=0.000000 trigger_done_delay_max_ms=0.000000 "
           "latch_uncertainty_ms=0.200000 latch=ok verdict=ok");
    const std::string path = "/tmp/tdm_frame_loop_summary.json";
    std::string error;
    assert(write_frame_timing_summary(path, summary, error));
    std::ifstream input(path.c_str());
    const std::string json((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    assert(json.find("\"min_slack_ms\": 0.756000") != std::string::npos);
    assert(json.find("\"latch\": \"ok\"") != std::string::npos);
    assert(json.find("\"verdict\": \"ok\"") != std::string::npos);
    assert(json.find("\"latch_before\": {\"camera_timestamp_ns\": 1000000000") != std::string::npos);
    assert(json.find("\"host_before_ms\": -0.100000, \"host_after_ms\": 0.100000") != std::string::npos);
    assert(json.find("\"trigger_start_delay_median_ms\": 0.200000") != std::string::npos);
    assert(json.find("\"trigger_start_delay_min_ms\": 0.200000") != std::string::npos);
    assert(json.find("\"trigger_start_delay_max_ms\": 0.200000") != std::string::npos);
    assert(json.find("\"trigger_done_delay_median_ms\": 0.000000") != std::string::npos);
    assert(json.find("\"trigger_done_delay_min_ms\": 0.000000") != std::string::npos);
    assert(json.find("\"trigger_done_delay_max_ms\": 0.000000") != std::string::npos);
    assert(json.find("\"latch_uncertainty_ms\": 0.200000") != std::string::npos);
    assert(json.find("\"timestamp_meaning_suspect\": false") != std::string::npos);
    std::remove(path.c_str());
    summary = check_frame_timing(rows, false, 5, 244, 500);
    assert(write_frame_timing_summary(path, summary, error));
    std::ifstream absent_input(path.c_str());
    const std::string absent_json((std::istreambuf_iterator<char>(absent_input)), std::istreambuf_iterator<char>());
    assert(absent_json.find("\"min_slack_ms\": null") != std::string::npos);
    assert(absent_json.find("\"latch\": \"absent\"") != std::string::npos);
    assert(absent_json.find("\"latch_before\": null") != std::string::npos);
    assert(absent_json.find("\"trigger_start_delay_median_ms\": null") != std::string::npos);
    assert(absent_json.find("\"trigger_done_delay_median_ms\": null") != std::string::npos);
    assert(absent_json.find("\"latch_uncertainty_ms\": null") != std::string::npos);
    std::remove(path.c_str());
    assert(!write_frame_timing_summary("/nonexistent-dir/summary.json", summary, error));
    check_frame_timing(rows, true, 5, 244, 500, clock);
    const std::string csv_path = "/tmp/tdm_frame_loop.csv";
    assert(write_frame_timing_csv(csv_path, rows, 5, error));
    std::ifstream csv(csv_path.c_str());
    std::string line;
    std::getline(csv, line);
    std::getline(csv, line);
    assert(line.substr(line.size() - 18) == ",6.000000,0.756000");
    std::getline(csv, line);
    std::getline(csv, line);
    assert(line.substr(line.size() - 11) == ",46.000000,");
    std::remove(csv_path.c_str());
    rows.resize(1);
    rows[0].camera_timestamp_ns += 2000000ULL;
    summary = check_frame_timing(rows, false, 5, 244, 500, clock);
    assert(write_frame_timing_summary(path, summary, error));
    std::ifstream suspect_input(path.c_str());
    const std::string suspect_json((std::istreambuf_iterator<char>(suspect_input)), std::istreambuf_iterator<char>());
    assert(suspect_json.find("\"timestamp_meaning_suspect\": true") != std::string::npos);
    assert(suspect_json.find("timestamp meaning is not the exposure start; slack checks are not trustworthy") != std::string::npos);
    std::remove(path.c_str());
}

} // namespace

int main()
{
    check_config();
    check_exposure_diagnostics();
    check_invalid_overlap_exposure();
    check_summary_context();
    check_latch_quality();
    check_slack_uncertainty();
    check_real_timing_replays();
    for (bool overlap : {false, true}) {
        check_success(overlap, 0);
        check_success(overlap, 1);
        check_success(overlap, 5);
        check_success(overlap, 5, 0.1, 20); // Transfer shorter than guard; disk longer than transfer.
        for (int kind = 0; kind < 7; ++kind) check_failure(overlap, kind);
    }
    check_timing();
    return 0;
}
