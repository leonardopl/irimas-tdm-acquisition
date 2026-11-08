#include "frame_loop.h"

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <set>
#include <unistd.h>

namespace {

// Only reader round trips need a file; production writers are text builders.
bool write_fixture(const std::string& path, const std::string& text)
{
    std::ofstream out(path.c_str(), std::ios::binary);
    out << text;
    out.close();
    return out.good();
}

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
    int dac_failure_at = -1, dac_failures = 0; // The first dac_failures writes of image dac_failure_at fail.
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
    bool command(std::size_t k) {
        event("dac", k);
        time += 1.18;
        if (static_cast<int>(k) != dac_failure_at || dac_failures == 0)
            return true;
        --dac_failures;
        return false;
    }
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

// The fake camera clock is its host clock in ns: an exact latch, so the settle and slack checks run.
FrameClockMapping fake_clock()
{
    FrameClockLatch first, last;
    first.valid = last.valid = true;
    last.camera_ns = 1000000000000ULL;
    last.host_before_ms = last.host_after_ms = 1000000;
    return frame_clock_mapping(first, last, true);
}

std::string summary_json(const FrameTimingSummary& summary);
std::vector<FrameTiming> synthetic_rows();

bool near(double a, double b) { return std::abs(a - b) < 1e-8; }
bool contains(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

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
    std::vector<std::string> expected;
    if (!overlap) {
        for (std::size_t k = 0; k < count; ++k) {
            const std::string n = std::to_string(k);
            const std::string per_frame[] = {"dac" + n, "sleep_for", "trigger" + n, "pop" + n,
                                              "write" + n, "progress" + n, "push" + n};
            expected.insert(expected.end(), per_frame, per_frame + 7);
        }
    } else {
        if (count)
            expected.push_back("dac0");
        for (std::size_t k = 0; k < count; ++k) {
            const std::string n = std::to_string(k), previous = std::to_string(k - 1);
            const std::string start[] = {"sleep_until", "trigger" + n, "sleep_until"};
            expected.insert(expected.end(), start, start + 3);
            if (k + 1 < count)
                expected.push_back("dac" + std::to_string(k + 1));
            if (k > 0) {
                const std::string flush[] = {"write" + previous, "progress" + previous, "push" + previous};
                expected.insert(expected.end(), flush, flush + 3);
            }
            expected.push_back("pop" + n);
        }
        if (count) {
            const std::string last = std::to_string(count - 1);
            const std::string flush[] = {"write" + last, "progress" + last, "push" + last};
            expected.insert(expected.end(), flush, flush + 3);
        }
    }
    expected.push_back("save_timing");
    expected.push_back("stop");
    assert(io.events == expected);
    // Good path: one write per image and the DAC interval of that one write; the new summary keys are zero.
    for (std::size_t k = 0; k < count; ++k)
        assert(!rows[k].dac_retried && !rows[k].dac_failed && near(rows[k].dac_done_ms - rows[k].dac_start_ms, 1.18));
    const FrameTimingSummary summary = check_frame_timing(rows, io.saved_options, fake_clock());
    assert(!summary.suspect && summary.frames == count);
    assert(summary.dac_write_retries == 0 && summary.dac_retry_images.empty() && !summary.dac_write_failed);
    assert(contains(frame_timing_summary_line(summary), " id_gaps=0 dac_retries=0 spacing_dev_max_ms="));
    const std::string json = summary_json(summary);
    assert(contains(json, "\"dac_write_retries\": 0,") && contains(json, "\"dac_write_failed\": false,"));
    assert(contains(json, "\"dac_retry_images\": []\n}"));
}

// One failed DAC write at image `at`, then a good repeat: the scan completes and the retry is recorded.
void check_dac_retry(bool overlap, std::size_t at)
{
    const std::size_t count = 5;
    const FrameLoopOptions config = options(overlap);
    FakeIO clean(count);
    std::vector<FrameTiming> clean_rows;
    assert(run_frame_loop(count, config, clean, clean_rows));
    FakeIO io(count);
    io.dac_failure_at = static_cast<int>(at);
    io.dac_failures = 1;
    std::vector<FrameTiming> rows;
    assert(run_frame_loop(count, config, io, rows));
    assert(io.finishes == 1 && io.finished_ok && io.held.empty() && rows.size() == count);
    assert((io.written == std::vector<std::size_t>{0, 1, 2, 3, 4}));
    const std::string n = std::to_string(at);
    assert(std::count(io.events.begin(), io.events.end(), "dac" + n) == 2);
    const std::size_t first = position(io, "dac" + n);
    assert(io.events[first + 1] == "dac" + n); // Repeated at once.
    // Apart from the repeat, the order is the good path's.
    std::vector<std::string> events = io.events;
    events.erase(events.begin() + first);
    assert(events == clean.events);
    if (overlap && at > 0) {
        const std::string previous = std::to_string(at - 1);
        assert(position(io, "trigger" + previous) < first && first + 1 < position(io, "pop" + previous));
    }
    for (std::size_t k = 0; k < count; ++k) {
        assert(rows[k].dac_retried == (k == at) && !rows[k].dac_failed);
        // dac_start_ms is the first attempt's start, dac_done_ms the repeat's end; the settle counts from it.
        assert(near(rows[k].dac_done_ms - rows[k].dac_start_ms, k == at ? 2.36 : 1.18));
        assert(rows[k].trigger_start_ms >= rows[k].dac_done_ms + config.settle_ms - 1e-9);
    }
    const FrameTimingSummary summary = check_frame_timing(rows, io.saved_options, fake_clock());
    const FrameTimingSummary clean_summary = check_frame_timing(clean_rows, clean.saved_options, fake_clock());
    assert(summary.dac_write_retries == 1 && summary.dac_retry_images == std::vector<std::size_t>{at});
    assert(!summary.dac_write_failed && summary.frames == count);
    assert(!summary.suspect && summary.suspect == clean_summary.suspect); // A retry alone keeps the verdict.
    assert(summary.unsettled_frames == 0 && summary.negative_slack_frames == 0 && summary.overlap_violations == 0);
    const std::string line = frame_timing_summary_line(summary);
    assert(contains(line, " dac_retries=1 ") && contains(line, " verdict=ok"));
    const std::string json = summary_json(summary);
    assert(contains(json, "\"dac_write_retries\": 1,") && contains(json, "\"dac_write_failed\": false,"));
    assert(contains(json, "\"dac_retry_images\": [" + n + "]") && contains(json, "\"verdict\": \"ok\""));
}

// Both DAC writes of image `at` fail: the scan stops there, like a failed trigger, and the timing is suspect.
void check_dac_failure(bool overlap, std::size_t at)
{
    const std::size_t count = 5;
    FakeIO io(count);
    io.dac_failure_at = static_cast<int>(at);
    io.dac_failures = 2;
    std::vector<FrameTiming> rows;
    // The loop's stderr line is captured and checked.
    // A unique file: two checkouts may run make test at the same time.
    char log_template[] = "/tmp/tdm_frame_loop_stderr_XXXXXX";
    const int log_fd = mkstemp(log_template);
    const std::string log_path = log_template;
    std::fflush(stderr);
    const int saved_stderr = dup(2);
    std::FILE* log = log_fd >= 0 ? fdopen(log_fd, "w") : nullptr;
    assert(saved_stderr >= 0 && log && dup2(fileno(log), 2) == 2);
    const bool completed = run_frame_loop(count, options(overlap), io, rows);
    std::fflush(stderr);
    assert(dup2(saved_stderr, 2) == 2 && close(saved_stderr) == 0 && std::fclose(log) == 0);
    std::ifstream log_input(log_path.c_str());
    const std::string logged((std::istreambuf_iterator<char>(log_input)), std::istreambuf_iterator<char>());
    std::remove(log_path.c_str());
    assert(logged == "FSM DAC write failed twice at image " + std::to_string(at) +
                     ": stopping; check the LabJack U3 USB connection\n");
    assert(!completed && io.finishes == 1 && !io.finished_ok && io.held.empty());
    assert(rows.size() == at + 1 && io.saved.size() == rows.size());
    const std::string n = std::to_string(at);
    assert(std::count(io.events.begin(), io.events.end(), "dac" + n) == 2);
    assert(std::find(io.events.begin(), io.events.end(), "trigger" + n) == io.events.end());
    // Every frame before `at` is saved; in overlap frame at - 1 was exposed before the failed write.
    std::vector<std::size_t> saved;
    for (std::size_t k = 0; k < at; ++k)
        saved.push_back(k);
    assert(io.written == saved);
    for (std::size_t k = 0; k < count; ++k)
        assert(io.pushes[k] == (k < at ? 1 : 0));
    if (overlap && at > 0)
        assert(position(io, "dac" + n) < position(io, "write" + std::to_string(at - 1)));
    for (std::size_t k = 0; k < rows.size(); ++k)
        assert(rows[k].dac_retried == (k == at) && rows[k].dac_failed == (k == at));
    assert(rows[at].buffer_ms < 0 && rows[at].written_ms < 0);
    assert(near(rows[at].dac_done_ms - rows[at].dac_start_ms, 2.36));
    const FrameTimingSummary summary = check_frame_timing(rows, io.saved_options, fake_clock(), false);
    assert(summary.dac_write_failed && summary.suspect && summary.frames == at);
    assert(summary.dac_write_retries == 0 && summary.dac_retry_images.empty());   // the failed retry is not a retry
    assert(contains(frame_timing_summary_line(summary), " dac_retries=0 "));
    const std::string json = summary_json(summary);
    assert(contains(json, "\"dac_write_failed\": true,") && contains(json, "\"verdict\": \"suspect\""));
}

// dac_write_failed alone makes the summary suspect; a retry alone does not.
void check_dac_summary()
{
    auto rows = synthetic_rows();
    assert(!check_frame_timing(rows, false, 5, 244, 500).suspect);
    rows[1].dac_retried = rows[2].dac_retried = true;
    FrameTimingSummary summary = check_frame_timing(rows, true, 5, 244, 500);
    assert(!summary.suspect && summary.dac_write_retries == 2 && !summary.dac_write_failed);
    assert((summary.dac_retry_images == std::vector<std::size_t>{1, 2}));
    assert(contains(summary_json(summary), "\"dac_retry_images\": [1, 2]"));
    rows[2].dac_failed = true;
    summary = check_frame_timing(rows, false, 5, 244, 500);
    assert(summary.suspect && summary.dac_write_failed && summary.dac_write_retries == 1);
    assert((summary.dac_retry_images == std::vector<std::size_t>{1}));
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
    return frame_timing_summary_json(summary);
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
        std::vector<FrameTiming> rows;
        int settle = -1;
        std::string error;
        assert(read_frame_timing_csv(replay.path, rows, settle, error) && error.empty());
        assert(settle == replay.settle_ms && rows.size() == 400 && rows[399].image_index == 399);
        // The derived columns as the program wrote them (the recheck recomputes them).
        assert(std::isfinite(rows[0].exposure_start_host_ms) && std::isfinite(rows[0].next_dac_slack_ms));
        assert(std::isnan(rows[399].next_dac_slack_ms));
        const auto summary = recheck_frame_timing(rows, true, settle, replay.exposure_us, 500, replay.first, replay.last);
        const FrameClockMapping& clock = summary.clock;
        assert(clock.valid);
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

// frame_timing_csv -> fixture -> read_frame_timing_csv gives the rows back (times to the 3 decimals written, derived columns
// to 6, empty ones NaN) and the settle; any other file is refused with its reason.
void check_csv_round_trip(const std::string& path, const std::vector<FrameTiming>& written)
{
    std::vector<FrameTiming> rows;
    int settle = -1;
    std::string error;
    assert(read_frame_timing_csv(path, rows, settle, error) && error.empty());
    assert(settle == 5 && rows.size() == written.size());
    for (std::size_t k = 0; k < rows.size(); ++k) {
        const FrameTiming& a = rows[k];
        const FrameTiming& b = written[k];
        assert(a.image_index == b.image_index && a.frame_id == b.frame_id);
        assert(a.camera_timestamp_ns == b.camera_timestamp_ns && a.system_timestamp_ns == b.system_timestamp_ns);
        const double times[][2] = {{a.dac_start_ms, b.dac_start_ms}, {a.dac_done_ms, b.dac_done_ms},
                                   {a.trigger_start_ms, b.trigger_start_ms}, {a.trigger_done_ms, b.trigger_done_ms},
                                   {a.buffer_ms, b.buffer_ms}, {a.written_ms, b.written_ms}};
        for (const auto& pair : times)
            assert(std::abs(pair[0] - pair[1]) <= 0.0005);
        for (const auto& pair : {std::make_pair(a.exposure_start_host_ms, b.exposure_start_host_ms),
                                 std::make_pair(a.next_dac_slack_ms, b.next_dac_slack_ms)})
            assert(std::isnan(pair.first) == std::isnan(pair.second) &&
                   (std::isnan(pair.first) || std::abs(pair.first - pair.second) <= 5e-7));
        assert(!a.dac_retried && !a.dac_failed);
    }
    // A camera that lost frame 1: buffer and written -1, no exposure columns.
    std::vector<FrameTiming> lost = written;
    lost[1].buffer_ms = lost[1].written_ms = -1;
    lost[1].exposure_start_host_ms = lost[1].next_dac_slack_ms = NAN;
    assert(write_fixture(path, frame_timing_csv(lost, 5)));
    assert(read_frame_timing_csv(path, rows, settle, error) && rows.size() == lost.size());
    assert(near(rows[1].buffer_ms, -1) && near(rows[1].written_ms, -1) && std::isnan(rows[1].exposure_start_host_ms));
    // CRLF line ends read the same.
    std::ifstream input(path.c_str());
    std::string text((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    std::string crlf;
    for (char c : text) crlf += c == '\n' ? std::string("\r\n") : std::string(1, c);
    const std::string header = text.substr(0, text.find('\n') + 1);
    const std::string row0 = text.substr(header.size(), text.find('\n', header.size()) + 1 - header.size());
    auto refused = [&](const std::string& content, const char* reason) {
        std::ofstream(path.c_str(), std::ios::binary) << content;
        std::vector<FrameTiming> out(1);
        int out_settle = -1;
        std::string out_error;
        const bool read = read_frame_timing_csv(path, out, out_settle, out_error);
        assert(!read && out.empty() && out_settle == 0 && contains(out_error, reason));
        (void)read;
    };
    std::ofstream(path.c_str(), std::ios::binary) << crlf;
    assert(read_frame_timing_csv(path, rows, settle, error) && rows.size() == lost.size() && settle == 5);
    // Header only: no rows, no error.
    std::ofstream(path.c_str(), std::ios::binary) << header;
    assert(read_frame_timing_csv(path, rows, settle, error) && rows.empty());
    auto replace = [](std::string value, const std::string& from, const std::string& to) {
        value.replace(value.find(from), from.size(), to);
        return value;
    };
    refused("", "empty file");
    refused("image_index,settle_ms,dac_start_ms\n0,5,0.5\n", "not a schema 2");   // schema 1
    refused(header + replace(row0, "2,0,5,", "1,0,5,"), "malformed row 2");          // another schema in a row
    refused(header + replace(row0, "2,0,5,", "2,1,5,"), "malformed row 2");          // not image 0
    refused(header + row0 + row0, "malformed row 3");                                // image 0 twice
    // A settle that changes from row to row (an FSM schedule): every row carries its own, settle is -1.
    std::ofstream(path.c_str(), std::ios::binary) << header + replace(row0, "2,0,5,", "2,0,7,") +
                                                     text.substr(header.size() + row0.size());
    assert(read_frame_timing_csv(path, rows, settle, error) && settle == -1 && rows.size() == lost.size());
    assert(rows[0].settle_ms == 7 && rows[1].settle_ms == 5 && rows[2].settle_ms == 5);
    // One settle in every row: the rows keep -1, as before.
    assert(write_fixture(path, frame_timing_csv(lost, 5)));
    assert(read_frame_timing_csv(path, rows, settle, error) && settle == 5);
    for (const FrameTiming& row : rows)
        assert(row.settle_ms == -1);
    refused(header + replace(row0, "2,0,5,0.000", "2,0,5,abc"), "malformed row 2");  // a number that is not one
    refused(header + row0.substr(0, 20) + "\n", "malformed row 2");                  // a short row
    refused(header + replace(row0, ",100,", ",-100,"), "malformed row 2");            // a negative frame id
    std::remove(path.c_str());
    assert(!read_frame_timing_csv(path, rows, settle, error) && rows.empty() && contains(error, "No such file"));
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
           "[frame-timing] mode=overlap frames=3 id_gaps=0 dac_retries=0 spacing_dev_max_ms=0.000000 "
           "min_slack_ms=0.756000 trigger_start_delay_median_ms=0.200000 trigger_start_delay_min_ms=0.200000 "
           "trigger_start_delay_max_ms=0.200000 trigger_done_delay_median_ms=0.000000 "
           "trigger_done_delay_min_ms=0.000000 trigger_done_delay_max_ms=0.000000 "
           "latch_uncertainty_ms=0.200000 latch=ok verdict=ok");
    const std::string path = "/tmp/tdm_frame_loop_summary.json";
    std::string error;
    assert(write_fixture(path, frame_timing_summary_json(summary)));
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
    assert(json.find("{\n  \"rules_version\": 2,\n  \"mode\": \"overlap\",\n") == 0 && frame_timing_rules_version == 2);
    std::remove(path.c_str());
    summary = check_frame_timing(rows, false, 5, 244, 500);
    assert(write_fixture(path, frame_timing_summary_json(summary)));
    std::ifstream absent_input(path.c_str());
    const std::string absent_json((std::istreambuf_iterator<char>(absent_input)), std::istreambuf_iterator<char>());
    assert(absent_json.find("\"min_slack_ms\": null") != std::string::npos);
    assert(absent_json.find("\"latch\": \"absent\"") != std::string::npos);
    assert(absent_json.find("\"latch_before\": null") != std::string::npos);
    assert(absent_json.find("\"trigger_start_delay_median_ms\": null") != std::string::npos);
    assert(absent_json.find("\"trigger_done_delay_median_ms\": null") != std::string::npos);
    assert(absent_json.find("\"latch_uncertainty_ms\": null") != std::string::npos);
    std::remove(path.c_str());
    assert(!write_fixture("/nonexistent-dir/summary.json", frame_timing_summary_json(summary)));
    check_frame_timing(rows, true, 5, 244, 500, clock);
    const std::string csv_path = "/tmp/tdm_frame_loop.csv";
    assert(write_fixture(csv_path, frame_timing_csv(rows, 5)));
    std::ifstream csv(csv_path.c_str());
    std::string line;
    std::getline(csv, line);
    std::getline(csv, line);
    assert(line.substr(line.size() - 18) == ",6.000000,0.756000");
    std::getline(csv, line);
    std::getline(csv, line);
    assert(line.substr(line.size() - 11) == ",46.000000,");
    check_csv_round_trip(csv_path, rows);
    std::remove(csv_path.c_str());
    rows.resize(1);
    rows[0].camera_timestamp_ns += 2000000ULL;
    summary = check_frame_timing(rows, false, 5, 244, 500, clock);
    assert(write_fixture(path, frame_timing_summary_json(summary)));
    std::ifstream suspect_input(path.c_str());
    const std::string suspect_json((std::istreambuf_iterator<char>(suspect_input)), std::istreambuf_iterator<char>());
    assert(suspect_json.find("\"timestamp_meaning_suspect\": true") != std::string::npos);
    assert(suspect_json.find("timestamp meaning is not the exposure start; slack checks are not trustworthy") != std::string::npos);
    std::remove(path.c_str());
}

// B199: an FSM schedule gives each frame its own settle: the frame sleeps it after its DAC write and its row records it.
// The loop is sequential also when overlap was requested (in overlap the trigger also waits for the previous frame's
// transfer, so a short settle would not be the real wait): the steps are the constant-settle sequential loop's, and the
// summary says sequential, overlap requested, with the schedule as the fallback reason.
void check_schedule_settle(bool overlap)
{
    const std::size_t count = 5;
    const int settles[] = {3, 20, 0, 7, 1000};
    FrameLoopOptions config = options(overlap);
    config.frame_settle_ms.assign(settles, settles + count);
    FakeIO io(count);
    std::vector<FrameTiming> rows;
    assert(run_frame_loop(count, config, io, rows));
    assert(io.finishes == 1 && io.finished_ok && io.held.empty() && rows.size() == count);
    assert((io.written == std::vector<std::size_t>{0, 1, 2, 3, 4}));
    FakeIO constant(count);
    std::vector<FrameTiming> constant_rows;
    assert(run_frame_loop(count, options(false), constant, constant_rows));
    assert(io.events == constant.events);   // the sequential loop's order, whatever overlap said
    for (std::size_t k = 0; k < count; ++k) {
        assert(rows[k].settle_ms == settles[k] && constant_rows[k].settle_ms == -1);
        assert(near(rows[k].trigger_start_ms - rows[k].dac_done_ms, settles[k]));   // exactly the frame's sleep
        if (k > 0)
            assert(rows[k].dac_start_ms >= rows[k - 1].written_ms);   // no move before the previous frame is saved
    }
    assert(!io.saved_options.overlap && io.saved_options.requested_overlap == overlap);
    assert(io.saved_options.fallback_reason == (overlap ? fsm_schedule_sequential_reason : ""));
    assert(io.saved_options.frame_settle_ms == config.frame_settle_ms);
    FrameTimingSummary summary = check_frame_timing(rows, io.saved_options, fake_clock());
    assert(!summary.suspect && summary.unsettled_frames == 0 && summary.frames == count && summary.overlap_violations == 0);
    assert(summary.settle_per_frame && summary.settle_ms_min == 0 && summary.settle_ms_max == 1000);
    std::string json = summary_json(summary);
    assert(contains(json, "\"settle_ms\": null,\n  \"settle_ms_min\": 0,\n  \"settle_ms_max\": 1000,\n  \"exposure_us\""));
    assert(contains(json, "\"mode\": \"sequential\""));
    assert(contains(json, std::string("\"requested_mode\": \"") + (overlap ? "overlap" : "sequential") + "\""));
    assert(contains(json, overlap ? "\"fallback_reason\": \"FSM schedule: sequential loop" : "\"fallback_reason\": null"));
    // The constant run's summary is the old one: settle_ms, no range keys.
    const FrameTimingSummary constant_summary = check_frame_timing(constant_rows, constant.saved_options, fake_clock());
    assert(!constant_summary.settle_per_frame && !constant_summary.suspect);
    const std::string constant_json = summary_json(constant_summary);
    assert(contains(constant_json, "\"settle_ms\": 5,\n  \"exposure_us\"") && !contains(constant_json, "settle_ms_min"));
    // The CSV records each frame's settle; it reads back per frame and re-checks to the same verdict.
    const std::string path = "/tmp/tdm_frame_loop_schedule_" + std::to_string(getpid()) + ".csv";
    std::string error;
    assert(write_fixture(path, frame_timing_csv(rows, config.settle_ms)));
    std::ifstream input(path.c_str());
    std::string line;
    std::getline(input, line);
    for (std::size_t k = 0; k < count; ++k) {
        std::getline(input, line);
        assert(line.compare(0, 4 + std::to_string(k).size() + std::to_string(settles[k]).size(),
                            "2," + std::to_string(k) + "," + std::to_string(settles[k]) + ",") == 0);
    }
    std::vector<FrameTiming> read;
    int settle = 0;
    assert(read_frame_timing_csv(path, read, settle, error) && settle == -1 && read.size() == count);
    for (std::size_t k = 0; k < count; ++k)
        assert(read[k].settle_ms == settles[k]);
    const FrameClockMapping clock = fake_clock();
    summary = recheck_frame_timing(read, false, settle, config.exposure_us, config.margin_us, clock.first, clock.last);
    assert(!summary.suspect && summary.unsettled_frames == 0 && summary.settle_ms_min == 0 && summary.settle_ms_max == 1000);
    std::remove(path.c_str());
}

// A settle per frame that does not fit the frames is refused before any DAC write.
void check_schedule_refused()
{
    for (bool overlap : {false, true}) {
        for (int kind = 0; kind < 2; ++kind) {
            FrameLoopOptions config = options(overlap);
            config.frame_settle_ms = kind == 0 ? std::vector<int>{5, 5, 5} : std::vector<int>{5, -1, 5, 5};
            FakeIO io(4);
            std::vector<FrameTiming> rows;
            char log_template[] = "/tmp/tdm_frame_loop_stderr_XXXXXX";
            const int log_fd = mkstemp(log_template);
            const std::string log_path = log_template;
            std::fflush(stderr);
            const int saved_stderr = dup(2);
            assert(log_fd >= 0 && saved_stderr >= 0 && dup2(log_fd, 2) == 2);
            const bool completed = run_frame_loop(4, config, io, rows);
            std::fflush(stderr);
            assert(dup2(saved_stderr, 2) == 2 && close(saved_stderr) == 0 && close(log_fd) == 0);
            std::ifstream log_input(log_path.c_str());
            const std::string logged((std::istreambuf_iterator<char>(log_input)), std::istreambuf_iterator<char>());
            std::remove(log_path.c_str());
            assert(logged == (kind == 0 ? "FSM schedule: 3 settle values for 4 frames: nothing moved\n"
                                        : "FSM schedule: a negative settle: nothing moved\n"));
            assert(!completed && io.finishes == 1 && !io.finished_ok && rows.empty() && io.written.empty());
            assert((io.events == std::vector<std::string>{"save_timing", "stop"}));
        }
    }
}

// The unsettled check uses each row's own settle: a frame exposed 3 ms after its DAC write is unsettled with a 5 ms
// settle of its own and settled with a 2 ms one, whatever the run's settle; rows without their own keep the run's.
void check_schedule_unsettled()
{
    // check_timing's clock: exposure k starts at 6 + 20 k ms (0.2 ms after its trigger start), uncertainty 0.2 ms.
    FrameClockLatch first, last;
    first.valid = last.valid = true;
    first.camera_ns = 1000000000ULL;
    first.host_before_ms = -0.1;
    first.host_after_ms = 0.1;
    last.camera_ns = 1100000000ULL;
    last.host_before_ms = 99.8;
    last.host_after_ms = 100.2;
    const FrameClockMapping clock = frame_clock_mapping(first, last);
    auto rows = synthetic_rows();
    for (FrameTiming& row : rows)
        row.dac_done_ms = row.trigger_start_ms + 0.2 - 3;   // each exposure starts 3 ms after its DAC write
    assert(check_frame_timing(rows, false, 2, 244, 500, clock).unsettled_frames == 0);
    assert(check_frame_timing(rows, false, 5, 244, 500, clock).unsettled_frames == 3);
    rows[1].settle_ms = 2;
    auto summary = check_frame_timing(rows, false, 5, 244, 500, clock);
    assert(summary.unsettled_frames == 2 && (summary.unsettled_images == std::vector<std::size_t>{0, 2}));
    assert(summary.settle_per_frame && summary.settle_ms_min == 2 && summary.settle_ms_max == 2);
    rows[0].settle_ms = 5;
    rows[2].settle_ms = 0;
    summary = check_frame_timing(rows, false, 2, 244, 500, clock);
    assert(summary.unsettled_frames == 1 && summary.unsettled_images == std::vector<std::size_t>{0});
    assert(summary.suspect && summary.settle_ms_min == 0 && summary.settle_ms_max == 5);
    // The same settle in every row's own field: the verdict of the constant run.
    for (int settle : {2, 5}) {
        auto constant = synthetic_rows(), own = synthetic_rows();
        for (std::size_t k = 0; k < constant.size(); ++k) {
            constant[k].dac_done_ms = own[k].dac_done_ms = constant[k].trigger_start_ms + 0.2 - 3;
            own[k].settle_ms = settle;
        }
        const auto a = check_frame_timing(constant, true, settle, 244, 500, clock);
        const auto b = check_frame_timing(own, true, 99, 244, 500, clock);
        assert(a.suspect == b.suspect && a.unsettled_images == b.unsettled_images);
    }
}

// A stop arriving during frame acquisition saves that frame, returns every buffer and finishes
// partial timing. In overlap a speculative next DAC row is allowed, but never another trigger.
void check_stop(bool overlap)
{
    FakeIO io(5);
    FrameLoopOptions config = options(overlap);
    bool delivered = false;
    config.stop_requested = [&]() {
        if (!delivered && std::find(io.events.begin(), io.events.end(), "trigger1") != io.events.end()) {
            delivered = true;
            return true; // Even a one-shot stop request must be latched until finish.
        }
        return false;
    };
    std::vector<FrameTiming> rows;
    assert(!run_frame_loop(5, config, io, rows));
    assert(io.finishes == 1 && !io.finished_ok && io.held.empty());
    assert((io.written == std::vector<std::size_t>{0, 1}));
    assert(io.pushes[0] == 1 && io.pushes[1] == 1 && io.pushes[2] == 0);
    assert(std::find(io.events.begin(), io.events.end(), "trigger2") == io.events.end());
    assert(io.saved.size() == rows.size() && rows[1].written_ms >= 0);
    assert(position(io, "push1") < position(io, "save_timing"));
    assert(position(io, "save_timing") < position(io, "stop"));
    io.events.clear(); io.written.clear(); io.pushes.assign(5, 0); io.time = io.ready = 0;
    assert(run_frame_loop(5, config, io, rows)); // The previous loop's stop latch must be fresh.
    assert(io.finishes == 2 && io.held.empty() && io.written.size() == 5 && rows.size() == 5);
    FakeIO idle(5);
    config.stop_requested = []() { return true; };
    assert(!run_frame_loop(5, config, idle, rows));
    assert(rows.empty() && idle.held.empty() && idle.finishes == 1);
    assert((idle.events == std::vector<std::string>{"save_timing", "stop"}));
}

// The generic loop has no retained rows, held buffer or command index when reused on one IO.
// Production uses a new adapter (and hence clock/latches) for each run on the same connection.
void check_repeated_loops(bool overlap)
{
    FakeIO io(3);
    std::vector<FrameTiming> rows;
    const auto config = options(overlap);
    assert(run_frame_loop(3, config, io, rows));
    const auto first = rows;
    assert(io.held.empty());
    io.events.clear(); io.written.clear(); io.pushes.assign(3, 0);
    io.time = 0; io.ready = 0;
    assert(run_frame_loop(3, config, io, rows));
    assert(io.finishes == 2 && io.held.empty() && rows.size() == first.size());
    for (std::size_t k = 0; k < rows.size(); ++k) {
        assert(rows[k].image_index == k && rows[k].dac_start_ms == first[k].dac_start_ms);
        assert(rows[k].buffer_ms == first[k].buffer_ms && rows[k].written_ms == first[k].written_ms);
        assert(rows[k].camera_timestamp_ns == first[k].camera_timestamp_ns && io.pushes[k] == 1);
    }
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
        for (std::size_t at : {0, 2, 4}) {
            check_dac_retry(overlap, at);
            check_dac_failure(overlap, at);
        }
    }
    check_dac_summary();
    check_timing();
    for (bool overlap : {false, true})
        check_schedule_settle(overlap);
    check_schedule_refused();
    check_schedule_unsettled();
    for (bool overlap : {false, true}) { check_stop(overlap); check_repeated_loops(overlap); }
    return 0;
}
