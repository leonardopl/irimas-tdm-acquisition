#pragma once

// Host times use steady_clock milliseconds from the first DAC command. Camera times are nanoseconds.
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <string>
#include <vector>

// The version of check_frame_timing's verdict rules, written into frame_timing_summary.json as "rules_version" (a summary
// without the key counts as 1). 1: before a6e7f77 (the overlap latch rule and the trigger-done spacing made good scans
// suspect); 2: a6e7f77's latch and spacing rules, and B206's dac_* keys (they do not change the verdict of a scan
// without DAC failures). Bump it whenever check_frame_timing's verdict rules change: readers re-check older summaries from
// frame_timing.csv (recheck_frame_timing). This header is copied byte-identical into the qwlsi-gui repository.
constexpr int frame_timing_rules_version = 2;

struct FrameTiming {
    std::size_t image_index = 0;
    double dac_start_ms = 0;
    double dac_done_ms = 0;
    double trigger_start_ms = 0;
    double trigger_done_ms = 0;
    double buffer_ms = -1;          // -1: no buffer arrived
    double written_ms = -1;         // -1: not written
    std::uint64_t frame_id = 0;
    std::uint64_t camera_timestamp_ns = 0;
    std::uint64_t system_timestamp_ns = 0;
    double exposure_start_host_ms = std::numeric_limits<double>::quiet_NaN();
    double next_dac_slack_ms = std::numeric_limits<double>::quiet_NaN();
    // Not in the CSV: the first DAC write failed and was repeated; the repeat failed too (the scan stopped here).
    bool dac_retried = false;
    bool dac_failed = false;
    // The frame's own settle (an FSM schedule, B199: a settle per frame); -1: the run's settle_ms. The CSV's settle
    // column holds the one the frame waited.
    int settle_ms = -1;
};

struct FrameClockLatch {
    bool valid = false;
    std::uint64_t camera_ns = 0;
    double host_before_ms = 0;
    double host_after_ms = 0;
};

inline double camera_delta_ms(std::uint64_t a, std::uint64_t b)
{
    // Subtract before converting, retaining precision with large camera timestamps.
    return a >= b ? (a - b) / 1000000.0 : -((b - a) / 1000000.0);
}

struct FrameClockMapping {
    bool valid = false;
    bool failed = false;
    FrameClockLatch first, last;
    double scale = 1;
    double offset_ms = 0;
    double drift_ppm = 0;
    double uncertainty_ms = 0;

    double host_ms(std::uint64_t camera_ns) const {
        return (first.host_before_ms + first.host_after_ms) / 2 +
               scale * camera_delta_ms(camera_ns, first.camera_ns);
    }
};

inline FrameClockMapping frame_clock_mapping(const FrameClockLatch& first, const FrameClockLatch& last,
                                             bool latch_supported = false)
{
    FrameClockMapping mapping;
    mapping.first = first;
    mapping.last = last;
    mapping.failed = latch_supported || first.valid || last.valid;
    if (!first.valid || !last.valid || last.camera_ns <= first.camera_ns ||
        !std::isfinite(first.host_before_ms) || !std::isfinite(first.host_after_ms) ||
        !std::isfinite(last.host_before_ms) || !std::isfinite(last.host_after_ms) ||
        first.host_after_ms < first.host_before_ms || last.host_after_ms < last.host_before_ms ||
        last.host_before_ms <= first.host_after_ms)
        return mapping;
    const double first_mid = (first.host_before_ms + first.host_after_ms) / 2;
    const double last_mid = (last.host_before_ms + last.host_after_ms) / 2;
    mapping.scale = (last_mid - first_mid) / camera_delta_ms(last.camera_ns, first.camera_ns);
    mapping.offset_ms = first_mid - mapping.scale * (first.camera_ns / 1000000.0);
    mapping.drift_ppm = (mapping.scale - 1) * 1000000;
    // For interpolated frames, the larger half-window bounds the midpoint mapping's uncertainty.
    mapping.uncertainty_ms = std::max(first.host_after_ms - first.host_before_ms,
                                    last.host_after_ms - last.host_before_ms) / 2;
    mapping.valid = true;
    mapping.failed = false;
    return mapping;
}

inline const char* frame_clock_latch_status(const FrameClockMapping& clock)
{
    return clock.valid ? "ok" : (clock.failed ? "failed" : "absent");
}

struct FrameDelayStatistics {
    double median_ms = std::numeric_limits<double>::quiet_NaN();
    double min_ms = std::numeric_limits<double>::quiet_NaN();
    double max_ms = std::numeric_limits<double>::quiet_NaN();
};

inline FrameDelayStatistics frame_delay_statistics(std::vector<double> values)
{
    FrameDelayStatistics result;
    if (values.empty())
        return result;
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2;
    result.median_ms = values.size() % 2 ? values[middle] : (values[middle - 1] + values[middle]) / 2;
    result.min_ms = values.front();
    result.max_ms = values.back();
    return result;
}

struct FrameTimingSummary {
    bool overlap = false;
    bool requested_overlap = false;
    std::string fallback_reason;
    int settle_ms = 0;
    // Rows with their own settle (an FSM schedule): the range of the frames' settles. The JSON then has "settle_ms": null
    // and "settle_ms_min" / "settle_ms_max"; without them it has neither key, as before.
    bool settle_per_frame = false;
    int settle_ms_min = 0, settle_ms_max = 0;
    double exposure_us = std::numeric_limits<double>::quiet_NaN();
    int margin_us = 500;
    std::size_t frames = 0;
    std::size_t id_gaps = 0;
    std::vector<std::size_t> id_gap_after_images;
    double spacing_dev_median_ms = 0;
    double spacing_dev_max_ms = 0;
    double min_slack_ms = std::numeric_limits<double>::quiet_NaN();
    std::size_t overlap_violations = 0;
    std::size_t negative_slack_frames = 0;
    std::size_t unsettled_frames = 0;
    std::vector<std::size_t> negative_slack_images, unsettled_images;
    std::size_t timestamp_errors = 0;
    std::size_t dac_write_retries = 0;
    std::vector<std::size_t> dac_retry_images;
    bool dac_write_failed = false;
    FrameDelayStatistics trigger_start_delay, trigger_done_delay;
    bool timestamp_meaning_suspect = false;
    bool latch_uncertain = false;
    bool suspect = false;
    FrameClockMapping clock;
};

// Pure apart from filling the two derived row columns. Spacing excess above 0.5 ms warns;
// good runs on 29 Oct had zero excess. Slack and settle checks allow the latch uncertainty. A row with its own settle_ms
// (>= 0) is checked against it, any other against settle_ms: a run with one settle gets the same verdict either way.
inline FrameTimingSummary check_frame_timing(std::vector<FrameTiming>& rows, bool overlap,
                                             int settle_ms, double exposure_us, int margin_us,
                                             const FrameClockMapping& clock = FrameClockMapping(),
                                             bool scan_complete = true)
{
    FrameTimingSummary summary;
    summary.overlap = overlap;
    summary.requested_overlap = overlap;
    summary.settle_ms = settle_ms;
    summary.exposure_us = exposure_us;
    summary.margin_us = margin_us;
    summary.clock = clock;
    // The absolute cap also guards settling, which next-DAC slack cannot validate.
    summary.latch_uncertain = clock.valid && clock.uncertainty_ms > 0.5;
    summary.suspect = !scan_complete || clock.failed;
    std::vector<double> deviations;
    std::vector<double> trigger_start_delays, trigger_done_delays, trigger_durations;
    const FrameTiming* previous = NULL;
    for (std::size_t k = 0; k < rows.size(); ++k) {
        FrameTiming& row = rows[k];
        row.exposure_start_host_ms = row.next_dac_slack_ms = std::numeric_limits<double>::quiet_NaN();
        // A retry that landed leaves the verdict: dac_done_ms follows it, so the settle checks still apply. One that
        // failed too is dac_write_failed only (rig 29 Oct: "DAC write failed, DAC retried 1" read as a landed retry).
        if (row.dac_retried && !row.dac_failed) {
            ++summary.dac_write_retries;
            summary.dac_retry_images.push_back(row.image_index);
        }
        summary.dac_write_failed = summary.dac_write_failed || row.dac_failed;
        const int frame_settle_ms = row.settle_ms >= 0 ? row.settle_ms : settle_ms;
        if (row.settle_ms >= 0) {
            summary.settle_ms_min = summary.settle_per_frame ? std::min(summary.settle_ms_min, row.settle_ms) : row.settle_ms;
            summary.settle_ms_max = summary.settle_per_frame ? std::max(summary.settle_ms_max, row.settle_ms) : row.settle_ms;
            summary.settle_per_frame = true;
        }
        if (row.buffer_ms < 0) {
            summary.suspect = true;
            continue;
        }
        ++summary.frames;
        if (row.written_ms < 0)
            summary.suspect = true;
        if (previous) {
            if (row.frame_id <= previous->frame_id || row.frame_id - previous->frame_id != 1) {
                ++summary.id_gaps;
                summary.id_gap_after_images.push_back(previous->image_index);
            }
            if (row.camera_timestamp_ns <= previous->camera_timestamp_ns)
                ++summary.timestamp_errors;
            const double spacing = camera_delta_ms(row.camera_timestamp_ns, previous->camera_timestamp_ns);
            const double minimum = row.trigger_start_ms - previous->trigger_done_ms;
            const double maximum = row.trigger_done_ms - previous->trigger_start_ms;
            // Signed excess outside the trigger-call interval: negative below, positive above, zero inside.
            const double deviation = spacing < minimum ? spacing - minimum :
                                     (spacing > maximum ? spacing - maximum : 0);
            deviations.push_back(deviation);
            summary.spacing_dev_max_ms = std::max(summary.spacing_dev_max_ms, std::abs(deviation));
        }
        previous = &row;
        if (overlap && k + 1 < rows.size() &&
            rows[k + 1].dac_start_ms - row.trigger_done_ms + 1e-9 < (exposure_us + margin_us) / 1000.0)
            ++summary.overlap_violations;
        if (clock.valid) {
            row.exposure_start_host_ms = clock.host_ms(row.camera_timestamp_ns);
            trigger_start_delays.push_back(row.exposure_start_host_ms - row.trigger_start_ms);
            trigger_done_delays.push_back(row.exposure_start_host_ms - row.trigger_done_ms);
            trigger_durations.push_back(row.trigger_done_ms - row.trigger_start_ms);
            if (row.exposure_start_host_ms < row.dac_done_ms + frame_settle_ms - clock.uncertainty_ms) {
                ++summary.unsettled_frames;
                summary.unsettled_images.push_back(row.image_index);
            }
            if (k + 1 < rows.size() && std::isfinite(exposure_us) && exposure_us > 0) {
                row.next_dac_slack_ms = rows[k + 1].dac_start_ms -
                                        (row.exposure_start_host_ms + exposure_us / 1000.0);
                if (std::isfinite(row.next_dac_slack_ms) && row.next_dac_slack_ms >= -clock.uncertainty_ms &&
                    row.next_dac_slack_ms < clock.uncertainty_ms)
                    summary.latch_uncertain = true;
                if (!std::isfinite(summary.min_slack_ms) || row.next_dac_slack_ms < summary.min_slack_ms)
                    summary.min_slack_ms = row.next_dac_slack_ms;
                if (row.next_dac_slack_ms < -clock.uncertainty_ms) {
                    ++summary.negative_slack_frames;
                    summary.negative_slack_images.push_back(row.image_index);
                }
            }
        }
    }
    if (!deviations.empty()) {
        std::sort(deviations.begin(), deviations.end());
        const std::size_t middle = deviations.size() / 2;
        summary.spacing_dev_median_ms = deviations.size() % 2 ? deviations[middle] :
                                        (deviations[middle - 1] + deviations[middle]) / 2;
    }
    summary.trigger_start_delay = frame_delay_statistics(trigger_start_delays);
    summary.trigger_done_delay = frame_delay_statistics(trigger_done_delays);
    if (!trigger_start_delays.empty()) {
        const double trigger_duration_median_ms = frame_delay_statistics(trigger_durations).median_ms;
        summary.timestamp_meaning_suspect = summary.trigger_start_delay.median_ms < -clock.uncertainty_ms ||
            summary.trigger_start_delay.median_ms > trigger_duration_median_ms + clock.uncertainty_ms + 1;
    }
    summary.suspect = summary.suspect || summary.id_gaps || summary.timestamp_errors ||
                      summary.spacing_dev_max_ms > 0.5 || summary.overlap_violations ||
                      summary.negative_slack_frames || summary.unsettled_frames || summary.timestamp_meaning_suspect ||
                      summary.latch_uncertain || summary.dac_write_failed;
    return summary;
}

inline std::string frame_timing_number(double value, const char* missing = "null")
{
    if (!std::isfinite(value))
        return missing;
    std::ostringstream text;
    text.imbue(std::locale::classic());
    text << std::fixed << std::setprecision(6) << value;
    return text.str();
}

inline std::string frame_timing_summary_line(const FrameTimingSummary& summary)
{
    std::ostringstream text;
    text << "[frame-timing] mode=" << (summary.overlap ? "overlap" : "sequential")
         << " frames=" << summary.frames << " id_gaps=" << summary.id_gaps
         << " dac_retries=" << summary.dac_write_retries
         << " spacing_dev_max_ms=" << frame_timing_number(summary.spacing_dev_max_ms)
         << " min_slack_ms=" << frame_timing_number(summary.min_slack_ms, "na")
         << " trigger_start_delay_median_ms=" << frame_timing_number(summary.trigger_start_delay.median_ms, "na")
         << " trigger_start_delay_min_ms=" << frame_timing_number(summary.trigger_start_delay.min_ms, "na")
         << " trigger_start_delay_max_ms=" << frame_timing_number(summary.trigger_start_delay.max_ms, "na")
         << " trigger_done_delay_median_ms=" << frame_timing_number(summary.trigger_done_delay.median_ms, "na")
         << " trigger_done_delay_min_ms=" << frame_timing_number(summary.trigger_done_delay.min_ms, "na")
         << " trigger_done_delay_max_ms=" << frame_timing_number(summary.trigger_done_delay.max_ms, "na")
         << " latch_uncertainty_ms=" << frame_timing_number(summary.clock.valid ? summary.clock.uncertainty_ms : NAN, "na")
         << " latch=" << frame_clock_latch_status(summary.clock)
         << " verdict=" << (summary.suspect ? "suspect" : "ok");
    if (summary.timestamp_meaning_suspect)
        text << " note=timestamp_is_not_exposure_start;slack_checks_untrustworthy";
    if (summary.latch_uncertain)
        text << " note=latch_uncertain;latch_checks_inconclusive";
    return text.str();
}

inline std::string frame_clock_latch_json(const FrameClockLatch& latch)
{
    if (!latch.valid)
        return "null";
    std::ostringstream text;
    text << "{\"camera_timestamp_ns\": " << latch.camera_ns
         << ", \"host_before_ms\": " << frame_timing_number(latch.host_before_ms)
         << ", \"host_after_ms\": " << frame_timing_number(latch.host_after_ms) << "}";
    return text.str();
}

inline std::string frame_timing_indices_json(const std::vector<std::size_t>& indices)
{
    std::ostringstream text;
    text << "[";
    for (std::size_t i = 0; i < indices.size(); ++i)
        text << (i ? ", " : "") << indices[i];
    text << "]";
    return text.str();
}

inline std::string frame_timing_json_string(const std::string& value)
{
    std::ostringstream text;
    text << '"';
    for (unsigned char character : value) {
        if (character == '"' || character == '\\')
            text << '\\' << character;
        else if (character < 0x20)
            text << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(character);
        else
            text << character;
    }
    text << '"';
    return text.str();
}

inline bool write_frame_timing_summary(const std::string& path, const FrameTimingSummary& summary,
                                      std::string& error)
{
    std::FILE* file = std::fopen(path.c_str(), "w");
    if (!file) {
        error = path + ": " + std::strerror(errno);
        return false;
    }
    std::ostringstream text;
    text << "{\n  \"rules_version\": " << frame_timing_rules_version
         << ",\n  \"mode\": \"" << (summary.overlap ? "overlap" : "sequential") << "\",\n"
         << "  \"requested_mode\": \"" << (summary.requested_overlap ? "overlap" : "sequential") << "\",\n"
         << "  \"fallback_reason\": " << (summary.fallback_reason.empty() ? "null" : frame_timing_json_string(summary.fallback_reason));
    if (summary.settle_per_frame)
        text << ",\n  \"settle_ms\": null,\n  \"settle_ms_min\": " << summary.settle_ms_min
             << ",\n  \"settle_ms_max\": " << summary.settle_ms_max;
    else
        text << ",\n  \"settle_ms\": " << summary.settle_ms;
    text << ",\n  \"exposure_us\": " << frame_timing_number(summary.exposure_us)
         << ",\n  \"margin_us\": " << summary.margin_us << ",\n"
         << "  \"frames\": " << summary.frames << ",\n  \"id_gaps\": " << summary.id_gaps
         << ",\n  \"spacing_dev_median_ms\": " << frame_timing_number(summary.spacing_dev_median_ms)
         << ",\n  \"spacing_dev_max_ms\": " << frame_timing_number(summary.spacing_dev_max_ms)
         << ",\n  \"min_slack_ms\": " << frame_timing_number(summary.min_slack_ms)
         << ",\n  \"trigger_start_delay_median_ms\": " << frame_timing_number(summary.trigger_start_delay.median_ms)
         << ",\n  \"trigger_start_delay_min_ms\": " << frame_timing_number(summary.trigger_start_delay.min_ms)
         << ",\n  \"trigger_start_delay_max_ms\": " << frame_timing_number(summary.trigger_start_delay.max_ms)
         << ",\n  \"trigger_done_delay_median_ms\": " << frame_timing_number(summary.trigger_done_delay.median_ms)
         << ",\n  \"trigger_done_delay_min_ms\": " << frame_timing_number(summary.trigger_done_delay.min_ms)
         << ",\n  \"trigger_done_delay_max_ms\": " << frame_timing_number(summary.trigger_done_delay.max_ms)
         << ",\n  \"timestamp_meaning_suspect\": " << (summary.timestamp_meaning_suspect ? "true" : "false")
         << ",\n  \"timestamp_note\": " << (summary.timestamp_meaning_suspect ?
             "\"Camera timestamp meaning is not the exposure start; slack checks are not trustworthy\"" : "null")
         << ",\n  \"latch_uncertain\": " << (summary.latch_uncertain ? "true" : "false")
         << ",\n  \"latch_note\": " << (summary.latch_uncertain ? "\"latch_uncertain: latch checks are inconclusive\"" : "null")
         << ",\n  \"latch\": \"" << frame_clock_latch_status(summary.clock)
         << "\",\n  \"verdict\": \"" << (summary.suspect ? "suspect" : "ok") << "\",\n"
         << "  \"overlap_violations\": " << summary.overlap_violations
         << ",\n  \"negative_slack_frames\": " << summary.negative_slack_frames
         << ",\n  \"unsettled_frames\": " << summary.unsettled_frames
         << ",\n  \"timestamp_errors\": " << summary.timestamp_errors
         << ",\n  \"dac_write_retries\": " << summary.dac_write_retries
         << ",\n  \"dac_write_failed\": " << (summary.dac_write_failed ? "true" : "false")
         << ",\n  \"clock_offset_ms\": " << frame_timing_number(summary.clock.valid ? summary.clock.offset_ms : NAN)
         << ",\n  \"clock_drift_ppm\": " << frame_timing_number(summary.clock.valid ? summary.clock.drift_ppm : NAN)
         << ",\n  \"latch_uncertainty_ms\": " << frame_timing_number(summary.clock.valid ? summary.clock.uncertainty_ms : NAN)
         << ",\n  \"latch_before\": " << frame_clock_latch_json(summary.clock.first)
         << ",\n  \"latch_after\": " << frame_clock_latch_json(summary.clock.last)
         << ",\n  \"id_gap_after_images\": " << frame_timing_indices_json(summary.id_gap_after_images)
         << ",\n  \"negative_slack_images\": " << frame_timing_indices_json(summary.negative_slack_images)
         << ",\n  \"unsettled_images\": " << frame_timing_indices_json(summary.unsettled_images)
         << ",\n  \"dac_retry_images\": " << frame_timing_indices_json(summary.dac_retry_images) << "\n}\n";
    const bool ok = std::fputs(text.str().c_str(), file) >= 0;
    if (std::fclose(file) != 0 || !ok) {
        error = path + ": write failed";
        return false;
    }
    return true;
}

inline bool write_frame_timing_csv(const std::string& path, const std::vector<FrameTiming>& rows, int settle_ms,
                                   std::string& error)
{
    std::FILE* file = std::fopen(path.c_str(), "w");
    if (!file) {
        error = path + ": " + std::strerror(errno);
        return false;
    }
    std::fprintf(file, "schema_version,image_index,settle_ms,dac_start_ms,dac_done_ms,trigger_start_ms,"
                       "trigger_done_ms,buffer_ms,written_ms,frame_id,camera_timestamp_ns,system_timestamp_ns,"
                       "exposure_start_host_ms,next_dac_slack_ms\n");
    for (const FrameTiming& row : rows)
        std::fprintf(file, "2,%zu,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%llu,%llu,%llu,%s,%s\n", row.image_index,
                     row.settle_ms >= 0 ? row.settle_ms : settle_ms,
                     row.dac_start_ms, row.dac_done_ms, row.trigger_start_ms, row.trigger_done_ms, row.buffer_ms,
                     row.written_ms, (unsigned long long)row.frame_id, (unsigned long long)row.camera_timestamp_ns,
                     (unsigned long long)row.system_timestamp_ns,
                     frame_timing_number(row.exposure_start_host_ms, "").c_str(),
                     frame_timing_number(row.next_dac_slack_ms, "").c_str());
    const bool ok = !std::ferror(file);
    if (std::fclose(file) != 0 || !ok) {
        error = path + ": write failed";
        return false;
    }
    return true;
}

// One frame_timing.csv field as a number, in the C locale; the whole field must be the number.
inline bool frame_timing_field(std::istringstream& input, const std::string& text, double& value)
{
    char rest;
    input.clear();
    input.str(text);
    return !text.empty() && (input >> value) && !(input >> rest);
}

inline bool frame_timing_field(std::istringstream& input, const std::string& text, unsigned long long& value)
{
    char rest;
    input.clear();
    input.str(text);
    return !text.empty() && text[0] >= '0' && text[0] <= '9' && (input >> value) && !(input >> rest);
}

// frame_timing.csv (schema 2) as write_frame_timing_csv writes it: one row per commanded frame, image_index 0, 1, ...
// in order; the derived columns (exposure_start_host_ms, next_dac_slack_ms) as written, NaN when empty. settle_ms is the
// file's settle column when it is the same in every row (the rows' own settle_ms then stay -1); when it differs (an FSM
// schedule) settle_ms is -1 and every row carries its own. dac_retried / dac_failed are not in the file and stay false.
// False with the reason for a file that cannot be read, another header (schema 1 has no derived columns) or a malformed
// row.
inline bool read_frame_timing_csv(const std::string& path, std::vector<FrameTiming>& rows, int& settle_ms,
                                  std::string& error)
{
    static const char* const header = "schema_version,image_index,settle_ms,dac_start_ms,dac_done_ms,trigger_start_ms,"
                                      "trigger_done_ms,buffer_ms,written_ms,frame_id,camera_timestamp_ns,"
                                      "system_timestamp_ns,exposure_start_host_ms,next_dac_slack_ms";
    rows.clear();
    settle_ms = 0;
    std::ifstream file(path.c_str(), std::ios::binary);
    if (!file) {
        error = path + ": " + std::strerror(errno);
        return false;
    }
    std::istringstream number;
    number.imbue(std::locale::classic());
    std::string line;
    if (!std::getline(file, line)) {
        error = path + ": empty file";
        return false;
    }
    if (!line.empty() && line[line.size() - 1] == '\r')
        line.erase(line.size() - 1);
    if (line != header) {
        error = path + ": not a schema 2 frame timing header";
        return false;
    }
    for (std::size_t row = 2; std::getline(file, line); ++row) {
        if (!line.empty() && line[line.size() - 1] == '\r')
            line.erase(line.size() - 1);
        std::vector<std::string> fields;
        for (std::size_t from = 0;;) {
            const std::size_t comma = line.find(',', from);
            fields.push_back(line.substr(from, comma == std::string::npos ? std::string::npos : comma - from));
            if (comma == std::string::npos)
                break;
            from = comma + 1;
        }
        FrameTiming timing;
        double values[6];
        unsigned long long index = 0, settle = 0, ids[3];
        bool ok = fields.size() == 14 && fields[0] == "2" && frame_timing_field(number, fields[1], index) &&
                  index == rows.size() && frame_timing_field(number, fields[2], settle) && settle <= 1000000;
        for (int k = 0; ok && k < 6; ++k)
            ok = frame_timing_field(number, fields[3 + k], values[k]) && std::isfinite(values[k]);
        for (int k = 0; ok && k < 3; ++k)
            ok = frame_timing_field(number, fields[9 + k], ids[k]);
        double derived[2] = {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};
        for (int k = 0; ok && k < 2; ++k)
            ok = fields[12 + k].empty() || (frame_timing_field(number, fields[12 + k], derived[k]) &&
                                            std::isfinite(derived[k]));
        if (!ok) {
            std::ostringstream reason;
            reason << path << ": malformed row " << row;
            error = reason.str();
            rows.clear();
            settle_ms = 0;
            return false;
        }
        timing.settle_ms = static_cast<int>(settle);
        timing.image_index = static_cast<std::size_t>(index);
        timing.dac_start_ms = values[0];
        timing.dac_done_ms = values[1];
        timing.trigger_start_ms = values[2];
        timing.trigger_done_ms = values[3];
        timing.buffer_ms = values[4];
        timing.written_ms = values[5];
        timing.frame_id = ids[0];
        timing.camera_timestamp_ns = ids[1];
        timing.system_timestamp_ns = ids[2];
        timing.exposure_start_host_ms = derived[0];
        timing.next_dac_slack_ms = derived[1];
        rows.push_back(timing);
    }
    if (file.bad()) {
        error = path + ": read failed";
        rows.clear();
        return false;
    }
    bool constant = true;
    for (std::size_t k = 1; k < rows.size(); ++k)
        constant = constant && rows[k].settle_ms == rows[0].settle_ms;
    settle_ms = rows.empty() ? 0 : (constant ? rows[0].settle_ms : -1);
    if (constant)
        for (FrameTiming& row : rows)
            row.settle_ms = -1;
    return true;
}

// A recorded scan's verdict under the current rules: its frame_timing.csv rows (read_frame_timing_csv; the derived
// columns are recomputed), the mode, settle (the reader's: -1 when the rows carry their own), exposure and margin of its
// summary, and the camera/host latch windows it recorded (latch_before / latch_after). The clock is rebuilt as the program builds it for a camera with the latch nodes.
// The CSV cannot say whether a DAC write failed: the caller keeps the summary's dac_* keys.
inline FrameTimingSummary recheck_frame_timing(std::vector<FrameTiming>& rows, bool overlap, int settle_ms,
                                               double exposure_us, int margin_us, const FrameClockLatch& first,
                                               const FrameClockLatch& last)
{
    return check_frame_timing(rows, overlap, settle_ms, exposure_us, margin_us,
                              frame_clock_mapping(first, last, true));
}
