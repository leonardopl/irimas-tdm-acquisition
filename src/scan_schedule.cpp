#include "scan_schedule.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <locale>
#include <sstream>

namespace {

const double kNormalizedLimit = 10.0;
const double kCommandLimit = 10.0;
// Settle tiers measured on the rig (qwlsi-gui B185, 2026-10-28, same bead at 5/10/100/300 ms): 5 ms matched 300 ms
// on UNIFORM3D (95 % of the steps 0.40-0.46 V, at most 0.73 V); on FERMAT jumps up to 6.1 V, 5 ms left ~5 % of each
// jump of 1 V or more and 10 ms matched 300 ms; 0.5-1 V jumps were not resolved. Larger steps (the (0, 8 V)
// intensity-reference move) were not tested and keep 100 ms. Same tiers as qwlsi-gui include/fsm_settle.h.
const double kSmallStepLimitVolts = 0.75;          // largest step
const double kSmallTypicalStepLimitVolts = 0.5;    // 95th percentile of the steps
const double kTypicalStepFraction = 0.95;
const double kTestedStepLimitVolts = 6.1;
const int kSmallStepSettleMilliseconds = 5;
const int kSettleMilliseconds = 10;
const int kConservativeSettleMilliseconds = 100;

int uniform3d_generated_count(int frame_count)
{
    const double surface_per_point = 2.0 * M_PI / frame_count;
    const double distance = std::sqrt(surface_per_point);
    const int circle_count = std::max(1,
        static_cast<int>(std::lround((M_PI / 2.0) / distance)));
    const double delta_theta = (M_PI / 2.0) / circle_count;
    double total_length = 0.0;
    for (int circle = 0; circle < circle_count; ++circle)
        total_length += 2.0 * M_PI * std::sin(circle * delta_theta);
    if (total_length <= 0.0)
        return 0;
    const double recalculated_distance = total_length / frame_count;
    int generated = 0;
    for (int circle = 0; circle < circle_count; ++circle) {
        generated += static_cast<int>(std::lround(
            2.0 * M_PI * std::sin(circle * delta_theta) / recalculated_distance));
    }
    return generated;
}

bool annular_counts_fit(int frame_count, int circle_count)
{
    if (circle_count <= 0)
        return false;
    double radius_sum = 0.0;
    for (int circle = 0; circle < circle_count; ++circle)
        radius_sum += static_cast<double>(circle_count - circle) / circle_count;
    int assigned_inner = 0;
    for (int circle = circle_count - 1; circle > 0; --circle) {
        const double radius = static_cast<double>(circle_count - circle) / circle_count;
        assigned_inner += static_cast<int>(std::lround(radius / radius_sum * frame_count));
    }
    return assigned_inner <= frame_count;
}

bool finite_pair(double x, double y)
{
    return std::isfinite(x) && std::isfinite(y);
}

bool fail(const std::string& message, std::string& error)
{
    error = message;
    return false;
}

} // namespace

bool build_scan_frame_commands(
    const std::string& pattern,
    int frame_count,
    Var2D dimensions,
    int annular_circle_count,
    double vxmin,
    double vxmax,
    double vymin,
    double vymax,
    double na_cond_lim,
    unsigned int random_seed,
    std::vector<ScanFrameCommand>& commands,
    std::string& error,
    const std::function<bool()>& stop_requested)
{
    commands.clear();
    error.clear();

    const bool supported = pattern == "ROSACE" || pattern == "FERMAT"
        || pattern == "ARCHIMEDE" || pattern == "ANNULAR"
        || pattern == "UNIFORM3D" || pattern == "RANDOM_POLAR"
        || pattern == "STAR";
    if (!supported)
        return fail("unsupported scan pattern: " + pattern, error);
    if (frame_count < 2)
        return fail("NB_HOLO must be at least 2", error);
    if (dimensions.x <= 0 || dimensions.y <= 0)
        return fail("scan dimensions must be positive", error);
    if (!finite_pair(vxmin, vxmax) || !finite_pair(vymin, vymax))
        return fail("voltage limits must be finite", error);
    if (!std::isfinite(na_cond_lim) || na_cond_lim <= 0.0 || na_cond_lim > 1.0)
        return fail("NA_COND_LIM must be in (0, 1]", error);
    if (pattern == "ANNULAR" && !annular_counts_fit(frame_count, annular_circle_count))
        return fail("invalid ANNULAR point distribution", error);
    if (pattern == "ARCHIMEDE" && frame_count > 10000)
        return fail("ARCHIMEDE supports at most 10000 frames", error);
    if (pattern == "RANDOM_POLAR"
        && static_cast<long long>(frame_count) > static_cast<long long>(dimensions.x) * dimensions.y)
        return fail("RANDOM_POLAR requests more unique points than the scan grid", error);
    if (pattern == "UNIFORM3D") {
        const int generated = uniform3d_generated_count(frame_count);
        if (generated < frame_count - 1 || generated > frame_count)
            return fail("UNIFORM3D does not generate a capture-aligned point count", error);
    }

    std::vector<float2D> generated(frame_count);
    const double rho = kNormalizedLimit;
    if (pattern == "ROSACE")
        scan_rose(rho, frame_count, generated);
    else if (pattern == "FERMAT")
        scan_fermat(rho, frame_count, generated, dimensions);
    else if (pattern == "ARCHIMEDE")
        scan_spiralOS(rho, frame_count, generated, 4);
    else if (pattern == "ANNULAR")
        scan_annular(rho, annular_circle_count, frame_count, generated);
    else if (pattern == "UNIFORM3D")
        scan_uniform3D(rho, frame_count, generated);
    else if (pattern == "RANDOM_POLAR") {
        std::srand(random_seed);
        if (!scan_random_polar3D(rho, frame_count, generated, dimensions, error, stop_requested))
            return false;
    } else if (pattern == "STAR")
        scan_star(rho, frame_count, 3, generated);

    const double factor_x = (std::abs(vxmin) + std::abs(vxmax)) * na_cond_lim / 20.0;
    const double factor_y = (std::abs(vymin) + std::abs(vymax)) * na_cond_lim / 20.0;
    const double offset_x = vxmin * na_cond_lim + factor_x * 10.0;
    const double offset_y = vymin * na_cond_lim + factor_y * 10.0;
    if (!finite_pair(offset_x, offset_y)
        || std::abs(offset_x) > kCommandLimit || std::abs(offset_y) > kCommandLimit)
        return fail("centre voltage lies outside the DAC range", error);

    commands.reserve(frame_count);
    ScanFrameCommand centre = {0, -1, 0.0, 0.0, offset_x, offset_y};
    commands.push_back(centre);
    for (int image_index = 1; image_index < frame_count; ++image_index) {
        const float2D& point = generated[image_index - 1];
        if (!finite_pair(point.x, point.y)
            || std::abs(point.x) > kNormalizedLimit + 1e-3
            || std::abs(point.y) > kNormalizedLimit + 1e-3)
            return fail("scan generator produced an invalid normalized command", error);
        const double command_x = point.x * factor_x + offset_x;
        const double command_y = point.y * factor_y + offset_y;
        if (!finite_pair(command_x, command_y)
            || std::abs(command_x) > kCommandLimit
            || std::abs(command_y) > kCommandLimit)
            return fail("scan generator produced a command outside the DAC range", error);
        ScanFrameCommand command = {
            image_index,
            image_index - 1,
            point.x,
            point.y,
            command_x,
            command_y
        };
        commands.push_back(command);
    }
    return true;
}

bool scan_frame_commands_csv(
    const std::vector<ScanFrameCommand>& commands,
    std::string& text,
    std::string& error,
    bool fsm_schedule)
{
    text.clear();
    error.clear();
    if (commands.empty())
        return fail("cannot write an empty frame-command table", error);
    for (std::size_t row = 0; row < commands.size(); ++row) {
        const ScanFrameCommand& command = commands[row];
        const int scan_command_index = fsm_schedule ? static_cast<int>(row)
                                                    : (row == 0 ? -1 : static_cast<int>(row) - 1);
        if (command.image_index != static_cast<int>(row)
            || command.scan_command_index != scan_command_index
            || !finite_pair(command.normalized_scan_x, command.normalized_scan_y)
            || !finite_pair(command.command_vx_v, command.command_vy_v))
            return fail("frame-command table violates its row contract", error);
    }

    std::ostringstream out;
    out << "schema_version,image_index,scan_command_index,normalized_scan_x,"
           "normalized_scan_y,command_vx_v,command_vy_v\n";
    out << std::setprecision(17);
    for (std::size_t row = 0; row < commands.size(); ++row) {
        const ScanFrameCommand& command = commands[row];
        out << 1 << ',' << command.image_index << ',' << command.scan_command_index << ','
            << command.normalized_scan_x << ',' << command.normalized_scan_y << ','
            << command.command_vx_v << ',' << command.command_vy_v << '\n';
    }
    if (!out.good())
        return fail("failed while formatting frame-command table", error);
    text = out.str();
    return true;
}

double maximum_scan_command_step_volts(
    const std::vector<ScanFrameCommand>& commands)
{
    double maximum_step = 0.0;
    for (std::size_t row = 1; row < commands.size(); ++row) {
        const double delta_x = commands[row].command_vx_v
            - commands[row - 1].command_vx_v;
        const double delta_y = commands[row].command_vy_v
            - commands[row - 1].command_vy_v;
        maximum_step = std::max(maximum_step, std::hypot(delta_x, delta_y));
    }
    return maximum_step;
}

double scan_command_step_volts_quantile(
    const std::vector<ScanFrameCommand>& commands, double fraction)
{
    std::vector<double> steps;
    for (std::size_t row = 1; row < commands.size(); ++row)
        steps.push_back(std::hypot(commands[row].command_vx_v - commands[row - 1].command_vx_v,
                                   commands[row].command_vy_v - commands[row - 1].command_vy_v));
    if (steps.empty())
        return 0.0;
    // nearest rank: the step below which `fraction` of the steps lie
    const double rank = std::min(std::max(std::ceil(fraction * steps.size()), 1.0), double(steps.size()));
    const std::size_t index = std::size_t(rank) - 1;
    std::nth_element(steps.begin(), steps.begin() + index, steps.end());
    return steps[index];
}

int default_fsm_settle_ms(const std::vector<ScanFrameCommand>& commands)
{
    if (commands.empty())
        return kConservativeSettleMilliseconds;
    const double step = maximum_scan_command_step_volts(commands);
    const double typical = scan_command_step_volts_quantile(commands, kTypicalStepFraction);
    return step <= kSmallStepLimitVolts && typical <= kSmallTypicalStepLimitVolts ? kSmallStepSettleMilliseconds
        : step <= kTestedStepLimitVolts ? kSettleMilliseconds
        : kConservativeSettleMilliseconds;
}

namespace {

const char kFsmScheduleHeader[] = "vx_v,vy_v,settle_ms,label";
const std::streamoff kFsmScheduleMaxBytes = 8 << 20;
// get_val reads the limits as float: a schedule voltage written as the config's limit can lie just outside them.
const double kFsmScheduleLimitToleranceVolts = 1e-6;

std::string schedule_number(double value)
{
    std::ostringstream text;
    text.imbue(std::locale::classic());
    text << value;
    return text.str();
}

bool schedule_line_fail(std::size_t line, const std::string& message, std::vector<FsmScheduleRow>& rows,
                        std::string& error)
{
    rows.clear();
    std::ostringstream text;
    text << "line " << line << ": " << message;
    error = text.str();
    return false;
}

// The whole field is one finite number in the C locale (blanks around it allowed).
bool schedule_volts(const std::string& field, double& value)
{
    std::istringstream input(field);
    input.imbue(std::locale::classic());
    char rest;
    return (input >> value) && !(input >> rest) && std::isfinite(value);
}

// Digits only (blanks around them allowed), 0 to kFsmScheduleMaxSettleMs.
bool schedule_settle(const std::string& field, int& value)
{
    const std::size_t begin = field.find_first_not_of(" \t");
    if (begin == std::string::npos)
        return false;
    const std::size_t end = field.find_last_not_of(" \t") + 1;
    const std::string digits = field.substr(begin, end - begin);
    if (digits.size() > 4 || digits.find_first_not_of("0123456789") != std::string::npos)
        return false;
    value = std::atoi(digits.c_str());
    return value <= kFsmScheduleMaxSettleMs;
}

bool schedule_axis(double min, double max)
{
    return finite_pair(min, max) && min <= max && std::abs(min) <= kCommandLimit && std::abs(max) <= kCommandLimit;
}

} // namespace

bool parse_fsm_schedule(
    const std::string& text,
    double vxmin,
    double vxmax,
    double vymin,
    double vymax,
    double na_cond_lim,
    std::vector<FsmScheduleRow>& rows,
    std::string& error)
{
    rows.clear();
    error.clear();
    if (!schedule_axis(vxmin, vxmax))
        return fail("VXMIN..VXMAX (" + schedule_number(vxmin) + ".." + schedule_number(vxmax) +
                    " V) is not a range inside +/-10 V", error);
    if (!schedule_axis(vymin, vymax))
        return fail("VYMIN..VYMAX (" + schedule_number(vymin) + ".." + schedule_number(vymax) +
                    " V) is not a range inside +/-10 V", error);
    if (!std::isfinite(na_cond_lim) || na_cond_lim <= 0.0 || na_cond_lim > 1.0)
        return fail("NA_COND_LIM must be in (0, 1]: it places the scan centre the mirror starts at", error);
    bool header = false;
    std::size_t line_number = 0;
    for (std::size_t from = 0; from < text.size();) {
        const std::size_t newline = text.find('\n', from);
        std::string line = text.substr(from, newline == std::string::npos ? std::string::npos : newline - from);
        from = newline == std::string::npos ? text.size() : newline + 1;
        ++line_number;
        if (line_number == 1 && line.compare(0, 3, "\xEF\xBB\xBF") == 0)
            line.erase(0, 3);   // a UTF-8 byte order mark
        if (!line.empty() && line[line.size() - 1] == '\r')
            line.erase(line.size() - 1);
        if (line.find_first_not_of(" \t") == std::string::npos || line[0] == '#')
            continue;
        if (!header) {
            if (line != kFsmScheduleHeader)
                return schedule_line_fail(line_number, std::string("the header must be ") + kFsmScheduleHeader,
                                          rows, error);
            header = true;
            continue;
        }
        if (rows.size() == static_cast<std::size_t>(kFsmScheduleMaxRows))
            return schedule_line_fail(line_number, "more than 4000 rows", rows, error);
        std::vector<std::string> fields;
        for (std::size_t start = 0;;) {
            const std::size_t comma = line.find(',', start);
            fields.push_back(line.substr(start, comma == std::string::npos ? std::string::npos : comma - start));
            if (comma == std::string::npos)
                break;
            start = comma + 1;
        }
        if (fields.size() != 4) {
            std::ostringstream message;
            message << fields.size() << " fields, not 4 (vx_v,vy_v,settle_ms,label; a label has no commas)";
            return schedule_line_fail(line_number, message.str(), rows, error);
        }
        FsmScheduleRow row;
        if (!schedule_volts(fields[0], row.vx_v))
            return schedule_line_fail(line_number, "vx_v '" + fields[0] + "' is not a finite number", rows, error);
        if (!schedule_volts(fields[1], row.vy_v))
            return schedule_line_fail(line_number, "vy_v '" + fields[1] + "' is not a finite number", rows, error);
        if (!schedule_settle(fields[2], row.settle_ms))
            return schedule_line_fail(line_number, "settle_ms '" + fields[2] + "' is not an integer from 0 to 1000",
                                      rows, error);
        if (row.vx_v < vxmin - kFsmScheduleLimitToleranceVolts || row.vx_v > vxmax + kFsmScheduleLimitToleranceVolts)
            return schedule_line_fail(line_number, "vx_v " + schedule_number(row.vx_v) + " V is outside VXMIN..VXMAX (" +
                                      schedule_number(vxmin) + ".." + schedule_number(vxmax) + " V)", rows, error);
        if (row.vy_v < vymin - kFsmScheduleLimitToleranceVolts || row.vy_v > vymax + kFsmScheduleLimitToleranceVolts)
            return schedule_line_fail(line_number, "vy_v " + schedule_number(row.vy_v) + " V is outside VYMIN..VYMAX (" +
                                      schedule_number(vymin) + ".." + schedule_number(vymax) + " V)", rows, error);
        // Clamp the tolerance away: the DAC never gets a command outside the limits.
        row.vx_v = std::min(std::max(row.vx_v, vxmin), vxmax);
        row.vy_v = std::min(std::max(row.vy_v, vymin), vymax);
        row.label = fields[3];
        rows.push_back(row);
    }
    if (!header)
        return fail(std::string("no header line (") + kFsmScheduleHeader + ")", error);
    if (rows.empty())
        return fail("no frames: a schedule has 1 to 4000 rows", error);
    return true;
}

bool read_fsm_schedule(
    const std::string& path,
    double vxmin,
    double vxmax,
    double vymin,
    double vymax,
    double na_cond_lim,
    std::string& text,
    std::vector<FsmScheduleRow>& rows,
    std::string& error)
{
    text.clear();
    rows.clear();
    std::ifstream input(path.c_str(), std::ios::binary);
    if (!input)
        return fail(std::string("cannot open it: ") + std::strerror(errno), error);
    input.seekg(0, std::ios::end);
    const std::streamoff bytes = input.tellg();
    if (bytes < 0)
        return fail("cannot read it", error);
    if (bytes > kFsmScheduleMaxBytes)
        return fail("larger than 8 MB", error);
    input.seekg(0, std::ios::beg);
    text.assign(static_cast<std::size_t>(bytes), '\0');
    if (bytes > 0 && !input.read(&text[0], bytes)) {
        text.clear();
        return fail("cannot read it", error);
    }
    if (!parse_fsm_schedule(text, vxmin, vxmax, vymin, vymax, na_cond_lim, rows, error)) {
        text.clear();
        return false;
    }
    return true;
}

bool build_fsm_schedule_frame_commands(
    const std::vector<FsmScheduleRow>& rows,
    double vxmin,
    double vxmax,
    double vymin,
    double vymax,
    double na_cond_lim,
    std::vector<ScanFrameCommand>& commands,
    std::string& error)
{
    commands.clear();
    error.clear();
    if (rows.empty() || rows.size() > static_cast<std::size_t>(kFsmScheduleMaxRows))
        return fail("an FSM schedule has 1 to 4000 rows", error);
    if (!schedule_axis(vxmin, vxmax) || !schedule_axis(vymin, vymax)
        || !std::isfinite(na_cond_lim) || na_cond_lim <= 0.0 || na_cond_lim > 1.0)
        return fail("invalid FSM schedule limits", error);
    // The scan's mapping (build_scan_frame_commands): command = normalized * factor + offset.
    const double factor_x = (std::abs(vxmin) + std::abs(vxmax)) * na_cond_lim / 20.0;
    const double factor_y = (std::abs(vymin) + std::abs(vymax)) * na_cond_lim / 20.0;
    const double offset_x = vxmin * na_cond_lim + factor_x * 10.0;
    const double offset_y = vymin * na_cond_lim + factor_y * 10.0;
    commands.reserve(rows.size());
    for (std::size_t k = 0; k < rows.size(); ++k) {
        const FsmScheduleRow& row = rows[k];
        if (!finite_pair(row.vx_v, row.vy_v) || row.vx_v < vxmin || row.vx_v > vxmax
            || row.vy_v < vymin || row.vy_v > vymax) {
            commands.clear();
            return fail("FSM schedule row outside its limits", error);
        }
        ScanFrameCommand command = {
            static_cast<int>(k),
            static_cast<int>(k),
            factor_x > 0.0 ? (row.vx_v - offset_x) / factor_x : 0.0,
            factor_y > 0.0 ? (row.vy_v - offset_y) / factor_y : 0.0,
            row.vx_v,
            row.vy_v
        };
        commands.push_back(command);
    }
    return true;
}
