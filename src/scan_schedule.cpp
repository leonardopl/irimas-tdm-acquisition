#include "scan_schedule.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace {

const double kNormalizedLimit = 10.0;
const double kCommandLimit = 10.0;

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
    std::string& error)
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
        scan_random_polar3D(rho, frame_count, generated, dimensions);
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

bool write_scan_frame_commands_csv(
    const std::string& path,
    const std::vector<ScanFrameCommand>& commands,
    std::string& error)
{
    error.clear();
    if (commands.empty())
        return fail("cannot write an empty frame-command table", error);
    for (std::size_t row = 0; row < commands.size(); ++row) {
        const ScanFrameCommand& command = commands[row];
        if (command.image_index != static_cast<int>(row)
            || command.scan_command_index != (row == 0 ? -1 : static_cast<int>(row) - 1)
            || !finite_pair(command.normalized_scan_x, command.normalized_scan_y)
            || !finite_pair(command.command_vx_v, command.command_vy_v))
            return fail("frame-command table violates its row contract", error);
    }

    const std::string temporary = path + ".tmp";
    std::ofstream out(temporary.c_str(), std::ios::out | std::ios::trunc);
    if (!out)
        return fail("cannot open temporary frame-command table: " + temporary, error);
    out << "schema_version,image_index,scan_command_index,normalized_scan_x,"
           "normalized_scan_y,command_vx_v,command_vy_v\n";
    out << std::setprecision(17);
    for (std::size_t row = 0; row < commands.size(); ++row) {
        const ScanFrameCommand& command = commands[row];
        out << 1 << ',' << command.image_index << ',' << command.scan_command_index << ','
            << command.normalized_scan_x << ',' << command.normalized_scan_y << ','
            << command.command_vx_v << ',' << command.command_vy_v << '\n';
    }
    out.close();
    if (!out.good()) {
        std::remove(temporary.c_str());
        return fail("failed while writing frame-command table", error);
    }
    if (std::rename(temporary.c_str(), path.c_str()) != 0) {
        std::remove(temporary.c_str());
        return fail("cannot publish frame-command table: " + path, error);
    }
    return true;
}
