#include "scan_schedule.h"
#include "frame_timing.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {

void check_pattern(const std::string& pattern, int frame_count)
{
    std::vector<ScanFrameCommand> commands;
    std::string error;
    const bool ok = build_scan_frame_commands(
        pattern, frame_count, Var2D{256, 256}, 3,
        -3.0, 5.0, -4.0, 2.0, 0.6, 12345,
        commands, error);
    assert(ok);
    assert(error.empty());
    assert(commands.size() == static_cast<std::size_t>(frame_count));
    assert(commands[0].image_index == 0);
    assert(commands[0].scan_command_index == -1);
    assert(commands[0].normalized_scan_x == 0.0);
    assert(commands[0].normalized_scan_y == 0.0);
    for (int image = 1; image < frame_count; ++image) {
        const ScanFrameCommand& command = commands[image];
        assert(command.image_index == image);
        assert(command.scan_command_index == image - 1);
        assert(std::isfinite(command.normalized_scan_x));
        assert(std::isfinite(command.normalized_scan_y));
        assert(std::abs(command.normalized_scan_x) <= 10.001);
        assert(std::abs(command.normalized_scan_y) <= 10.001);
        assert(std::abs(command.command_vx_v) <= 10.0);
        assert(std::abs(command.command_vy_v) <= 10.0);
    }
}

void check_fermat_rings(int frames)
{
    const double vmax = 10.0;
    std::vector<float2D> table(frames, float2D{123.0f, -456.0f});
    std::vector<float2D> large_grid = table, repeated = table;
    scan_fermat(vmax, frames, table, Var2D{146, 146});
    scan_fermat(vmax, frames, large_grid, Var2D{2048, 2048});
    scan_fermat(vmax, frames, repeated, Var2D{146, 146});
    for (int i = 0; i < frames; ++i) {
        assert(table[i].x == large_grid[i].x && table[i].y == large_grid[i].y);
        assert(table[i].x == repeated[i].x && table[i].y == repeated[i].y);
        assert(std::hypot(double(table[i].x), double(table[i].y)) <= vmax + 1e-6);
    }
    assert(table.back().x == 0.0f && table.back().y == 0.0f);
    const int captured = frames - 1;  // the last row is never captured

    const double golden = M_PI * (3.0 - std::sqrt(5.0));
    const int ring_count = std::max(1, int(std::lround(std::sqrt((frames - 1) / M_PI))));
    std::vector<bool> seen(frames, false);
    std::vector<double> angles(frames);
    std::vector<std::vector<int> > ring_points(ring_count);
    int previous_ring = -1;
    for (int i = 0; i < captured; ++i) {
        const double x = table[i].x, y = table[i].y;
        // Recover k from radius alone, then verify the unsnapped spiral coordinates.
        const int k = int(std::lround((x * x + y * y) * frames / (vmax * vmax)));
        assert(k >= 1 && k < frames && !seen[k]);
        seen[k] = true;
        const double r = vmax * std::sqrt(double(k) / frames);
        angles[k] = std::fmod(k * golden, 2.0 * M_PI);
        assert(std::hypot(x - r * std::cos(angles[k]), y - r * std::sin(angles[k])) < 1e-6);
        const int ring = std::min(ring_count - 1, int((static_cast<long long>(ring_count) * k) / frames));
        assert(ring >= previous_ring);
        previous_ring = ring;
        ring_points[ring].push_back(k);
    }
    for (int k = 1; k < frames; ++k)
        assert(seen[k]);

    // Each ring starts at the smallest relative angle, then walks in alternating directions.
    double last = 0.0;
    for (int ring = 0; ring < ring_count; ++ring) {
        const std::vector<int>& points = ring_points[ring];
        if (points.empty())
            continue;
        const std::pair<double, int> first(
            std::fmod(angles[points.front()] - last + 4.0 * M_PI, 2.0 * M_PI), points.front());
        std::pair<double, int> previous = first;
        for (std::size_t i = 1; i < points.size(); ++i) {
            const std::pair<double, int> current(
                std::fmod(angles[points[i]] - last + 4.0 * M_PI, 2.0 * M_PI), points[i]);
            assert(first < current);
            if (ring % 2 == 0)
                assert(previous < current);
            else if (i > 1)
                assert(current < previous);
            previous = current;
        }
        last = angles[points.back()];
    }

    if (frames != 400)
        return;
    double sum = 0.0, sum2 = 0.0;
    for (int i = 0; i < captured; ++i) {
        double nearest = 1e30;
        for (int j = 0; j < captured; ++j)
            if (j != i)
                nearest = std::min(nearest, std::hypot(double(table[i].x - table[j].x), double(table[i].y - table[j].y)));
        sum += nearest;
        sum2 += nearest * nearest;
    }
    const double mean = sum / captured;
    const double cv = std::sqrt(std::max(0.0, sum2 / captured - mean * mean)) / mean;
    std::printf("FERMAT nearest-neighbour spacing CV %.3f\n", cv);
    assert(cv < 0.05);
}

std::vector<ScanFrameCommand> fermat_rig_commands(Var2D dim = Var2D{146, 146})
{
    std::vector<ScanFrameCommand> commands;
    std::string error;
    const bool ok = build_scan_frame_commands(
        "FERMAT", 400, dim, 4,
        -3.7759, 3.9288, -2.7571, 2.5481, 0.978, 1, commands, error);
    assert(ok && error.empty());
    return commands;
}

void check_fermat_rig_fixture()
{
    const std::vector<ScanFrameCommand> commands = fermat_rig_commands();
    const std::vector<ScanFrameCommand> large_grid = fermat_rig_commands(Var2D{2048, 2048});
    const std::vector<ScanFrameCommand> repeated = fermat_rig_commands();
    const double maximum = maximum_scan_command_step_volts(commands);
    const double p95 = scan_command_step_volts_quantile(commands, 0.95);
    std::printf("FERMAT rig 400 steps: max %.9f V, p95 %.9f V\n", maximum, p95);
    assert(maximum <= 0.9);
    assert(default_fsm_settle_ms(commands) == 10);

    std::ifstream fixture("tests/fixtures/fermat_rings_rig_400.csv");
    assert(fixture.is_open());
    std::string line;
    assert(std::getline(fixture, line));
    assert(line == "schema_version,image_index,scan_command_index,normalized_scan_x,"
                   "normalized_scan_y,command_vx_v,command_vy_v");
    for (std::size_t row = 0; row < commands.size(); ++row) {
        const ScanFrameCommand& expected = commands[row];
        for (const ScanFrameCommand* other : {&large_grid[row], &repeated[row]}) {
            assert(expected.image_index == other->image_index);
            assert(expected.scan_command_index == other->scan_command_index);
            assert(expected.normalized_scan_x == other->normalized_scan_x);
            assert(expected.normalized_scan_y == other->normalized_scan_y);
            assert(expected.command_vx_v == other->command_vx_v);
            assert(expected.command_vy_v == other->command_vy_v);
        }
        assert(std::getline(fixture, line));
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream fields(line);
        int schema_version = 0;
        ScanFrameCommand actual = {};
        assert(fields >> schema_version >> actual.image_index >> actual.scan_command_index
                      >> actual.normalized_scan_x >> actual.normalized_scan_y
                      >> actual.command_vx_v >> actual.command_vy_v);
        fields >> std::ws;
        assert(fields.eof());
        assert(schema_version == 1);
        assert(actual.image_index == expected.image_index);
        assert(actual.scan_command_index == expected.scan_command_index);
        assert(std::abs(actual.normalized_scan_x - expected.normalized_scan_x) <= 1e-9);
        assert(std::abs(actual.normalized_scan_y - expected.normalized_scan_y) <= 1e-9);
        assert(std::abs(actual.command_vx_v - expected.command_vx_v) <= 1e-9);
        assert(std::abs(actual.command_vy_v - expected.command_vy_v) <= 1e-9);
    }
    assert(!std::getline(fixture, line) && fixture.eof());
}

} // namespace

int main(int argc, char* argv[])
{
    if (argc > 1) {
        if (argc != 3 || std::string(argv[1]) != "--write-fermat-fixture") {
            std::fprintf(stderr, "Usage: %s [--write-fermat-fixture PATH]\n", argv[0]);
            return 1;
        }
        std::string error;
        if (!write_scan_frame_commands_csv(argv[2], fermat_rig_commands(), error)) {
            std::fprintf(stderr, "%s\n", error.c_str());
            return 1;
        }
        return 0;
    }

    for (int frames : {2, 3, 4, 7, 40, 128, 400, 800})
        check_fermat_rings(frames);
    check_fermat_rig_fixture();
    const char* patterns[] = {
        "ROSACE", "FERMAT", "ARCHIMEDE", "ANNULAR",
        "UNIFORM3D", "RANDOM_POLAR", "STAR"
    };
    for (const char* pattern : patterns) {
        check_pattern(pattern, 40);
        check_pattern(pattern, 400);
    }

    std::vector<ScanFrameCommand> commands;
    std::string error;
    assert(build_scan_frame_commands(
        "UNIFORM3D", 400, Var2D{146, 146}, 4,
        -3.4, 3.4, -2.5, 2.5, 0.8, 1, commands, error));
    assert(std::abs(maximum_scan_command_step_volts(commands) - 0.529872) < 1e-5);
    assert(default_fsm_settle_ms(commands) == 5);
    assert(!build_scan_frame_commands(
        "UNKNOWN", 40, Var2D{256, 256}, 3,
        -3.0, 5.0, -4.0, 2.0, 0.6, 1, commands, error));
    assert(!build_scan_frame_commands(
        "UNIFORM3D", 36, Var2D{256, 256}, 3,
        -3.0, 5.0, -4.0, 2.0, 0.6, 1, commands, error));
    assert(!build_scan_frame_commands(
        "ROSACE", 1, Var2D{256, 256}, 3,
        -3.0, 5.0, -4.0, 2.0, 0.6, 1, commands, error));

    assert(build_scan_frame_commands(
        "RANDOM_POLAR", 40, Var2D{256, 256}, 3,
        -3.0, 5.0, -4.0, 2.0, 0.6, 12345, commands, error));
    std::vector<ScanFrameCommand> repeated;
    assert(build_scan_frame_commands(
        "RANDOM_POLAR", 40, Var2D{256, 256}, 3,
        -3.0, 5.0, -4.0, 2.0, 0.6, 12345, repeated, error));
    for (std::size_t row = 0; row < commands.size(); ++row) {
        assert(commands[row].normalized_scan_x == repeated[row].normalized_scan_x);
        assert(commands[row].normalized_scan_y == repeated[row].normalized_scan_y);
    }

    std::vector<ScanFrameCommand> settling_commands(3);
    settling_commands[0].command_vx_v = 0.0;
    settling_commands[0].command_vy_v = 0.0;
    settling_commands[1].command_vx_v = 0.3;
    settling_commands[1].command_vy_v = 0.4;
    settling_commands[2].command_vx_v = 0.6;
    settling_commands[2].command_vy_v = 0.4;
    assert(std::abs(maximum_scan_command_step_volts(settling_commands) - 0.5) < 1e-12);
    assert(default_fsm_settle_ms(settling_commands) == 5);
    settling_commands[2].command_vx_v = 1.04;   // a 0.74 V step of two: 95 % of the steps are not small
    assert(default_fsm_settle_ms(settling_commands) == 10);
    std::vector<ScanFrameCommand> mostly_small(21);   // 19 steps of 0.4 V, then one of 0.74 V
    for (std::size_t row = 1; row < mostly_small.size(); ++row)
        mostly_small[row].command_vx_v = mostly_small[row - 1].command_vx_v + (row + 1 < mostly_small.size() ? 0.4 : 0.74);
    assert(std::abs(scan_command_step_volts_quantile(mostly_small, 0.95) - 0.4) < 1e-12);
    assert(default_fsm_settle_ms(mostly_small) == 5);
    assert(scan_command_step_volts_quantile(std::vector<ScanFrameCommand>(), 0.95) == 0.0);
    settling_commands[2].command_vx_v = 1.2;    // 0.9 V
    assert(default_fsm_settle_ms(settling_commands) == 10);
    settling_commands[2].command_vx_v = 0.3;    // the intensity-reference move (0, 8 V): untested size
    settling_commands[2].command_vy_v = 8.0;
    assert(default_fsm_settle_ms(settling_commands) == 100);
    settling_commands[2].command_vx_v = 0.6;
    settling_commands[2].command_vy_v = 0.4;
    assert(default_fsm_settle_ms(std::vector<ScanFrameCommand>()) == 100);

    const std::string path = "/tmp/irimas_fsm_frame_commands_test.csv";
    assert(write_scan_frame_commands_csv(path, commands, error));
    std::ifstream input(path.c_str());
    std::string line;
    int lines = 0;
    while (std::getline(input, line))
        ++lines;
    assert(lines == 41);
    std::remove(path.c_str());

    // Per-frame timing CSV (B192): header, one row per frame, a failed frame's -1 fields, an unwritable path.
    std::vector<FrameTiming> timing(2);
    timing[0].image_index = 0;
    timing[0].dac_done_ms = 1.25;
    timing[0].buffer_ms = 17.5;
    timing[0].written_ms = 20.0;
    timing[0].frame_id = 7;
    timing[0].camera_timestamp_ns = 123456789012ULL;
    timing[1].image_index = 1;
    const std::string timing_path = "/tmp/irimas_frame_timing_test.csv";
    assert(write_frame_timing_csv(timing_path, timing, 5, error));
    std::ifstream timing_input(timing_path.c_str());
    std::vector<std::string> timing_lines;
    while (std::getline(timing_input, line))
        timing_lines.push_back(line);
    assert(timing_lines.size() == 3);
    assert(timing_lines[0] == "schema_version,image_index,settle_ms,dac_start_ms,dac_done_ms,trigger_start_ms,"
                              "trigger_done_ms,buffer_ms,written_ms,frame_id,camera_timestamp_ns,system_timestamp_ns,"
                              "exposure_start_host_ms,next_dac_slack_ms");
    assert(timing_lines[1] == "2,0,5,0.000,1.250,0.000,0.000,17.500,20.000,7,123456789012,0,,");
    assert(timing_lines[2] == "2,1,5,0.000,0.000,0.000,0.000,-1.000,-1.000,0,0,0,,");
    std::remove(timing_path.c_str());
    assert(!write_frame_timing_csv("/nonexistent-dir/frame_timing.csv", timing, 5, error) && !error.empty());
    return 0;
}
