#include "scan_schedule.h"
#include "frame_timing.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <unistd.h>
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

// The rig's limits (tests/fixtures/fermat_rings_rig_400.csv) as get_val reads them: float.
const double kVxmin = float(-3.7759), kVxmax = float(3.9288), kVymin = float(-2.7571), kVymax = float(2.5481);
const double kNaCondLim = float(0.978);

bool parse(const std::string& text, std::vector<FsmScheduleRow>& rows, std::string& error)
{
    return parse_fsm_schedule(text, kVxmin, kVxmax, kVymin, kVymax, kNaCondLim, rows, error);
}

// One refused schedule: false, no rows, the reason in one line.
void refused(const std::string& text, const std::string& reason)
{
    std::vector<FsmScheduleRow> rows(1);
    std::string error;
    const bool ok = parse(text, rows, error);
    if (ok || !rows.empty() || error.find(reason) == std::string::npos || error.find('\n') != std::string::npos)
        std::fprintf(stderr, "schedule not refused as expected (%s): %s\n", reason.c_str(), error.c_str());
    assert(!ok && rows.empty() && error.find(reason) != std::string::npos && error.find('\n') == std::string::npos);
}

// B199: an FSM schedule (vx_v,vy_v,settle_ms,label) is read whole and every row checked before the program moves.
void check_fsm_schedule()
{
    const std::string header = "vx_v,vy_v,settle_ms,label\n";
    // Comments and blank lines anywhere, a byte order mark, CRLF, an empty label, the limits themselves (as the config
    // writes them: the float limits round them by up to ~1e-7 V).
    const std::string good = "\xEF\xBB\xBF# B199 probe\n" + header +
        "# fsm_settle_probe.py writes its comments after the header\n"
        "0.075,-0.102,1000,reference centre\r\n"
        "# a 2 V move\n"
        "\n"
        "2.075,-0.102,2,step 2V settle 2\n"
        "-3.7759,2.5481,20,\n"
        "3.9288,-2.7571,0,corner\n";
    std::vector<FsmScheduleRow> rows;
    std::string error;
    assert(parse(good, rows, error) && error.empty() && rows.size() == 4);
    assert(rows[0].vx_v == 0.075 && rows[0].vy_v == -0.102 && rows[0].settle_ms == 1000);
    assert(rows[0].label == "reference centre" && rows[1].label == "step 2V settle 2" && rows[2].label.empty());
    // A value at a limit is kept inside it (clamped by at most the float rounding).
    const double at_limits[][2] = {{rows[2].vx_v, -3.7759}, {rows[2].vy_v, 2.5481}, {rows[3].vx_v, 3.9288},
                                   {rows[3].vy_v, -2.7571}};
    for (const auto& pair : at_limits)
        assert(std::abs(pair[0] - pair[1]) < 1e-6);
    for (const FsmScheduleRow& row : rows)
        assert(row.vx_v >= kVxmin && row.vx_v <= kVxmax && row.vy_v >= kVymin && row.vy_v <= kVymax);
    assert(rows[2].vx_v == kVxmin);   // float(-3.7759) lies above -3.7759: clamped
    assert(rows[2].settle_ms == 20 && rows[3].settle_ms == 0);

    // Each refusal names the file's line.
    refused("", "no header line");
    refused("# only a comment\n", "no header line");
    refused(header, "no frames");
    refused(header + "# no rows\n\n", "no frames");
    refused("vx,vy,settle_ms,label\n0,0,5,\n", "line 1: the header must be vx_v,vy_v,settle_ms,label");
    refused("0,0,5,\n", "line 1: the header must be");
    refused(header + "0,0,5\n", "line 2: 3 fields, not 4");
    refused(header + "0,0,5,a,b\n", "line 2: 5 fields, not 4");
    refused(header + "0,0,5,\nnan,0,5,\n", "line 3: vx_v 'nan' is not a finite number");
    refused(header + "0,inf,5,\n", "line 2: vy_v 'inf' is not a finite number");
    refused(header + "0,1e999,5,\n", "line 2: vy_v '1e999' is not a finite number");
    refused(header + ",0,5,\n", "line 2: vx_v '' is not a finite number");
    refused(header + "0.5V,0,5,\n", "line 2: vx_v '0.5V' is not a finite number");
    refused(header + "0,0,-1,\n", "line 2: settle_ms '-1' is not an integer from 0 to 1000");
    refused(header + "0,0,1001,\n", "settle_ms '1001' is not an integer from 0 to 1000");
    refused(header + "0,0,5.0,\n", "settle_ms '5.0' is not an integer");
    refused(header + "0,0,,\n", "settle_ms '' is not an integer");
    refused(header + "0,0,10000,\n", "settle_ms '10000' is not an integer");
    refused(header + "3.9289,0,5,\n", "line 2: vx_v 3.9289 V is outside VXMIN..VXMAX (-3.7759..3.9288 V)");
    refused(header + "-3.776,0,5,\n", "line 2: vx_v -3.776 V is outside VXMIN..VXMAX");
    refused(header + "0,2.5482,5,\n", "line 2: vy_v 2.5482 V is outside VYMIN..VYMAX (-2.7571..2.5481 V)");
    refused(header + "0,-2.7572,5,\n", "vy_v -2.7572 V is outside VYMIN..VYMAX");
    refused(header + "0,8,5,\n", "vy_v 8 V is outside");
    std::string many = header;
    for (int k = 0; k < kFsmScheduleMaxRows; ++k)
        many += "0.1,0.2,5,row\n";
    assert(parse(many, rows, error) && rows.size() == 4000);
    refused(many + "0.1,0.2,5,one too many\n", "line 4002: more than 4000 rows");
    // Limits the scans would refuse refuse the schedule too.
    assert(!parse_fsm_schedule(good, 4, -4, kVymin, kVymax, kNaCondLim, rows, error) && rows.empty());
    assert(error.find("VXMIN..VXMAX (4..-4 V) is not a range") != std::string::npos);
    assert(!parse_fsm_schedule(good, kVxmin, kVxmax, -12, kVymax, kNaCondLim, rows, error));
    assert(!parse_fsm_schedule(good, kVxmin, kVxmax, kVymin, kVymax, 0, rows, error));
    assert(error.find("NA_COND_LIM must be in (0, 1]") != std::string::npos);
    assert(!parse_fsm_schedule(good, kVxmin, kVxmax, kVymin, kVymax, NAN, rows, error));

    // The file: read whole, refused with its reason.
    const std::string path = "/tmp/irimas_fsm_schedule_test_" + std::to_string(getpid()) + ".csv";
    std::ofstream(path.c_str(), std::ios::binary) << good;
    std::string text;
    assert(read_fsm_schedule(path, kVxmin, kVxmax, kVymin, kVymax, kNaCondLim, text, rows, error));
    assert(text == good && rows.size() == 4);
    std::ofstream(path.c_str(), std::ios::binary) << header << "0,9,5,\n";
    assert(!read_fsm_schedule(path, kVxmin, kVxmax, kVymin, kVymax, kNaCondLim, text, rows, error));
    assert(text.empty() && rows.empty() && error.find("line 2: vy_v 9 V is outside") != std::string::npos);
    std::remove(path.c_str());
    assert(!read_fsm_schedule(path, kVxmin, kVxmax, kVymin, kVymax, kNaCondLim, text, rows, error));
    assert(error.find("cannot open it") != std::string::npos);

    // fsm_frame_commands.csv: row k is schedule row k; the normalized coordinates map back to its voltages as a scan's.
    assert(parse(good, rows, error));
    std::vector<ScanFrameCommand> commands;
    assert(build_fsm_schedule_frame_commands(rows, kVxmin, kVxmax, kVymin, kVymax, kNaCondLim, commands, error));
    assert(commands.size() == rows.size());
    const double factor_x = (std::abs(kVxmin) + std::abs(kVxmax)) * kNaCondLim / 20.0;
    const double factor_y = (std::abs(kVymin) + std::abs(kVymax)) * kNaCondLim / 20.0;
    const double offset_x = kVxmin * kNaCondLim + factor_x * 10.0;
    const double offset_y = kVymin * kNaCondLim + factor_y * 10.0;
    for (std::size_t k = 0; k < rows.size(); ++k) {
        const ScanFrameCommand& command = commands[k];
        assert(command.image_index == int(k) && command.scan_command_index == int(k));
        assert(command.command_vx_v == rows[k].vx_v && command.command_vy_v == rows[k].vy_v);
        assert(std::abs(command.normalized_scan_x * factor_x + offset_x - rows[k].vx_v) < 1e-12);
        assert(std::abs(command.normalized_scan_y * factor_y + offset_y - rows[k].vy_v) < 1e-12);
    }
    // The scan centre maps to (0, 0); an axis with both limits at 0 V has no mapping and gets 0.
    std::vector<FsmScheduleRow> centre(1);
    centre[0].vx_v = offset_x;
    centre[0].vy_v = offset_y;
    centre[0].settle_ms = 5;
    assert(build_fsm_schedule_frame_commands(centre, kVxmin, kVxmax, kVymin, kVymax, kNaCondLim, commands, error));
    assert(std::abs(commands[0].normalized_scan_x) < 1e-12 && std::abs(commands[0].normalized_scan_y) < 1e-12);
    centre[0].vx_v = 0;
    assert(build_fsm_schedule_frame_commands(centre, 0, 0, kVymin, kVymax, kNaCondLim, commands, error));
    assert(commands[0].normalized_scan_x == 0);
    centre[0].vx_v = 4;
    assert(!build_fsm_schedule_frame_commands(centre, kVxmin, kVxmax, kVymin, kVymax, kNaCondLim, commands, error));
    assert(commands.empty());

    // The table keeps the frame-command schema; the scan contract and the schedule's are told apart.
    assert(parse(good, rows, error));
    assert(build_fsm_schedule_frame_commands(rows, kVxmin, kVxmax, kVymin, kVymax, kNaCondLim, commands, error));
    assert(!scan_frame_commands_csv(commands, text, error));
    assert(scan_frame_commands_csv(commands, text, error, true));
    std::istringstream table(text);
    std::string line;
    std::vector<std::string> lines;
    while (std::getline(table, line))
        lines.push_back(line);
    assert(lines.size() == 5);
    assert(lines[0] == "schema_version,image_index,scan_command_index,normalized_scan_x,"
                       "normalized_scan_y,command_vx_v,command_vy_v");
    assert(lines[1].compare(0, 6, "1,0,0,") == 0 && lines[4].compare(0, 6, "1,3,3,") == 0);
    std::vector<ScanFrameCommand> scan;
    assert(build_scan_frame_commands("STAR", 4, Var2D{256, 256}, 3, kVxmin, kVxmax, kVymin, kVymax, kNaCondLim, 1,
                                     scan, error));
    assert(!scan_frame_commands_csv(scan, text, error, true));
    // Verbatim copy and filesystem failure checks moved to test_session and test_run_folder.
}

} // namespace

int main(int argc, char* argv[])
{
    if (argc > 1) {
        if (argc != 3 || std::string(argv[1]) != "--write-fermat-fixture") {
            std::fprintf(stderr, "Usage: %s [--write-fermat-fixture PATH]\n", argv[0]);
            return 1;
        }
        std::string error, text;
        if (!scan_frame_commands_csv(fermat_rig_commands(), text, error)) {
            std::fprintf(stderr, "%s\n", error.c_str());
            return 1;
        }
        // Repository fixture generation intentionally replaces the requested path.
        std::ofstream out(argv[2], std::ios::binary | std::ios::trunc);
        out << text;
        out.close();
        if (!out.good()) {
            std::fprintf(stderr, "Cannot write fixture %s\n", argv[2]);
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
    check_fsm_schedule();

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

    std::string text;
    assert(scan_frame_commands_csv(commands, text, error));
    std::istringstream input(text);
    std::string line;
    int lines = 0;
    while (std::getline(input, line))
        ++lines;
    assert(lines == 41);

    // Per-frame timing CSV (B192): header, one row per frame and a failed frame's -1 fields.
    std::vector<FrameTiming> timing(2);
    timing[0].image_index = 0;
    timing[0].dac_done_ms = 1.25;
    timing[0].buffer_ms = 17.5;
    timing[0].written_ms = 20.0;
    timing[0].frame_id = 7;
    timing[0].camera_timestamp_ns = 123456789012ULL;
    timing[1].image_index = 1;
    std::istringstream timing_input(frame_timing_csv(timing, 5));
    std::vector<std::string> timing_lines;
    while (std::getline(timing_input, line))
        timing_lines.push_back(line);
    assert(timing_lines.size() == 3);
    assert(timing_lines[0] == "schema_version,image_index,settle_ms,dac_start_ms,dac_done_ms,trigger_start_ms,"
                              "trigger_done_ms,buffer_ms,written_ms,frame_id,camera_timestamp_ns,system_timestamp_ns,"
                              "exposure_start_host_ms,next_dac_slack_ms");
    assert(timing_lines[1] == "2,0,5,0.000,1.250,0.000,0.000,17.500,20.000,7,123456789012,0,,");
    assert(timing_lines[2] == "2,1,5,0.000,0.000,0.000,0.000,-1.000,-1.000,0,0,0,,");
    // Filesystem write/refusal checks now live in test_run_folder.
    return 0;
}
