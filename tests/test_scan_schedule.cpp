#include "scan_schedule.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
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

} // namespace

int main()
{
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
    assert(default_fsm_settle_ms(commands) == 10);
    assert(build_scan_frame_commands(
        "FERMAT", 400, Var2D{146, 146}, 4,
        -3.4, 3.4, -2.5, 2.5, 0.8, 1, commands, error));
    assert(default_fsm_settle_ms(commands) == 100);

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
    assert(default_fsm_settle_ms(settling_commands) == 10);
    settling_commands[2].command_vx_v = 1.0;
    assert(default_fsm_settle_ms(settling_commands) == 100);
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
    return 0;
}
