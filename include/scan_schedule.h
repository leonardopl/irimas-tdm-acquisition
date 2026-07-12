#pragma once

#include "scan_functions.h"

#include <string>
#include <vector>

struct ScanFrameCommand {
    int image_index;
    int scan_command_index;
    double normalized_scan_x;
    double normalized_scan_y;
    double command_vx_v;
    double command_vy_v;
};

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
    std::string& error);

bool write_scan_frame_commands_csv(
    const std::string& path,
    const std::vector<ScanFrameCommand>& commands,
    std::string& error);
