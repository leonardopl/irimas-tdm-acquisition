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
    std::string& error,
    const std::function<bool()>& stop_requested = {});

// A scan table: row 0 is the centre (scan_command_index -1), row k has scan_command_index k - 1. An FSM schedule's table
// (`fsm_schedule`) has scan_command_index k in row k (the schedule's row).
// Validates the row contract and returns the CSV text; no filesystem I/O.
bool scan_frame_commands_csv(
    const std::vector<ScanFrameCommand>& commands,
    std::string& text,
    std::string& error,
    bool fsm_schedule = false);

double maximum_scan_command_step_volts(
    const std::vector<ScanFrameCommand>& commands);

// The step below which `fraction` of the consecutive steps lie (nearest rank); 0 without steps.
double scan_command_step_volts_quantile(
    const std::vector<ScanFrameCommand>& commands, double fraction);

int default_fsm_settle_ms(
    const std::vector<ScanFrameCommand>& commands);

// An FSM schedule (B199, the settle probe): FSM_SCHEDULE names a CSV file that replaces the scan pattern and NB_HOLO.
// UTF-8, the header `vx_v,vy_v,settle_ms,label`, then one row per frame in order: the mirror command in volts (finite,
// inside VXMIN..VXMAX and VYMIN..VYMAX), the settle after that command (integer ms, 0 to 1000) and a free label without
// commas (may be empty). Lines starting with '#' and blank lines are skipped. 1 to 4000 rows.
struct FsmScheduleRow {
    double vx_v;
    double vy_v;
    int settle_ms;
    std::string label;
};

const int kFsmScheduleMaxRows = 4000;
const int kFsmScheduleMaxSettleMs = 1000;

// The whole schedule checked against the run's limits (and NA_COND_LIM, which places the scan centre the mirror starts
// at). False with one line naming the file's line and the problem; then rows is empty.
bool parse_fsm_schedule(
    const std::string& text,
    double vxmin,
    double vxmax,
    double vymin,
    double vymax,
    double na_cond_lim,
    std::vector<FsmScheduleRow>& rows,
    std::string& error);

// Reads the file (at most 8 MB) into text, then parse_fsm_schedule.
bool read_fsm_schedule(
    const std::string& path,
    double vxmin,
    double vxmax,
    double vymin,
    double vymax,
    double na_cond_lim,
    std::string& text,
    std::vector<FsmScheduleRow>& rows,
    std::string& error);

// One command per schedule row for fsm_frame_commands.csv: image_index and scan_command_index k, the row's voltages, and
// the normalized coordinates a scan command at those voltages would have (the inverse of the scan's volt mapping; 0 on
// an axis whose limits are both 0 V). The rows must have passed parse_fsm_schedule with the same limits.
bool build_fsm_schedule_frame_commands(
    const std::vector<FsmScheduleRow>& rows,
    double vxmin,
    double vxmax,
    double vymin,
    double vymax,
    double na_cond_lim,
    std::vector<ScanFrameCommand>& commands,
    std::string& error);
