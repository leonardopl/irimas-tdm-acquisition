// TDM holographic tomography acquisition - IRIMAS

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;
#include <string>
#include <sstream>

// Aravis
#include <arv.h>

#include "Ljack.h"
#include "Ljack_DAC.h"
#include "macros.h"
#include "vChronos.h"
#include "frame_timing.h"
#include "frame_loop.h"
#include <thread>
#include <chrono>

#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <termios.h>
#include <fcntl.h>
#include <signal.h>
#include <vector>
#include <set>

#include "functions.h"
#include "scan_functions.h"
#include "scan_schedule.h"

#include <list>
#include <iomanip>

float tiptilt_factor_x = 0;
float tiptilt_factor_y = 0;
float2D VfOffset = {0, 0};
unsigned int b_AmpliRef = 0;
float NAcondLim = 1; // condenser NA scanning limit coefficient, 0 < value <= 1
size_t MAX_IMAGES = 0;

// Global pointers for SIGINT cleanup
static Ljack_DAC* g_DAC_ptr = NULL;
static ArvCamera* g_camera_ptr = NULL;

void RestoreFreeRun(ArvCamera* camera);

// The LabJack library prints a failed DAC write and returns: its error counter (USB, I2C error code) and the I2C
// acknowledgements of each axis (a NACK is not counted) are the only signs.
static bool WriteFsm(Ljack_DAC* dac, float vx, float vy)
{
    const unsigned int errors = dac->communication_errors();
    dac->set_A_output(vx);
    const bool acknowledged_a = dac->last_output_acknowledged();
    dac->set_B_output(vy);
    return acknowledged_a && dac->last_output_acknowledged() && dac->communication_errors() == errors;
}

// A move the run depends on: one immediate retry, then the run stops before frames are taken.
static bool MoveFsm(Ljack_DAC* dac, float vx, float vy, const string& where)
{
    if (WriteFsm(dac, vx, vy) || WriteFsm(dac, vx, vy))
        return true;
    cerr << "FSM DAC write failed twice " << where << ": stopping; check the LabJack U3 USB connection" << endl;
    return false;
}

// The 0 V park at exit only warns.
static void ParkFsm(Ljack_DAC* dac)
{
    if (!WriteFsm(dac, 0.0, 0.0))
        cerr << "Warning: FSM DAC write failed: mirror not parked at 0 V" << endl;
}

void sigint_handler(int sig)
{
    cout << "\nSIGINT received, resetting DAC and shutting down..." << endl;
    if (g_DAC_ptr != NULL)
        ParkFsm(g_DAC_ptr);
    if (g_camera_ptr)
    {
        RestoreFreeRun(g_camera_ptr);
        g_object_unref(g_camera_ptr);
    }
    arv_shutdown();
    exit(1);
}

// Function Prototypes
ArvCamera* SelectAndConnectCamera();
void ConfigureCamera(ArvCamera* camera, int dimROI, int offsetROI_X, int offsetROI_Y,
                     const string& pixel_format_str);
ArvStream* StartAcquisition(ArvCamera* camera);
void StopAcquisition(ArvCamera* camera, ArvStream* stream);
bool ConfigureSoftwareTrigger(ArvCamera* camera);
bool AcquireImages(ArvCamera* camera, Ljack_DAC *DAC_tiptilt,
                   const string& acquis_path,
                   const vector<ScanFrameCommand>& commands,
                   FrameLoopOptions options, int bytes_per_pixel, int cv_type);
bool AcquireIntensityRef(ArvCamera* camera, Ljack_DAC *DAC_tiptilt,
                         const string& acquis_path, int dimROI,
                         int offsetROI_X, int offsetROI_Y,
                         const string& pixel_format_str, int settle_ms,
                         int bytes_per_pixel, int cv_type);

int main(int argc, char *argv[])
{
    cout << "########################## ACQUISITION ##################################" << endl;

    map<string, string> args = parse_args(argc, argv);

    // Initialize paths and configuration
    string gui_config_path, recon_path, manip_config_path, acquis_path, config_dir;
    string home = getenv("HOME");
    string gui_tomo_suffix = "/.config/gui_tomo.conf";
    gui_config_path = getenv("HOME") + gui_tomo_suffix;
    cout << "Reading paths from " << gui_config_path << endl;

    config_dir = get_string("CHEMIN_CONFIG", home + gui_tomo_suffix, args);
    acquis_path = get_string("CHEMIN_ACQUIS", home + gui_tomo_suffix, args);
    string config_error;
    if (!resolve_config_path(config_dir, "acquisition.conf", "config_manip.txt",
                             manip_config_path, config_error) ||
        !resolve_config_path(config_dir, "processing.conf", "recon.txt",
                             recon_path, config_error))
    {
        cerr << config_error << endl;
        return 1;
    }
    cout << "Acquisition config file: " << manip_config_path << endl;
    cout << "Processing config file: " << recon_path << endl;

    int dimROI = get_val("DIM_ROI", manip_config_path, args);
    int offsetROI_X = get_val("OFFSET_ROI_X", manip_config_path, args);
    int offsetROI_Y = get_val("OFFSET_ROI_Y", manip_config_path, args);
    string pixel_format_str = get_string("PIXEL_FORMAT", manip_config_path, args);
    if (pixel_format_str.empty() ||
        (pixel_format_str != "Mono8" && pixel_format_str != "Mono12" && pixel_format_str != "Mono16"))
    {
        cout << "PIXEL_FORMAT not set or invalid, defaulting to Mono8" << endl;
        pixel_format_str = "Mono8";
    }
    int bytes_per_pixel = (pixel_format_str == "Mono8") ? 1 : 2;
    int cv_type = (pixel_format_str == "Mono8") ? CV_8UC1 : CV_16UC1;

    // CONFIG_SCHEMA 3 has no NXMAX setting: derived from the optics, refused before anything moves when it cannot be.
    int NXMAX = 0;
    string nxmax_error;
    if (!resolve_nxmax(manip_config_path, args, NXMAX, nxmax_error))
    {
        cerr << nxmax_error << "; nothing moved" << endl;
        return 1;
    }

    // B199: an FSM schedule (the settle probe) replaces the scan pattern and NB_HOLO. The whole file is read and checked
    // here, before the acquis folder is cleared and before anything moves; a refused schedule stops the run.
    // Only from the command line (the GUI's one-shot probe field): a key left in the live acquisition.conf would turn every
    // acquisition into a probe without the GUI knowing (B199 review F1), and normal scans print no "key not found" line.
    const auto fsm_schedule_arg = args.find("FSM_SCHEDULE");
    const string fsm_schedule_path = fsm_schedule_arg == args.end() ? string() : fsm_schedule_arg->second;
    if (!fsm_schedule_path.empty()) cout << "FSM_SCHEDULE=" << fsm_schedule_path << " (from args)" << endl;
    const bool fsm_schedule = !fsm_schedule_path.empty();
    string fsm_schedule_text;
    vector<FsmScheduleRow> fsm_schedule_rows;
    if (fsm_schedule)
    {
        string error;
        if (!read_fsm_schedule(fsm_schedule_path,
                               get_val("VXMIN", manip_config_path, args), get_val("VXMAX", manip_config_path, args),
                               get_val("VYMIN", manip_config_path, args), get_val("VYMAX", manip_config_path, args),
                               get_val("NA_COND_LIM", manip_config_path, args),
                               fsm_schedule_text, fsm_schedule_rows, error))
        {
            cerr << "Invalid FSM schedule " << fsm_schedule_path << ": " << error << "; nothing moved" << endl;
            return 1;
        }
    }

    string acquis_dir = acquis_path;
    Var2D holo_dim = {2*NXMAX, 2*NXMAX};
    clear_acquisition(acquis_dir, manip_config_path, recon_path);

    // Extract configuration parameters
    MAX_IMAGES = get_val("NB_HOLO", manip_config_path, args);
    if (fsm_schedule)
    {
        cout << "FSM schedule " << fsm_schedule_path << ": " << fsm_schedule_rows.size()
             << " frames; it replaces NB_HOLO (" << MAX_IMAGES << ") and SCAN_PATTERN" << endl;
        MAX_IMAGES = fsm_schedule_rows.size();
    }
    string settle_value = get_string("FSM_SETTLE_MS", manip_config_path, args);
    int fsm_settle_ms = settle_value.empty() ? -1 : atoi(settle_value.c_str());
    if (!settle_value.empty() && (fsm_settle_ms < 0 || fsm_settle_ms > 10000))
    {
        cerr << "FSM_SETTLE_MS must be between 0 and 10000" << endl;
        return 1;
    }
    FrameLoopOptions frame_loop_options;
    string frame_loop_error;
    if (!parse_frame_loop_options(get_string("FSM_OVERLAP", manip_config_path, args),
                                  get_string("FSM_OVERLAP_MARGIN_US", manip_config_path, args),
                                  frame_loop_options, frame_loop_error))
    {
        cerr << frame_loop_error << endl;
        return 1;
    }
    NAcondLim = get_val("NA_COND_LIM", manip_config_path, args);
    tiptilt_factor_x = (abs(get_val("VXMIN", manip_config_path, args)) +
                        abs(get_val("VXMAX", manip_config_path, args))) * NAcondLim / 20;
    tiptilt_factor_y = (abs(get_val("VYMIN", manip_config_path, args)) +
                        abs(get_val("VYMAX", manip_config_path, args))) * NAcondLim / 20;

    VfOffset.x = (get_val("VXMIN", manip_config_path, args) * NAcondLim + tiptilt_factor_x * 10);
    VfOffset.y = (get_val("VYMIN", manip_config_path, args) * NAcondLim + tiptilt_factor_y * 10);
    b_AmpliRef = get_val("AMPLI_REF", ampli_ref_config_path(manip_config_path, recon_path), args);

    cout << "  ####################################" << endl;
    cout << "  #  voltage MAX x=" << tiptilt_factor_x*10 << "                   #" << endl;
    cout << "  #  voltage MAX y=" << tiptilt_factor_y*10 << "                     #" << endl;
    cout << "  #  VfOffset.x=" << VfOffset.x << " | VfOffset.y=" << VfOffset.y << "  #" << endl;
    cout << "  #  Number of holograms=" << MAX_IMAGES << "        #" << endl;
    cout << "  ####################################" << endl;

    // Initialize LabJack DAC
    Ljack LU3;
    Ljack_DAC ljDAC_flower(LU3);
    ASSERT(ljDAC_flower.connect(FIO5_4));

    if (!MoveFsm(&ljDAC_flower, VfOffset.x, VfOffset.y, "at the scan centre"))
    {
        ParkFsm(&ljDAC_flower);
        return 1;
    }

    // Install SIGINT handler for clean DAC shutdown
    g_DAC_ptr = &ljDAC_flower;
    signal(SIGINT, sigint_handler);

    // Generate one authoritative command per captured image.
    string scan_pattern_str = get_string("SCAN_PATTERN", manip_config_path, args);
    int NbCirclesAnnular = get_val("NB_CIRCLES_ANNULAR", manip_config_path, args);
    cout << "Scan pattern = " << (fsm_schedule ? string("SCHEDULE (FSM_SCHEDULE; SCAN_PATTERN not used)")
                                               : scan_pattern_str) << endl;
    vector<ScanFrameCommand> frame_commands;
    string schedule_error;
    string seed_value = get_string("SCAN_SEED", manip_config_path, args);
    const unsigned int random_seed = seed_value.empty()
        ? static_cast<unsigned int>(time(NULL))
        : static_cast<unsigned int>(strtoul(seed_value.c_str(), NULL, 10));
    const bool commands_built = fsm_schedule
        ? build_fsm_schedule_frame_commands(
              fsm_schedule_rows,
              get_val("VXMIN", manip_config_path, args),
              get_val("VXMAX", manip_config_path, args),
              get_val("VYMIN", manip_config_path, args),
              get_val("VYMAX", manip_config_path, args),
              NAcondLim, frame_commands, schedule_error)
        : build_scan_frame_commands(
              scan_pattern_str, static_cast<int>(MAX_IMAGES), holo_dim,
              NbCirclesAnnular,
              get_val("VXMIN", manip_config_path, args),
              get_val("VXMAX", manip_config_path, args),
              get_val("VYMIN", manip_config_path, args),
              get_val("VYMAX", manip_config_path, args),
              NAcondLim, random_seed, frame_commands, schedule_error);
    if (!commands_built)
    {
        cerr << "Invalid scan schedule: " << schedule_error << endl;
        ParkFsm(&ljDAC_flower);
        return 1;
    }
    const string command_table_path = acquis_path + "/fsm_frame_commands.csv";
    if (!write_scan_frame_commands_csv(command_table_path, frame_commands, schedule_error, fsm_schedule))
    {
        cerr << "Cannot persist scan schedule: " << schedule_error << endl;
        ParkFsm(&ljDAC_flower);
        return 1;
    }
    if (fsm_schedule)
    {
        // The schedule as read and checked, beside the frames.
        if (!write_fsm_schedule_copy(acquis_path + "/fsm_schedule.csv", fsm_schedule_text, schedule_error))
        {
            cerr << "Cannot save the FSM schedule copy: " << schedule_error << endl;
            ParkFsm(&ljDAC_flower);
            return 1;
        }
        int settle_min = fsm_schedule_rows[0].settle_ms, settle_max = settle_min;
        for (size_t k = 0; k < fsm_schedule_rows.size(); ++k)
        {
            frame_loop_options.frame_settle_ms.push_back(fsm_schedule_rows[k].settle_ms);
            settle_min = min(settle_min, fsm_schedule_rows[k].settle_ms);
            settle_max = max(settle_max, fsm_schedule_rows[k].settle_ms);
        }
        cout << "FSM settle per frame from the schedule: " << settle_min << "-" << settle_max << " ms"
             << (settle_value.empty() ? "" : " (FSM_SETTLE_MS not used)") << endl;
        if (frame_loop_options.overlap)
            cout << "FSM schedule: sequential frame loop, FSM_OVERLAP 1 not used (in overlap the trigger also waits for "
                    "the previous frame's transfer, so a short settle would not be the real wait)" << endl;
        if (b_AmpliRef == 1)
            cout << "AMPLI_REF 1: an FSM schedule run takes no intensity reference" << endl;
        cout << "Saved capture-aligned FSM schedule to " << command_table_path << " and "
             << acquis_path << "/fsm_schedule.csv" << endl;
        // Only the fallback of a frame without its own settle: none here.
        fsm_settle_ms = settle_max;
    }
    else
    {
        vector<ScanFrameCommand> settling_commands = frame_commands;
        if (b_AmpliRef == 1) {
            ScanFrameCommand reference_command = settling_commands.back();
            reference_command.command_vx_v = 0.0;
            reference_command.command_vy_v = 8.0;
            settling_commands.push_back(reference_command);
        }
        const double maximum_command_step_volts =
            maximum_scan_command_step_volts(settling_commands);
        if (settle_value.empty()) {
            fsm_settle_ms = default_fsm_settle_ms(settling_commands);
            cout << "FSM_SETTLE_MS not set; using " << fsm_settle_ms
                 << " ms for a maximum consecutive command step of "
                 << maximum_command_step_volts << " V" << endl;
        } else {
            cout << "FSM settle delay = " << fsm_settle_ms
                 << " ms (explicit), maximum consecutive command step = "
                 << maximum_command_step_volts << " V" << endl;
        }
        cout << "Saved capture-aligned FSM schedule to " << command_table_path << endl;
        cout << "Scan seed = " << random_seed << endl;
    }
    frame_loop_options.settle_ms = fsm_settle_ms;

    ArvCamera* camera = NULL;

    try
    {
        cout << "GenICam Camera Acquisition:" << endl << endl;

        camera = SelectAndConnectCamera();

        if (camera)
        {
            g_camera_ptr = camera;

            ConfigureCamera(camera, dimROI, offsetROI_X, offsetROI_Y, pixel_format_str);
            if (!ConfigureSoftwareTrigger(camera))
                throw runtime_error("camera does not support reliable software triggering");

            // Acquire hologram images
            if (!AcquireImages(camera, &ljDAC_flower, acquis_path, frame_commands,
                               frame_loop_options, bytes_per_pixel, cv_type))
                throw runtime_error("hologram acquisition did not complete");

            // Acquire reference intensity if needed (never in an FSM schedule run: only its own commands move the mirror)
            if(b_AmpliRef == 1 && !fsm_schedule)
                if (!AcquireIntensityRef(camera, &ljDAC_flower, acquis_path, dimROI,
                                         offsetROI_X, offsetROI_Y, pixel_format_str,
                                         fsm_settle_ms, bytes_per_pixel, cv_type))
                    throw runtime_error("reference acquisition did not complete");

            cout << "Closing camera" << endl;
            RestoreFreeRun(camera);
            g_object_unref(camera);
            g_camera_ptr = NULL;
        }
        else
        {
            cout << "No camera connected" << endl;
            throw runtime_error("no camera connected");
        }
    }
    catch (exception& e)
    {
        cout << "Exception: " << e.what() << endl;
        ParkFsm(&ljDAC_flower);
        if (camera)
        {
            RestoreFreeRun(camera);
            g_object_unref(camera);
            g_camera_ptr = NULL;
        }
        arv_shutdown();
        return -1;
    }

    arv_shutdown();

    // Reset DAC outputs
    ParkFsm(&ljDAC_flower);

    return 0;
}

// Select and connect to GenICam camera
ArvCamera* SelectAndConnectCamera()
{
    arv_update_device_list();
    unsigned int n_devices = arv_get_n_devices();

    if (n_devices == 0)
    {
        cout << "No cameras found!" << endl;
        return NULL;
    }

    // Build deduplicated device list (by serial + protocol)
    set<string> seen_keys;
    vector<unsigned int> unique_devices;
    for (unsigned int i = 0; i < n_devices; i++)
    {
        const char* serial = arv_get_device_serial_nbr(i);
        const char* protocol = arv_get_device_protocol(i);
        string key = string(serial ? serial : to_string(i)) + "|" + (protocol ? protocol : "");
        if (seen_keys.insert(key).second)
            unique_devices.push_back(i);
    }

    cout << "Found " << unique_devices.size() << " camera(s):" << endl;
    for (unsigned int i = 0; i < unique_devices.size(); i++)
    {
        unsigned int idx = unique_devices[i];
        const char* vendor = arv_get_device_vendor(idx);
        const char* model = arv_get_device_model(idx);
        const char* serial = arv_get_device_serial_nbr(idx);
        const char* protocol = arv_get_device_protocol(idx);
        cout << "  [" << i << "] "
             << (vendor ? vendor : "?") << " "
             << (model ? model : "?")
             << " (" << (serial ? serial : "?") << ")"
             << " [" << (protocol ? protocol : "?") << "]" << endl;
    }

    unsigned int selectedCamera = 0;
    if (unique_devices.size() > 1)
    {
        cout << "Select camera [0-" << unique_devices.size()-1 << "]: ";
        cin >> selectedCamera;
        if (selectedCamera >= unique_devices.size())
            selectedCamera = 0;
    }

    unsigned int deviceIdx = unique_devices[selectedCamera];
    cout << "Connecting to camera " << selectedCamera << "..." << endl;

    GError* error = NULL;
    ArvCamera* camera = arv_camera_new(arv_get_device_id(deviceIdx), &error);
    if (error)
    {
        cout << "Error connecting to camera: " << error->message << endl;
        g_error_free(error);
        return NULL;
    }

    const char* vendor = arv_get_device_vendor(deviceIdx);
    const char* model = arv_get_device_model(deviceIdx);
    cout << "Connected to: "
         << (vendor ? vendor : "?") << " "
         << (model ? model : "?") << endl;

    return camera;
}

// Configure camera ROI and settings
void ConfigureCamera(ArvCamera* camera, int dimROI, int offsetROI_X, int offsetROI_Y,
                     const string& pixel_format_str)
{
    GError* error = NULL;

    // Set ROI: offset and dimensions (skip if all values are -1)
    if (dimROI >= 0 || offsetROI_X >= 0 || offsetROI_Y >= 0)
    {
        arv_camera_set_region(camera, offsetROI_X, offsetROI_Y, dimROI, dimROI, &error);
        if (error)
        {
            cout << "Error setting ROI: " << error->message << endl;
            g_error_free(error);
            error = NULL;
        }
    }
    else
    {
        cout << "ROI config values are -1, keeping camera's current ROI" << endl;
    }

    // Set pixel format
    arv_camera_set_pixel_format_from_string(camera, pixel_format_str.c_str(), &error);
    if (error)
    {
        cout << "Error setting pixel format: " << error->message << endl;
        g_error_free(error);
        error = NULL;
    }

    // Readback
    gint x, y, w, h;
    arv_camera_get_region(camera, &x, &y, &w, &h, &error);
    if (!error)
    {
        cout << "Set Width to: " << w << endl;
        cout << "Set Height to: " << h << endl;
        cout << "Set OffsetX to: " << x << endl;
        cout << "Set OffsetY to: " << y << endl;
    }
    else
    {
        g_error_free(error);
        error = NULL;
    }

    const char* pf = arv_camera_get_pixel_format_as_string(camera, &error);
    if (!error && pf)
        cout << "Pixel Format: " << pf << endl;
    else if (error)
    {
        g_error_free(error);
        error = NULL;
    }
}

// Create stream, allocate buffers, and start acquisition. Returns stream or NULL on error.
ArvStream* StartAcquisition(ArvCamera* camera)
{
    GError* error = NULL;

    guint payload = arv_camera_get_payload(camera, &error);
    if (error)
    {
        cout << "Error getting payload: " << error->message << endl;
        g_error_free(error);
        return NULL;
    }

    ArvStream* stream = arv_camera_create_stream(camera, NULL, NULL, &error);
    if (error || !stream)
    {
        cout << "Error creating stream: " << (error ? error->message : "unknown") << endl;
        if (error) g_error_free(error);
        return NULL;
    }

    for (int i = 0; i < 10; i++)
        arv_stream_push_buffer(stream, arv_buffer_new(payload, NULL));

    arv_camera_start_acquisition(camera, &error);
    if (error)
    {
        cout << "Error starting acquisition: " << error->message << endl;
        g_error_free(error);
        g_object_unref(stream);
        return NULL;
    }

    return stream;
}

// Stop acquisition and release stream
void StopAcquisition(ArvCamera* camera, ArvStream* stream)
{
    GError* error = NULL;
    arv_camera_stop_acquisition(camera, &error);
    if (error)
    {
        cout << "Warning: error stopping acquisition: " << error->message << endl;
        g_error_free(error);
    }
    g_object_unref(stream);
}

bool ConfigureSoftwareTrigger(ArvCamera* camera)
{
    GError* error = NULL;
    arv_camera_set_trigger(camera, "Software", &error);
    if (error)
    {
        cerr << "Cannot configure software trigger: " << error->message << endl;
        g_error_free(error);
        return false;
    }
    return true;
}

// Leave the camera free-running for the next program (the GUI, pylon Viewer):
// the trigger mode set above persists in the camera until it is power-cycled.
void RestoreFreeRun(ArvCamera* camera)
{
    GError* error = NULL;
    arv_camera_clear_triggers(camera, &error);
    if (error)
    {
        cerr << "Warning: cannot switch the camera trigger off: " << error->message << endl;
        g_error_free(error);
    }
}

// Read the GUI's retained exposure; never change it or infer an exposure for overlap.
static bool ReadFixedExposure(ArvCamera* camera, double& exposure_us, bool overlap_requested,
                              string& fallback_reason)
{
    GError* error = NULL;
    fallback_reason.clear();
    auto unavailable = [&](const string& reason) {
        if (overlap_requested)
            fallback_reason = reason;
        cerr << frame_exposure_diagnostic(overlap_requested, reason) << endl;
        return false;
    };
    exposure_us = arv_camera_get_exposure_time(camera, &error);
    if (error || !std::isfinite(exposure_us) || exposure_us <= 0)
    {
        const string reason = string("cannot read camera exposure: ") +
                              (error ? error->message : "invalid ExposureTime");
        if (error) g_error_free(error);
        exposure_us = std::numeric_limits<double>::quiet_NaN();
        return unavailable(reason);
    }
    ArvDevice* device = arv_camera_get_device(camera);
    if (arv_device_get_feature(device, "ExposureAuto"))
    {
        const ArvAuto auto_mode = arv_camera_get_exposure_time_auto(camera, &error);
        if (error || auto_mode != ARV_AUTO_OFF)
        {
            const string reason = string("camera exposure auto is not known to be off: ") +
                                  (error ? error->message : "ExposureAuto enabled");
            if (error) g_error_free(error);
            return unavailable(reason);
        }
    }
    if (arv_device_get_feature(device, "SequencerMode"))
    {
        const char* mode = arv_device_get_string_feature_value(device, "SequencerMode", &error);
        if (error || !mode || string(mode) != "Off")
        {
            const string reason = string("camera sequencer is not known to be off: ") +
                                  (error ? error->message : (mode ? mode : "unreadable SequencerMode"));
            if (error) g_error_free(error);
            return unavailable(reason);
        }
    }
    if (arv_device_get_feature(device, "TriggerDelay"))
    {
        const double delay_us = arv_device_get_float_feature_value(device, "TriggerDelay", &error);
        if (error || !std::isfinite(delay_us) || delay_us != 0)
        {
            const string reason = string("camera TriggerDelay is not known to be zero: ") +
                                  (error ? error->message : frame_timing_number(delay_us, "unreadable"));
            if (error) g_error_free(error);
            return unavailable(reason);
        }
    }
    return true;
}

// The adapter keeps Aravis, LabJack and OpenCV out of the testable frame order.
struct AcquisitionFrameIO {
    typedef ArvBuffer* Buffer;
    ArvCamera* camera;
    ArvStream* stream;
    Ljack_DAC* dac;
    const string& acquis_path;
    const vector<ScanFrameCommand>& commands;
    FrameLoopOptions options;
    int bytes_per_pixel, cv_type;
    const std::chrono::steady_clock::time_point origin = std::chrono::steady_clock::now();
    bool latch_supported;
    FrameClockLatch first_latch;
    int doodle_index = 0;
    double t_last = (double)cv::getTickCount();
    double current_fps = 0, bandwidth_MBps = 0;

    AcquisitionFrameIO(ArvCamera* camera_, ArvStream* stream_, Ljack_DAC* dac_, const string& path,
                       const vector<ScanFrameCommand>& commands_, FrameLoopOptions options_, int bytes, int type)
        : camera(camera_), stream(stream_), dac(dac_), acquis_path(path), commands(commands_),
          options(options_), bytes_per_pixel(bytes), cv_type(type)
    {
        ArvDevice* device = arv_camera_get_device(camera);
        latch_supported = ARV_IS_GC_COMMAND(arv_device_get_feature(device, "TimestampLatch")) &&
                          ARV_IS_GC_INTEGER(arv_device_get_feature(device, "TimestampLatchValue"));
    }

    double now_ms() const {
        return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - origin).count();
    }
    void sleep_for_ms(int delay) {
        std::this_thread::sleep_for(std::chrono::milliseconds(delay));
    }
    void sleep_until_ms(double deadline) {
        // Round up to a clock tick so conversion cannot put the DAC before the deadline.
        const auto ticks = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double, std::milli>(deadline));
        std::this_thread::sleep_until(origin + ticks + std::chrono::steady_clock::duration(1));
    }
    FrameClockLatch latch() {
        FrameClockLatch result;
        if (!latch_supported)
            return result;
        GError* error = NULL;
        ArvDevice* device = arv_camera_get_device(camera);
        result.host_before_ms = now_ms();
        arv_device_execute_command(device, "TimestampLatch", &error);
        result.host_after_ms = now_ms();
        gint64 timestamp = 0;
        if (!error)
            timestamp = arv_device_get_integer_feature_value(device, "TimestampLatchValue", &error);
        if (error || timestamp < 0) {
            cerr << "Warning: camera clock latch failed: " << (error ? error->message : "negative timestamp") << endl;
            if (error) g_error_free(error);
            return result;
        }
        result.camera_ns = static_cast<std::uint64_t>(timestamp);
        result.valid = true;
        return result;
    }
    bool command(size_t k) {
        return WriteFsm(dac, commands[k].command_vx_v, commands[k].command_vy_v); // The loop repeats a failure.
    }
    bool trigger(size_t k) {
        GError* error = NULL;
        arv_camera_software_trigger(camera, &error);
        if (!error)
            return true;
        cerr << "Software trigger failed for i" << setw(3) << setfill('0')
             << k << setfill(' ') << ": " << error->message << endl;
        g_error_free(error);
        return false;
    }
    Buffer pop(size_t) {
        Buffer buffer = arv_stream_timeout_pop_buffer(stream, 5000000); // 5s in microseconds
        if (!buffer) cout << "Error: grab timeout" << endl;
        return buffer;
    }
    void describe(Buffer buffer, FrameTiming& row) {
        row.frame_id = arv_buffer_get_frame_id(buffer);
        row.camera_timestamp_ns = arv_buffer_get_timestamp(buffer);
        row.system_timestamp_ns = arv_buffer_get_system_timestamp(buffer);
    }
    bool complete(Buffer buffer) {
        if (arv_buffer_get_status(buffer) != ARV_BUFFER_STATUS_SUCCESS) {
            cout << "Error: incomplete buffer received" << endl;
            return false;
        }
        const double t_now = (double)cv::getTickCount();
        const double dt = (t_now - t_last) / cv::getTickFrequency();
        t_last = t_now;
        if (dt > 0) {
            const double instant_fps = 1.0 / dt;
            current_fps = current_fps == 0 ? instant_fps : 0.9 * current_fps + 0.1 * instant_fps;
            const double frame_bytes = (double)arv_buffer_get_image_width(buffer) *
                                       arv_buffer_get_image_height(buffer) * bytes_per_pixel;
            bandwidth_MBps = frame_bytes * current_fps / 1000000.0;
        }
        return true;
    }
    bool write(size_t k, Buffer buffer) {
        // The buffer stays held until imwrite returns, including when OpenCV throws.
        try {
            size_t data_size;
            const void* data = arv_buffer_get_image_data(buffer, &data_size);
            cv::Mat frame(arv_buffer_get_image_height(buffer), arv_buffer_get_image_width(buffer),
                          cv_type, const_cast<void*>(data));
            if (imwrite(acquis_path + format("/i%03zu.pgm", k), frame))
                return true;
        } catch (const std::exception& error) {
            cerr << "Image write error: " << error.what() << endl;
        }
        cerr << "Failed to write image i" << setw(3) << setfill('0') << k << setfill(' ') << endl;
        return false;
    }
    void progress(size_t k, Buffer buffer) {
        const char doodle[] = "|\\-|-/";
        cout << fixed << setprecision(1);
        cout << doodle[doodle_index]
             << " W: " << arv_buffer_get_image_width(buffer)
             << " H: " << arv_buffer_get_image_height(buffer)
             << " | " << current_fps << " FPS " << bandwidth_MBps << " MB/s "
             << "(" << k + 1 << "/" << commands.size() << ")\r";
        cout.flush();
        ++doodle_index %= 6;
    }
    void push(Buffer buffer) { arv_stream_push_buffer(stream, buffer); }
    void finish(vector<FrameTiming>& rows, bool success, const FrameLoopOptions& effective_options) {
        options = effective_options;
        const FrameClockMapping clock = frame_clock_mapping(first_latch, latch(), latch_supported);
        FrameTimingSummary summary = check_frame_timing(rows, options, clock, success);
        if (clock.failed) {
            cerr << "Warning: camera clock mapping unavailable; latch-based checks skipped" << endl;
            summary.suspect = true;
        }
        cout << endl << frame_timing_summary_line(summary) << endl;
        if (summary.suspect)
            cerr << "Warning: suspect frame timing; see frame_timing_summary.json" << endl;
        if (summary.timestamp_meaning_suspect)
            cerr << "Warning: camera timestamp meaning is not the exposure start; slack checks are not trustworthy" << endl;
        if (summary.latch_uncertain)
            cerr << "Warning: latch_uncertain; latch checks are inconclusive" << endl;
        string error;
        if (!write_frame_timing_csv(acquis_path + "/frame_timing.csv", rows, options.settle_ms, error))
            cerr << "Warning: cannot write the frame timing: " << error << endl;
        if (!write_frame_timing_summary(acquis_path + "/frame_timing_summary.json", summary, error))
            cerr << "Warning: cannot write the frame timing summary: " << error << endl;
        cout << "Stopping acquisition..." << endl;
        StopAcquisition(camera, stream);
    }
};

// Acquire hologram images
bool AcquireImages(ArvCamera* camera, Ljack_DAC *DAC_tiptilt,
                   const string& acquis_path,
                   const vector<ScanFrameCommand>& commands,
                   FrameLoopOptions options, int bytes_per_pixel, int cv_type)
{
    cout << "Starting acquisition..." << endl;
    double exposure_us;
    options.requested_overlap = options.overlap;
    const bool fixed_exposure = ReadFixedExposure(camera, exposure_us, options.requested_overlap, options.fallback_reason);
    if (!fixed_exposure)
        options.overlap = false;
    // An auto exposure's current value is printed but cannot support a slack check.
    options.exposure_us = fixed_exposure ? exposure_us : std::numeric_limits<double>::quiet_NaN();
    // An FSM schedule runs sequentially (run_frame_loop): say so here too.
    cout << "Frame loop mode=" << (options.overlap && options.frame_settle_ms.empty() ? "overlap" : "sequential")
         << " margin_us=" << options.margin_us << " exposure_us=" << frame_timing_number(exposure_us, "unavailable")
         << (fixed_exposure ? "" : " (not fixed)") << endl;

    ArvStream* stream = StartAcquisition(camera);
    if (!stream)
        return false;
    cout << endl << "##-- Hologram Acquisition --##" << endl;
    AcquisitionFrameIO io(camera, stream, DAC_tiptilt, acquis_path, commands, options, bytes_per_pixel, cv_type);
    io.first_latch = io.latch();
    vector<FrameTiming> timing;
    return run_frame_loop(commands.size(), options, io, timing);
}

// Acquire reference intensity image
bool AcquireIntensityRef(ArvCamera* camera, Ljack_DAC *DAC_tiptilt,
                         const string& acquis_path, int dimROI,
                         int offsetROI_X, int offsetROI_Y,
                         const string& pixel_format_str, int settle_ms,
                         int bytes_per_pixel, int cv_type)
{
    // Set reference voltages
    if (!MoveFsm(DAC_tiptilt, 0, 8, "for the intensity reference"))
        return false;

    ConfigureCamera(camera, dimROI, offsetROI_X, offsetROI_Y, pixel_format_str);

    cout << endl << "##-- Reference Acquisition --##" << endl;

    ArvStream* stream = StartAcquisition(camera);
    if (!stream)
        return false;

    // Wait for DAC to settle
    std::this_thread::sleep_for(std::chrono::milliseconds(settle_ms));

    GError* trigger_error = NULL;
    arv_camera_software_trigger(camera, &trigger_error);
    if (trigger_error)
    {
        cerr << "Software trigger failed for reference: " << trigger_error->message << endl;
        g_error_free(trigger_error);
        StopAcquisition(camera, stream);
        return false;
    }

    ArvBuffer* buffer = arv_stream_timeout_pop_buffer(stream, 5000000);
    bool saved = false;

    if (buffer != NULL && arv_buffer_get_status(buffer) == ARV_BUFFER_STATUS_SUCCESS)
    {
        // The reference frame is already taken at the right position: a failed move away only warns (the exit park
        // reports a dead U3 as well).
        if (!WriteFsm(DAC_tiptilt, 9.5, 9.5) && !WriteFsm(DAC_tiptilt, 9.5, 9.5))
            cerr << "Warning: FSM DAC write failed twice after the intensity reference; the reference frame is kept"
                 << endl;

        size_t data_size;
        const void* data = arv_buffer_get_image_data(buffer, &data_size);
        size_t lWidth = arv_buffer_get_image_width(buffer);
        size_t lHeight = arv_buffer_get_image_height(buffer);

        cout << "  W: " << lWidth << " H: " << lHeight << endl;

        cv::Mat lframe(lHeight, lWidth, cv_type, const_cast<void*>(data));

        // imwrite before pushing buffer back
        if (!imwrite(acquis_path + "/Intensite_ref.pgm", lframe))
        {
            cerr << "Failed to write reference intensity" << endl;
            arv_stream_push_buffer(stream, buffer);
            StopAcquisition(camera, stream);
            return false;
        }
        cout << "Reference intensity saved to " << acquis_path << endl;
        saved = true;

        arv_stream_push_buffer(stream, buffer);
    }
    else
    {
        cout << "Failed to acquire reference image" << endl;
        if (buffer)
            arv_stream_push_buffer(stream, buffer);
    }

    StopAcquisition(camera, stream);
    return saved;
}
