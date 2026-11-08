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
#include "camera_discovery.h"
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

#include "acquisition_protocol.h"
#include <atomic>
#include <memory>
#include <functional>
#include <sys/stat.h>
#include <poll.h>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <cerrno>
#include <cstring>
#include <cctype>

// A volatile pointer keeps the complete capability object in the linked binary.
static const char* volatile session_capability = acquisition_protocol::session_protocol_marker;
// Lock-free atomic operations are async-signal-safe and also synchronize with the stdin thread.
// Reason bits preserve the idle-EOF (0) versus signal (1) process exit convention.
static_assert(ATOMIC_INT_LOCK_FREE == 2, "signal stop flag must be lock-free");
static std::atomic<int> stop_requests(0);
static bool StopRequested() { return stop_requests.load(std::memory_order_relaxed) != 0; }
static bool SignalStopRequested() { return (stop_requests.load(std::memory_order_relaxed) & 1) != 0; }
static void stop_handler(int) { stop_requests.fetch_or(1, std::memory_order_relaxed); }
static void InputStopped() { stop_requests.fetch_or(2, std::memory_order_relaxed); }

#ifdef TDM_ACQUISITION_TEST
// Only the test fixture can inject hardware. No production command-line/environment simulation switch.
static Ljack* test_labjack = NULL;
static std::function<void(const string&, double, double)> test_hardware_event;
static std::function<void()> test_after_dac_open;
static std::function<void()> test_before_done;
#endif

void RestoreFreeRun(ArvCamera* camera);
// The LabJack library prints a failed DAC write and returns: its error counter (USB, I2C error code) and the I2C
// acknowledgements of each axis (a NACK is not counted) are the only signs.
static bool WriteFsm(Ljack_DAC* dac, float vx, float vy)
{
    const unsigned int errors = dac->communication_errors();
    dac->set_A_output(vx);
    const bool acknowledged_a = dac->last_output_acknowledged();
    dac->set_B_output(vy);
    const bool written = acknowledged_a && dac->last_output_acknowledged() && dac->communication_errors() == errors;
#ifdef TDM_ACQUISITION_TEST
    if (test_hardware_event) test_hardware_event("dac", vx, vy);
#endif
    return written;
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

// Function Prototypes
struct CameraIdentity { string identity, device_id, serial, model, vendor; };
ArvCamera* SelectAndConnectCamera(const string& wanted, bool session, CameraIdentity& identity);
void ConfigureCamera(ArvCamera* camera, int dimROI, int offsetROI_X, int offsetROI_Y,
                     const string& pixel_format_str);
ArvStream* StartAcquisition(ArvCamera* camera);
void StopAcquisition(ArvCamera* camera, ArvStream* stream);
bool ConfigureSoftwareTrigger(ArvCamera* camera);
bool AcquireImages(ArvCamera* camera, Ljack_DAC *DAC_tiptilt,
                   const string& acquis_path,
                   const vector<ScanFrameCommand>& commands,
                   FrameLoopOptions options, int bytes_per_pixel, int cv_type, size_t& frames);
bool AcquireIntensityRef(ArvCamera* camera, Ljack_DAC *DAC_tiptilt,
                         const string& acquis_path, int dimROI,
                         int offsetROI_X, int offsetROI_Y,
                         const string& pixel_format_str, int settle_ms,
                         int bytes_per_pixel, int cv_type);

// All run configuration and schedules are resolved before clearing files or commanding hardware.
// Both CLI and RUN use this function; no per-run data is retained by the connection.
struct RunConfig {
    string gui_config_path, recon_path, manip_config_path, acquis_path, config_dir;
    int dimROI = 0, offsetROI_X = 0, offsetROI_Y = 0, bytes_per_pixel = 1, cv_type = CV_8UC1, NXMAX = 0;
    string pixel_format_str, fsm_schedule_path, fsm_schedule_text;
    bool fsm_schedule = false;
    vector<FsmScheduleRow> fsm_schedule_rows;
    Var2D holo_dim;
    size_t MAX_IMAGES = 0;
    int fsm_settle_ms = 0;
    FrameLoopOptions frame_loop_options;
    float NAcondLim = 1, tiptilt_factor_x = 0, tiptilt_factor_y = 0;
    float2D VfOffset = {0, 0};
    unsigned int b_AmpliRef = 0;
    vector<ScanFrameCommand> frame_commands;

    // Session startup needs only paths and camera settings; RUN supplies its own scan overrides.
    bool resolveConnection(const map<string, string>& args, string& error) {
        // Initialize paths and configuration
        string home = getenv("HOME") ? getenv("HOME") : "";
        string gui_tomo_suffix = "/.config/gui_tomo.conf";
        gui_config_path = home + gui_tomo_suffix;
        cout << "Reading paths from " << gui_config_path << endl;

        config_dir = get_string("CHEMIN_CONFIG", home + gui_tomo_suffix, args);
        string config_error;
        if (!resolve_config_path(config_dir, "acquisition.conf", "config_manip.txt",
                                 manip_config_path, config_error) ||
            !resolve_config_path(config_dir, "processing.conf", "recon.txt",
                                 recon_path, config_error))
        {
            error = config_error;
            return false;
        }
        cout << "Acquisition config file: " << manip_config_path << endl;
        cout << "Processing config file: " << recon_path << endl;

        dimROI = get_val("DIM_ROI", manip_config_path, args);
        offsetROI_X = get_val("OFFSET_ROI_X", manip_config_path, args);
        offsetROI_Y = get_val("OFFSET_ROI_Y", manip_config_path, args);
        pixel_format_str = get_string("PIXEL_FORMAT", manip_config_path, args);
        if (pixel_format_str.empty() ||
            (pixel_format_str != "Mono8" && pixel_format_str != "Mono12" && pixel_format_str != "Mono16"))
        {
            cout << "PIXEL_FORMAT not set or invalid, defaulting to Mono8" << endl;
            pixel_format_str = "Mono8";
        }
        return true;
    }

    bool resolve(const map<string, string>& args, string& error) {
        if (!resolveConnection(args, error)) return false;
        acquis_path = get_string("CHEMIN_ACQUIS", gui_config_path, args);
        bytes_per_pixel = (pixel_format_str == "Mono8") ? 1 : 2;
        cv_type = (pixel_format_str == "Mono8") ? CV_8UC1 : CV_16UC1;

        // CONFIG_SCHEMA 3 has no NXMAX setting: derived from the optics, refused before anything moves when it cannot be.
        NXMAX = 0;
        string nxmax_error;
        if (!resolve_nxmax(manip_config_path, args, NXMAX, nxmax_error))
        {
            error = nxmax_error + "; nothing moved";
            return false;
        }

        // B199: an FSM schedule (the settle probe) replaces the scan pattern and NB_HOLO. The whole file is read and checked
        // here, before the acquis folder is cleared and before anything moves; a refused schedule stops the run.
        // Only from the command line (the GUI's one-shot probe field): a key left in the live acquisition.conf would turn every
        // acquisition into a probe without the GUI knowing (B199 review F1), and normal scans print no "key not found" line.
        const auto fsm_schedule_arg = args.find("FSM_SCHEDULE");
        fsm_schedule_path = fsm_schedule_arg == args.end() ? string() : fsm_schedule_arg->second;
        if (!fsm_schedule_path.empty()) cout << "FSM_SCHEDULE=" << fsm_schedule_path << " (from args)" << endl;
        fsm_schedule = !fsm_schedule_path.empty();
        if (fsm_schedule)
        {
            string schedule_read_error;
            if (!read_fsm_schedule(fsm_schedule_path,
                                   get_val("VXMIN", manip_config_path, args), get_val("VXMAX", manip_config_path, args),
                                   get_val("VYMIN", manip_config_path, args), get_val("VYMAX", manip_config_path, args),
                                   get_val("NA_COND_LIM", manip_config_path, args),
                                   fsm_schedule_text, fsm_schedule_rows, schedule_read_error))
            {
                schedule_read_error = "Invalid FSM schedule " + fsm_schedule_path + ": " + schedule_read_error + "; nothing moved";
                error = schedule_read_error;
                return false;
            }
        }

        holo_dim = {2*NXMAX, 2*NXMAX};

        // Extract configuration parameters
        MAX_IMAGES = get_val("NB_HOLO", manip_config_path, args);
        if (fsm_schedule)
        {
            cout << "FSM schedule " << fsm_schedule_path << ": " << fsm_schedule_rows.size()
                 << " frames; it replaces NB_HOLO (" << MAX_IMAGES << ") and SCAN_PATTERN" << endl;
            MAX_IMAGES = fsm_schedule_rows.size();
        }
        string settle_value = get_string("FSM_SETTLE_MS", manip_config_path, args);
        fsm_settle_ms = settle_value.empty() ? -1 : atoi(settle_value.c_str());
        if (!settle_value.empty() && (fsm_settle_ms < 0 || fsm_settle_ms > 10000))
        {
            error = "FSM_SETTLE_MS must be between 0 and 10000";
            return false;
        }
        string frame_loop_error;
        if (!parse_frame_loop_options(get_string("FSM_OVERLAP", manip_config_path, args),
                                      get_string("FSM_OVERLAP_MARGIN_US", manip_config_path, args),
                                      frame_loop_options, frame_loop_error))
        {
            error = frame_loop_error;
            return false;
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

        // Generate one authoritative command per captured image.
        string scan_pattern_str = get_string("SCAN_PATTERN", manip_config_path, args);
        int NbCirclesAnnular = get_val("NB_CIRCLES_ANNULAR", manip_config_path, args);
        cout << "Scan pattern = " << (fsm_schedule ? string("SCHEDULE (FSM_SCHEDULE; SCAN_PATTERN not used)")
                                                   : scan_pattern_str) << endl;
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
                  NAcondLim, random_seed, frame_commands, schedule_error, StopRequested);
        if (!commands_built)
        {
            error = "Invalid scan schedule: " + schedule_error;
            return false;
        }
        if (fsm_schedule)
        {
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
            cout << "Scan seed = " << random_seed << endl;
        }
        frame_loop_options.settle_ms = fsm_settle_ms;

        frame_loop_options.stop_requested = StopRequested;
        return true;
    }
};


struct DacConnectionError : runtime_error {
    DacConnectionError() : runtime_error("cannot connect LabJack DAC") {}
};

// Owns the connection only. Streams, clocks, buffers and schedules belong to each run.
struct Connection {
    unique_ptr<Ljack> host;
    unique_ptr<Ljack_DAC> dac;
    ArvCamera* camera = NULL;
    CameraIdentity identity;
    int width = 0, height = 0, x = 0, y = 0;
    string pixel_format;
    bool closed = false;
    bool dac_connected = false;

    void openDac() {
        Ljack* labjack = NULL;
#ifdef TDM_ACQUISITION_TEST
        labjack = test_labjack;
#endif
        if (!labjack) {
            host.reset(new Ljack());
            labjack = host.get();
        }
        dac.reset(new Ljack_DAC(*labjack));
        if (!dac->connect(FIO5_4)) {
            dac.reset(); // An unconnected DAC cannot accept a park command.
            throw DacConnectionError();
        }
        dac_connected = true;
#ifdef TDM_ACQUISITION_TEST
        if (test_after_dac_open) test_after_dac_open();
#endif
    }
    void openCamera(const RunConfig& c, const map<string, string>& args, bool session) {
        const auto id = args.find("CAMERA_ID");
        camera = SelectAndConnectCamera(id == args.end() ? string() : id->second, session, identity);
        if (!camera) {
            cout << "No camera connected" << endl;
            throw runtime_error("no camera connected");
        }
        ConfigureCamera(camera, c.dimROI, c.offsetROI_X, c.offsetROI_Y, c.pixel_format_str);
        if (!ConfigureSoftwareTrigger(camera))
            throw runtime_error("camera does not support reliable software triggering");
        if (session) {
            GError* error = NULL;
            arv_camera_get_region(camera, &x, &y, &width, &height, &error);
            if (error) {
                string message = error->message;
                g_error_free(error);
                throw runtime_error("cannot read camera ROI: " + message);
            }
            const char* format = arv_camera_get_pixel_format_as_string(camera, &error);
            pixel_format = format ? format : "";
            if (error) {
                string message = error->message;
                g_error_free(error);
                throw runtime_error("cannot read camera format: " + message);
            }
            if (!matches(c, true)) throw runtime_error("camera did not apply the requested ROI/format");
        }
    }
    bool matches(const RunConfig& c, bool retain_unset_roi = false) const {
        // Negative ROI values can retain the camera region at launch. RUN must resolve the
        // actual session geometry; accepting a different geometry would mislabel the saved data.
        return ((retain_unset_roi && c.dimROI < 0) || (width == c.dimROI && height == c.dimROI)) &&
               ((retain_unset_roi && c.offsetROI_X < 0) || x == c.offsetROI_X) &&
               ((retain_unset_roi && c.offsetROI_Y < 0) || y == c.offsetROI_Y) && pixel_format == c.pixel_format_str;
    }
    void close() {
        if (closed) return;
        closed = true;
        // Camera teardown can block on register timeouts; park before it can delay shutdown.
        if (dac && dac_connected) ParkFsm(dac.get());
        if (camera) {
            cout << "Closing camera" << endl;
            RestoreFreeRun(camera);
            g_object_unref(camera);
            camera = NULL;
        }
        arv_shutdown();
        dac.reset();
        host.reset();
    }
    ~Connection() { close(); }
};

struct RunResult {
    int status;
    size_t frames;
    string error;
    RunResult(int code = 0, size_t count = 0, const string& message = "")
        : status(code), frames(count), error(message) {}
};

static RunResult runOnce(const map<string, string>& args, Connection& connection, bool session)
{
    RunConfig c;
    string error;
    if (!c.resolve(args, error)) return RunResult(1, 0, error);
    if (session && !connection.matches(c))
        return RunResult(1, 0, "RUN ROI/format differs from the session connection; nothing cleared or moved");
    const auto wanted = args.find("CAMERA_ID");
    if (session && wanted != args.end() && !wanted->second.empty() &&
        wanted->second != connection.identity.identity && wanted->second != connection.identity.device_id)
        return RunResult(1, 0, "RUN CAMERA_ID differs from the session connection; nothing cleared or moved");
    if (StopRequested()) return RunResult(1, 0, "acquisition stopped");

    vector<string> left;
    const int clear_status = clear_acquisition(c.acquis_path, left, error);
    if (!left.empty()) {
        cerr << "Warning: kept entries in " << acquisition_protocol::diagnosticWord(c.acquis_path) << ": ";
        const size_t shown = min(left.size(), size_t(20));
        for (size_t i = 0; i < shown; ++i) {
            if (i) cerr << ", ";
            cerr << acquisition_protocol::diagnosticWord(left[i]);
        }
        if (shown < left.size()) cerr << ", ... and " << left.size() - shown << " more";
        cerr << endl;
    }
    if (clear_status != 0) return RunResult(1, 0, error);
    try {
        // Preserve the CLI's hardware order: clear, open DAC, centre, persist schedule, open camera.
        if (!connection.dac) connection.openDac();
        if (StopRequested()) return RunResult(1, 0, "acquisition stopped");
        if (!MoveFsm(connection.dac.get(), c.VfOffset.x, c.VfOffset.y, "at the scan centre"))
            return RunResult(1, 0, "FSM DAC write failed twice at the scan centre");
        const string table = c.acquis_path + "/fsm_frame_commands.csv";
        if (!write_scan_frame_commands_csv(table, c.frame_commands, error, c.fsm_schedule))
            return RunResult(1, 0, "Cannot persist scan schedule: " + error);
        if (c.fsm_schedule && !write_fsm_schedule_copy(c.acquis_path + "/fsm_schedule.csv", c.fsm_schedule_text, error))
            return RunResult(1, 0, "Cannot save the FSM schedule copy: " + error);
        cout << "Saved capture-aligned FSM schedule to " << table;
        if (c.fsm_schedule) cout << " and " << c.acquis_path << "/fsm_schedule.csv";
        cout << endl;
        if (!connection.camera) {
            cout << "GenICam Camera Acquisition:" << endl << endl;
            connection.openCamera(c, args, false);
        }
    } catch (const DacConnectionError& e) {
        return RunResult(1, 0, e.what()); // The previous DAC-connect assertion exited with 1.
    } catch (const exception& e) {
        return RunResult(StopRequested() ? 1 : -1, 0, StopRequested() ? "acquisition stopped" : e.what());
    }
    RunResult result;
    try {
        if (!AcquireImages(connection.camera, connection.dac.get(), c.acquis_path, c.frame_commands,
                           c.frame_loop_options, c.bytes_per_pixel, c.cv_type, result.frames))
            throw runtime_error("hologram acquisition did not complete");
        if (c.b_AmpliRef == 1 && !c.fsm_schedule)
            if (!AcquireIntensityRef(connection.camera, connection.dac.get(), c.acquis_path, c.dimROI,
                                     c.offsetROI_X, c.offsetROI_Y, c.pixel_format_str,
                                     c.fsm_settle_ms, c.bytes_per_pixel, c.cv_type))
                throw runtime_error("reference acquisition did not complete");
    } catch (const exception& e) {
        result.status = -1;
        result.error = e.what();
    }
    if (StopRequested()) {
        result.status = 1; // The original SIGINT handler exited with 1.
        result.error = "acquisition stopped";
    }
    return result;
}

// stdin is monitored throughout RUN. Commands are queued, but EOF requests immediate cooperative stop.
// poll also bounds thread teardown when the peer leaves stdin open after QUIT or a signal.
class SessionInput {
    atomic<bool> shutdown_{false};
    mutex mutex_;
    condition_variable changed_;
    deque<string> commands_;
    thread reader_;
    void readLines() {
        acquisition_protocol::LineReader framing;
        while (!shutdown_.load() && !SignalStopRequested()) {
            pollfd fd = {STDIN_FILENO, POLLIN, 0};
            const int ready = poll(&fd, 1, 50);
            if (ready < 0 && errno == EINTR) continue;
            if (ready == 0) continue;
            char bytes[4096];
            const ssize_t count = ready < 0 ? -1 : read(STDIN_FILENO, bytes, sizeof(bytes));
            if (count < 0 && (errno == EINTR || errno == EAGAIN)) continue;
            if (count <= 0) {
                string incomplete;
                if (!framing.finish(incomplete)) cerr << "Incomplete session line at EOF; stopping" << endl;
                InputStopped();
                changed_.notify_all();
                return;
            }
            const auto lines = framing.append(bytes, static_cast<size_t>(count));
            {
                lock_guard<mutex> lock(mutex_);
                commands_.insert(commands_.end(), lines.begin(), lines.end());
            }
            changed_.notify_all();
        }
    }
public:
    SessionInput() : reader_(&SessionInput::readLines, this) {}
    bool next(string& line) {
        unique_lock<mutex> lock(mutex_);
        while (commands_.empty() && !StopRequested()) changed_.wait_for(lock, chrono::milliseconds(50));
        if (StopRequested()) return false;
        line = commands_.front();
        commands_.pop_front();
        return true;
    }
    ~SessionInput() {
        shutdown_.store(true);
        if (reader_.joinable()) reader_.join();
    }
};

// Every reply uses the original stdout descriptor, while legacy diagnostics use redirected stdout/stderr.
class SessionReplies {
    FILE* output_ = NULL;
public:
    SessionReplies() {
        const int fd = dup(STDOUT_FILENO);
        if (fd < 0) throw runtime_error("cannot duplicate session stdout");
        output_ = fdopen(fd, "w");
        if (!output_) { close(fd); throw runtime_error("cannot open session reply stream"); }
        fflush(stdout);
        if (dup2(STDERR_FILENO, STDOUT_FILENO) < 0) {
            fclose(output_); output_ = NULL;
            throw runtime_error("cannot redirect session diagnostics");
        }
        fcntl(fd, F_SETFD, FD_CLOEXEC);
    }
    void send(const string& keyword, map<string, string> values = {}) {
#ifdef TDM_ACQUISITION_TEST
        if (keyword == "DONE" && test_before_done) test_before_done();
#endif
        // External error strings may contain line breaks. Keep every reply exactly one protocol line.
        for (auto& pair : values)
            for (char& c : pair.second)
                if (acquisition_protocol::detail::control(static_cast<unsigned char>(c))) c = ' ';
        const string line = acquisition_protocol::formatReply(keyword, values) + "\n";
        if (fwrite(line.data(), 1, line.size(), output_) != line.size() || fflush(output_) != 0)
            InputStopped(); // EPIPE is harmless: cleanup still runs normally.
    }
    ~SessionReplies() { if (output_) fclose(output_); }
};

// libLjack's constructors/calibration can call exit(1), rather than returning an error.
// During startup only, catch that normal-process exit with atexit: automatic objects have
// not been unwound, so the completed connection members and reply FILE are still alive.
// Never park a partially connected DAC. Exceptions and successful startup disarm the guard.
class SessionStartupGuard {
    struct State { Connection* connection = NULL; SessionReplies* replies = NULL; };
    static State& state() { static State value; return value; }
    static void driverExit() {
        State& active = state();
        if (!active.connection) return;
        Connection* connection = active.connection;
        SessionReplies* replies = active.replies;
        active = State(); // Prevent recursive cleanup if a driver exits again during close.
        replies->send("ERROR", {{"message", "hardware driver terminated during session startup"}});
        connection->close();
    }
public:
    SessionStartupGuard(Connection& connection, SessionReplies& replies) {
        static const bool registered = std::atexit(driverExit) == 0;
        if (!registered) throw runtime_error("cannot register session startup cleanup");
        state().connection = &connection;
        state().replies = &replies;
    }
    ~SessionStartupGuard() { state() = State(); }
};

static map<string, string> ExposureReply(ArvCamera* camera, const string& requested)
{
    string errors;
    GError* error = NULL;
    auto collect = [&]() {
        if (error) {
            if (!errors.empty()) errors += "; ";
            errors += error->message;
            g_error_free(error);
            error = NULL;
        }
    };
    char* end = NULL;
    const double us = strtod(requested.c_str(), &end);
    const bool valid = end != requested.c_str() && *end == '\0' && std::isfinite(us) && us > 0;
    if (!valid) errors = "EXPOSURE requires finite positive microseconds";
    const bool has_auto = arv_device_get_feature(arv_camera_get_device(camera), "ExposureAuto") != NULL;
    if (valid) {
        if (has_auto) { arv_camera_set_exposure_time_auto(camera, ARV_AUTO_OFF, &error); collect(); }
        arv_camera_set_exposure_time(camera, us, &error);
        collect();
    }
    // Always read back and always reply, including after either write failed.
    const double actual = arv_camera_get_exposure_time(camera, &error);
    const bool read_failed = error != NULL;
    collect();
    bool auto_on = false;
    if (has_auto) {
        auto_on = arv_camera_get_exposure_time_auto(camera, &error) != ARV_AUTO_OFF;
        if (error) auto_on = true; // Unknown auto state must not claim a fixed exposure.
        collect();
    }
    ostringstream value;
    value << setprecision(17) << (read_failed ? numeric_limits<double>::quiet_NaN() : actual);
    return {{"us", value.str()}, {"auto", auto_on ? "1" : "0"}, {"error", errors}};
}

static int RunSession(const map<string, string>& launch_args, SessionReplies& replies)
{
    SessionInput input;
    Connection connection;
    RunConfig initial;
    string error;
    if (!initial.resolveConnection(launch_args, error)) {
        replies.send("ERROR", {{"message", error}});
        return 1;
    }
    try {
        SessionStartupGuard startup(connection, replies);
        connection.openDac();
        connection.openCamera(initial, launch_args, true);
    } catch (const DacConnectionError& e) {
        replies.send("ERROR", {{"message", e.what()}});
        return 1;
    } catch (const exception& e) {
        replies.send("ERROR", {{"message", e.what()}});
        return SignalStopRequested() ? 1 : -1;
    }
    GError* exposure_error = NULL;
    const double exposure_us = arv_camera_get_exposure_time(connection.camera, &exposure_error);
    ostringstream exposure_value;
    exposure_value << setprecision(17) << (exposure_error ? numeric_limits<double>::quiet_NaN() : exposure_us);
    if (exposure_error) g_error_free(exposure_error);
    replies.send("HELLO", {{"protocol", "1"}, {"camera", connection.identity.identity},
                 {"serial", connection.identity.serial}, {"model", connection.identity.model},
                 {"vendor", connection.identity.vendor},
                 {"roi", to_string(connection.width) + "," + to_string(connection.x) + "," + to_string(connection.y)},
                 {"format", connection.pixel_format}, {"exposure_us", exposure_value.str()}});
    string line;
    while (input.next(line)) {
        vector<string> words;
        if (!acquisition_protocol::splitWords(line, words, error)) {
            replies.send("ERROR", {{"message", error}});
            continue;
        }
        if (words.empty()) continue;
        if (words[0] == "QUIT" && words.size() == 1) {
            connection.close();
            replies.send("BYE");
            return 0;
        }
        if (words[0] == "EXPOSURE" && words.size() == 2) {
            replies.send("EXPOSURE", ExposureReply(connection.camera, words[1]));
            continue;
        }
        if (words[0] == "RUN") {
            map<string, string> args;
            if (!acquisition_protocol::parseArguments(vector<string>(words.begin() + 1, words.end()),
                    acquisition_protocol::ArgumentOrigin::Run, args, error)) {
                replies.send("ERROR", {{"message", error}});
                continue;
            }
            const RunResult result = runOnce(args, connection, true);
            const bool stopped_during_run = StopRequested();
            replies.send("DONE", {{"status", to_string(acquisition_protocol::doneStatus(result.status))},
                         {"frames", to_string(result.frames)}, {"error", result.error}});
            if (stopped_during_run) return 1; // Later EOF belongs to the idle path.
            continue;
        }
        replies.send("ERROR", {{"message", "unknown command or invalid arguments: " + words[0]}});
    }
    return SignalStopRequested() ? 1 : 0; // EOF while idle is QUIT without BYE.
}

static void PrintUsage()
{
    cout << "Usage:\n"
         << "  tdm_acquisition [--KEY VALUE ...]   one acquisition\n"
         << "  tdm_acquisition --SESSION 1 [connection keys]   session mode; RUN lines on stdin\n"
         << "  tdm_acquisition --help | -h\n\n"
         << "Warning: one acquisition uses the camera, the FSM DAC and the config named by\n"
         << "~/.config/gui_tomo.conf, and writes into CHEMIN_ACQUIS (default from gui_tomo.conf).\n"
         << "Before a run, only regular files or symlinks named i + 3 or more ASCII digits + .pgm,\n"
         << "Intensite_ref.pgm, fsm_frame_commands.csv[.tmp], fsm_schedule.csv[.tmp],\n"
         << "frame_timing.csv and frame_timing_summary.json are deleted. Other entries are kept\n"
         << "and warned about. Archived i + 3 or more digits + .png/.jxl frames, an entry with\n"
         << "one of these names that is not a regular file or symlink (a directory, a FIFO), or\n"
         << "a removal failure refuse the run. Directories are never removed.\n\n"
         << "Use known --KEY VALUE pairs, once per key. Defaults come from acquisition.conf,\n"
         << "processing.conf and ~/.config/gui_tomo.conf. Scopes:\n"
         << "  launch only: session launch; connection: all launches and RUN; run: one-shot launch and RUN.\n";
    for (const auto& key : acquisition_protocol::argumentKeys()) {
        const char* scope = key.scope == acquisition_protocol::KeyScope::LaunchOnly ? "launch only" :
            (key.scope == acquisition_protocol::KeyScope::Connection ? "connection" : "run");
        cout << "  --" << key.name << " [" << scope << "] " << key.meaning << '\n';
    }
    cout << "\nExit codes: 0 ok / help; 1 refused or stopped; 2 usage error; 255 failure.\n";
}

int main(int argc, char* argv[])
{
    const vector<string> words = argc > 1 ? vector<string>(argv + 1, argv + argc) : vector<string>();
    if (acquisition_protocol::helpRequested(words)) { PrintUsage(); return 0; }
    map<string, string> args;
    string error;
    if (!acquisition_protocol::parseArguments(words, acquisition_protocol::ArgumentOrigin::Launch, args, error)) {
        cerr << "tdm_acquisition: " << error << " (--help lists the keys)" << endl;
        // A session client reads only stdout replies: a startup failure is an ERROR reply there too.
        if (acquisition_protocol::sessionRequested(words))
            cout << acquisition_protocol::formatReply("ERROR", {{"message", error}}) << '\n' << flush;
        return 2;
    }
    (void)session_capability;
    stop_requests.store(0, std::memory_order_relaxed);
    struct sigaction action = {};
    action.sa_handler = stop_handler;
    sigemptyset(&action.sa_mask);
    sigaction(SIGINT, &action, NULL);
    sigaction(SIGTERM, &action, NULL);
    const auto session = args.find("SESSION");
    if (session != args.end() && session->second == "1") {
        signal(SIGPIPE, SIG_IGN);
        try {
            SessionReplies replies;
            cout << "########################## ACQUISITION ##################################" << endl;
            try { return RunSession(args, replies); }
            catch (const exception& e) { replies.send("ERROR", {{"message", e.what()}}); return SignalStopRequested() ? 1 : -1; }
        } catch (const exception& e) { cerr << e.what() << endl; return 1; }
    }
    cout << "########################## ACQUISITION ##################################" << endl;
    Connection connection;
    const RunResult result = runOnce(args, connection, false);
    if (!result.error.empty()) {
        if (result.status == -1) cout << "Exception: " << result.error << endl;
        else cerr << result.error << endl;
    }
    return result.status;
}

// Select and connect to GenICam camera
static unsigned int PromptCameraSelection(size_t camera_count)
{
    if (StopRequested()) throw runtime_error("acquisition stopped");
    cout << "Select camera [0-" << camera_count - 1 << "]: " << flush;
    string word;
    while (!StopRequested()) {
        pollfd fd = {STDIN_FILENO, POLLIN, 0};
        const int ready = poll(&fd, 1, 50);
        if (ready < 0 && errno == EINTR) continue;
        if (ready == 0) continue;
        if (ready < 0) throw runtime_error("cannot read camera selection");
        // Read one byte at a time: formatted cin extraction can block again after
        // poll reports only whitespace or a partial selection from a pipe.
        char c;
        const ssize_t count = read(STDIN_FILENO, &c, 1);
        if (count < 0 && (errno == EINTR || errno == EAGAIN)) continue;
        if (count < 0) throw runtime_error("cannot read camera selection");
        if (count == 0) break;
        if (isspace(static_cast<unsigned char>(c))) {
            if (!word.empty()) break;
        } else word += c;
    }
    if (StopRequested()) throw runtime_error("acquisition stopped");
    unsigned int selected = 0;
    istringstream(word) >> selected;
    return selected < camera_count ? selected : 0;
}

ArvCamera* SelectAndConnectCamera(const string& wanted, bool session, CameraIdentity& identity)
{
    vector<unsigned int> unique_devices;
    vector<camera_discovery::Device> devices;
    bool wanted_found = wanted.empty();
    const unsigned int n_cameras = DiscoverCameras([&](bool network) -> unsigned int {
        if (network)
        {
            cout << (wanted.empty() ? "No USB3 Vision camera" : "Requested camera not found on USB3 Vision")
                 << ": looking for GigE Vision cameras too" << endl;
#ifdef TDM_ACQUISITION_TEST
            if (!test_labjack) // Tests isolate Aravis to Fake before entering the program.
#endif
            arv_enable_interface("GigEVision");
        }
        else
            arv_disable_interface("GigEVision");
        arv_update_device_list();

        devices.clear();
        const unsigned int n_devices = arv_get_n_devices();
        for (unsigned int i = 0; i < n_devices; i++)
        {
            const char* serial = arv_get_device_serial_nbr(i);
            const char* id = arv_get_device_id(i);
            const char* protocol = arv_get_device_protocol(i);
            devices.push_back({id ? id : "", serial ? serial : "", protocol ? protocol : ""});
        }
        unique_devices = camera_discovery::deduplicateDevices(devices, wanted);
        unsigned int selected;
        string error;
        wanted_found = wanted.empty() || camera_discovery::selectDevice(devices, unique_devices, wanted, selected, error);
        return static_cast<unsigned int>(unique_devices.size());
    }, [&]() { return wanted_found; });

    unsigned int selectedCamera = 0;
    if (!wanted.empty()) {
        string error;
        if (!camera_discovery::selectDevice(devices, unique_devices, wanted, selectedCamera, error))
            throw runtime_error(error);
    }
    if (n_cameras == 0)
    {
        cout << "No cameras found!" << endl;
        return NULL;
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

    if (wanted.empty() && unique_devices.size() > 1)
    {
        if (session) throw runtime_error("CAMERA_ID is required when several cameras are found");
        selectedCamera = PromptCameraSelection(unique_devices.size());
    }

    unsigned int deviceIdx = unique_devices[selectedCamera];
    const char* serial = arv_get_device_serial_nbr(deviceIdx);
    const char* id = arv_get_device_id(deviceIdx);
    const char* vendor_name = arv_get_device_vendor(deviceIdx);
    const char* model_name = arv_get_device_model(deviceIdx);
    identity.device_id = id ? id : "";
    identity.serial = serial ? serial : "";
    identity.identity = identity.serial.empty() ? identity.device_id : identity.serial;
    identity.vendor = vendor_name ? vendor_name : "";
    identity.model = model_name ? model_name : "";
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
#ifdef TDM_ACQUISITION_TEST
    if (test_hardware_event) {
        GError* check = NULL;
        const char* mode = arv_device_get_string_feature_value(arv_camera_get_device(camera), "TriggerMode", &check);
        test_hardware_event("free_run", !check && mode && string(mode) == "Off", 0);
        if (check) g_error_free(check);
    }
#endif
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
    size_t frames = 0;
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
            if (imwrite(acquis_path + format("/i%03zu.pgm", k), frame)) {
                ++frames;
                return true;
            }
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
        // AcquireImages owns the stream: its scope guard stops/releases it even if timing output throws.
    }
};

// Acquire hologram images
bool AcquireImages(ArvCamera* camera, Ljack_DAC *DAC_tiptilt,
                   const string& acquis_path,
                   const vector<ScanFrameCommand>& commands,
                   FrameLoopOptions options, int bytes_per_pixel, int cv_type, size_t& frames)
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
    unique_ptr<ArvStream, function<void(ArvStream*)>> stream_owner(stream,
        [camera](ArvStream* s) { StopAcquisition(camera, s); });
    cout << endl << "##-- Hologram Acquisition --##" << endl;
    AcquisitionFrameIO io(camera, stream, DAC_tiptilt, acquis_path, commands, options, bytes_per_pixel, cv_type);
    io.first_latch = io.latch();
    vector<FrameTiming> timing;
    const bool complete = run_frame_loop(commands.size(), options, io, timing);
    frames = io.frames;
    return complete;
}

// Acquire reference intensity image; all exits return its buffer and stop its private stream.
bool AcquireIntensityRef(ArvCamera* camera, Ljack_DAC *DAC_tiptilt,
                         const string& acquis_path, int dimROI,
                         int offsetROI_X, int offsetROI_Y,
                         const string& pixel_format_str, int settle_ms,
                         int bytes_per_pixel, int cv_type)
{
    if (StopRequested()) return false;
    if (!MoveFsm(DAC_tiptilt, 0, 8, "for the intensity reference")) return false;
    if (StopRequested()) return false;
    ConfigureCamera(camera, dimROI, offsetROI_X, offsetROI_Y, pixel_format_str);
    if (StopRequested()) return false;
    cout << endl << "##-- Reference Acquisition --##" << endl;
    ArvStream* stream = StartAcquisition(camera);
    if (!stream) return false;
    unique_ptr<ArvStream, function<void(ArvStream*)>> stream_owner(stream,
        [camera](ArvStream* s) { StopAcquisition(camera, s); });
    if (StopRequested()) return false;
    this_thread::sleep_for(chrono::milliseconds(settle_ms));
    if (StopRequested()) return false;
    GError* error = NULL;
    arv_camera_software_trigger(camera, &error);
    if (error) {
        cerr << "Software trigger failed for reference: " << error->message << endl;
        g_error_free(error);
        return false;
    }
    if (StopRequested()) return false;
    ArvBuffer* buffer = arv_stream_timeout_pop_buffer(stream, 5000000);
    if (!buffer) { cout << "Failed to acquire reference image" << endl; return false; }
    unique_ptr<ArvBuffer, function<void(ArvBuffer*)>> buffer_owner(buffer,
        [stream](ArvBuffer* b) { arv_stream_push_buffer(stream, b); });
    if (StopRequested()) return false;
    if (arv_buffer_get_status(buffer) != ARV_BUFFER_STATUS_SUCCESS) {
        cout << "Failed to acquire reference image" << endl;
        return false;
    }
    // The reference is already captured: a failed move away only warns.
    if (!WriteFsm(DAC_tiptilt, 9.5, 9.5) && !WriteFsm(DAC_tiptilt, 9.5, 9.5))
        cerr << "Warning: FSM DAC write failed twice after the intensity reference; the reference frame is kept" << endl;
    if (StopRequested()) return false;
    size_t size;
    const void* data = arv_buffer_get_image_data(buffer, &size);
    const int width = arv_buffer_get_image_width(buffer), height = arv_buffer_get_image_height(buffer);
    cout << "  W: " << width << " H: " << height << endl;
    cv::Mat frame(height, width, cv_type, const_cast<void*>(data));
    if (!imwrite(acquis_path + "/Intensite_ref.pgm", frame)) {
        cerr << "Failed to write reference intensity" << endl;
        return false;
    }
    cout << "Reference intensity saved to " << acquis_path << endl;
    return !StopRequested();
}
