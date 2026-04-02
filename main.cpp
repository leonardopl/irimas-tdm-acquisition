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

void sigint_handler(int sig)
{
    cout << "\nSIGINT received, resetting DAC and shutting down..." << endl;
    if (g_DAC_ptr != NULL)
    {
        g_DAC_ptr->set_A_output(0.0);
        g_DAC_ptr->set_B_output(0.0);
    }
    if (g_camera_ptr)
        g_object_unref(g_camera_ptr);
    arv_shutdown();
    exit(1);
}

// Function Prototypes
ArvCamera* SelectAndConnectCamera();
void ConfigureCamera(ArvCamera* camera, int dimROI, int offsetROI_X, int offsetROI_Y,
                     const string& pixel_format_str);
ArvStream* StartAcquisition(ArvCamera* camera);
void StopAcquisition(ArvCamera* camera, ArvStream* stream);
void AcquireImages(ArvCamera* camera, Ljack_DAC *DAC_tiptilt,
                   string acquis_path, int dimROI, vector<float2D> &Vout_table,
                   int bytes_per_pixel, int cv_type);
void AcquireIntensityRef(ArvCamera* camera, Ljack_DAC *DAC_tiptilt,
                         string acquis_path, int dimROI, int offsetROI_X, int offsetROI_Y,
                         const string& pixel_format_str, int bytes_per_pixel, int cv_type);

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
    recon_path = config_dir + "/recon.txt";
    manip_config_path = config_dir + "/config_manip.txt";
    cout << "Config file: " << manip_config_path << endl;

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

    int NXMAX = get_val("NXMAX", manip_config_path, args);

    string acquis_dir = acquis_path;
    Var2D holo_dim = {2*NXMAX, 2*NXMAX};
    clear_acquisition(acquis_dir, manip_config_path, recon_path);

    // Extract configuration parameters
    MAX_IMAGES = get_val("NB_HOLO", manip_config_path, args);
    NAcondLim = get_val("NA_COND_LIM", manip_config_path, args);
    tiptilt_factor_x = (abs(get_val("VXMIN", manip_config_path, args)) +
                        abs(get_val("VXMAX", manip_config_path, args))) * NAcondLim / 20;
    tiptilt_factor_y = (abs(get_val("VYMIN", manip_config_path, args)) +
                        abs(get_val("VYMAX", manip_config_path, args))) * NAcondLim / 20;

    VfOffset.x = (get_val("VXMIN", manip_config_path, args) * NAcondLim + tiptilt_factor_x * 10);
    VfOffset.y = (get_val("VYMIN", manip_config_path, args) * NAcondLim + tiptilt_factor_y * 10);
    b_AmpliRef = get_val("AMPLI_REF", recon_path, args);

    cout << "  ####################################" << endl;
    cout << "  #  voltage MAX x=" << tiptilt_factor_x*10 << "                   #" << endl;
    cout << "  #  voltage MAX y=" << tiptilt_factor_y*10 << "                     #" << endl;
    cout << "  #  VfOffset.x=" << VfOffset.x << " | VfOffset.y=" << VfOffset.y << "  #" << endl;
    cout << "  #  Number of holograms=" << MAX_IMAGES << "        #" << endl;
    cout << "  ####################################" << endl;

    // Initialize LabJack DAC
    Ljack LU3HV;
    Ljack_DAC ljDAC_flower(LU3HV);
    ASSERT(ljDAC_flower.connect(FIO5_4));

    ljDAC_flower.set_A_output(VfOffset.x);
    ljDAC_flower.set_B_output(VfOffset.y);

    // Install SIGINT handler for clean DAC shutdown
    g_DAC_ptr = &ljDAC_flower;
    signal(SIGINT, sigint_handler);

    // Generate voltage lookup table for scan pattern
    string scan_pattern_str = get_string("SCAN_PATTERN", manip_config_path, args);
    int NbCirclesAnnular = get_val("NB_CIRCLES_ANNULAR", manip_config_path, args);
    cout << "Scan pattern = " << scan_pattern_str << endl;

    int const rho = 10; // max voltage for LabJack
    int const nbHolo = MAX_IMAGES;
    vector<float2D> Vout_table(nbHolo);
    bool scan_pattern_exist = 0;

    if(scan_pattern_str == "ROSACE") { scan_rose(rho, nbHolo, Vout_table); scan_pattern_exist = 1; }
    if(scan_pattern_str == "FERMAT") { scan_fermat(rho, nbHolo, Vout_table, holo_dim); scan_pattern_exist = 1; }
    if(scan_pattern_str == "ARCHIMEDE") { scan_spiralOS(rho, nbHolo, Vout_table, 4); scan_pattern_exist = 1; }
    if(scan_pattern_str == "ANNULAR") { scan_annular(rho, NbCirclesAnnular, nbHolo, Vout_table); scan_pattern_exist = 1; }
    if(scan_pattern_str == "UNIFORM3D") { scan_uniform3D(rho, nbHolo, Vout_table); scan_pattern_exist = 1; }
    if(scan_pattern_str == "RANDOM_POLAR") { scan_random_polar3D(rho, nbHolo, Vout_table, holo_dim); scan_pattern_exist = 1; }
    if(scan_pattern_str == "STAR") { scan_star(rho, nbHolo, 3, Vout_table); scan_pattern_exist = 1; }

    cout << "scan_pattern_exist=" << scan_pattern_exist << endl;
    if(scan_pattern_exist == 0)
        cout << "Cannot handle this scan pattern" << endl;

    ArvCamera* camera = NULL;

    try
    {
        cout << "GenICam Camera Acquisition:" << endl << endl;

        camera = SelectAndConnectCamera();

        if (camera)
        {
            g_camera_ptr = camera;

            ConfigureCamera(camera, dimROI, offsetROI_X, offsetROI_Y, pixel_format_str);

            // Acquire hologram images
            AcquireImages(camera, &ljDAC_flower, acquis_path, dimROI, Vout_table,
                          bytes_per_pixel, cv_type);

            // Acquire reference intensity if needed
            if(b_AmpliRef == 1)
                AcquireIntensityRef(camera, &ljDAC_flower, acquis_path, dimROI, offsetROI_X, offsetROI_Y,
                                    pixel_format_str, bytes_per_pixel, cv_type);

            cout << "Closing camera" << endl;
            g_object_unref(camera);
            g_camera_ptr = NULL;
        }
        else
        {
            cout << "No camera connected" << endl;
        }
    }
    catch (exception& e)
    {
        cout << "Exception: " << e.what() << endl;
        if (camera)
        {
            g_object_unref(camera);
            g_camera_ptr = NULL;
        }
        arv_shutdown();
        return -1;
    }

    arv_shutdown();

    // Reset DAC outputs
    ljDAC_flower.set_A_output(0.0);
    ljDAC_flower.set_B_output(0.0);

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

// Acquire hologram images
void AcquireImages(ArvCamera* camera, Ljack_DAC *DAC_tiptilt,
                   string acquis_path, int dimROI, vector<float2D> &Vout_table,
                   int bytes_per_pixel, int cv_type)
{
    cout << "Starting acquisition..." << endl;

    ArvStream* stream = StartAcquisition(camera);
    if (!stream)
        return;

    // Initialize DAC
    float2D Vscan = {0*tiptilt_factor_x, 0*tiptilt_factor_y};
    DAC_tiptilt->set_A_output(Vscan.x*tiptilt_factor_x + VfOffset.x);
    DAC_tiptilt->set_B_output(Vscan.y*tiptilt_factor_y + VfOffset.y);

    cout << endl << "##-- Hologram Acquisition --##" << endl;

    char lDoodle[] = "|\\-|-/";
    int lDoodleIndex = 0;
    size_t img_count = 0;

    // FPS calculation
    double t_last = (double)cv::getTickCount();
    double current_fps = 0.0;
    double bandwidth_MBps = 0.0;
    double alpha = 0.1; // smoothing factor

    vChronos vTime("image_capture");
    vTime.clear();
    vTime.start();

    while (img_count < MAX_IMAGES)
    {
        ArvBuffer* buffer = arv_stream_timeout_pop_buffer(stream, 5000000); // 5s in microseconds

        if (buffer == NULL)
        {
            cout << "Error: grab timeout" << endl;
            break;
        }

        if (arv_buffer_get_status(buffer) == ARV_BUFFER_STATUS_SUCCESS)
        {
            size_t data_size;
            const void* data = arv_buffer_get_image_data(buffer, &data_size);
            size_t lWidth = arv_buffer_get_image_width(buffer);
            size_t lHeight = arv_buffer_get_image_height(buffer);

            double t_now = (double)cv::getTickCount();
            double dt = (t_now - t_last) / cv::getTickFrequency();
            t_last = t_now;

            if (dt > 0)
            {
                double instant_fps = 1.0 / dt;
                if (current_fps == 0) current_fps = instant_fps;
                else current_fps = (1.0 - alpha) * current_fps + alpha * instant_fps;

                // Bandwidth in MB/s
                double frameBytes = (double)lWidth * (double)lHeight * bytes_per_pixel;
                bandwidth_MBps = (frameBytes * current_fps) / 1000000.0;
            }

            cv::Mat lframe(lHeight, lWidth, cv_type, const_cast<void*>(data));

            // imwrite before pushing buffer back (data pointer invalidated after push)
            imwrite(acquis_path + format("/i%03zu.pgm", img_count), lframe);

            // Update mirrors (DAC) for next image
            Vscan.x = Vout_table[img_count].x;
            Vscan.y = Vout_table[img_count].y;
            DAC_tiptilt->set_A_output(Vscan.x*tiptilt_factor_x + VfOffset.x);
            DAC_tiptilt->set_B_output(Vscan.y*tiptilt_factor_y + VfOffset.y);

            img_count++;

            cout << fixed << setprecision(1);
            cout << lDoodle[lDoodleIndex]
                 << " W: " << lWidth
                 << " H: " << lHeight
                 << " | " << current_fps << " FPS "
                 << bandwidth_MBps << " MB/s "
                 << "(" << img_count << "/" << MAX_IMAGES << ")\r";
            cout.flush();
        }
        else
        {
            cout << "Error: incomplete buffer received" << endl;
        }

        // Always push the buffer back to the stream
        arv_stream_push_buffer(stream, buffer);

        ++lDoodleIndex %= 6;
    }

    cout << endl;

    cout << "Stopping acquisition..." << endl;
    StopAcquisition(camera, stream);
}

// Acquire reference intensity image
void AcquireIntensityRef(ArvCamera* camera, Ljack_DAC *DAC_tiptilt,
                         string acquis_path, int dimROI, int offsetROI_X, int offsetROI_Y,
                         const string& pixel_format_str, int bytes_per_pixel, int cv_type)
{
    // Set reference voltages
    DAC_tiptilt->set_A_output(0);
    DAC_tiptilt->set_B_output(8);

    ConfigureCamera(camera, dimROI, offsetROI_X, offsetROI_Y, pixel_format_str);

    cout << endl << "##-- Reference Acquisition --##" << endl;

    ArvStream* stream = StartAcquisition(camera);
    if (!stream)
        return;

    // Wait for DAC to settle
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    ArvBuffer* buffer = arv_stream_timeout_pop_buffer(stream, 5000000);

    if (buffer != NULL && arv_buffer_get_status(buffer) == ARV_BUFFER_STATUS_SUCCESS)
    {
        DAC_tiptilt->set_A_output(9.5);
        DAC_tiptilt->set_B_output(9.5);

        size_t data_size;
        const void* data = arv_buffer_get_image_data(buffer, &data_size);
        size_t lWidth = arv_buffer_get_image_width(buffer);
        size_t lHeight = arv_buffer_get_image_height(buffer);

        cout << "  W: " << lWidth << " H: " << lHeight << endl;

        cv::Mat lframe(lHeight, lWidth, cv_type, const_cast<void*>(data));

        // imwrite before pushing buffer back
        imwrite(acquis_path + "/Intensite_ref.pgm", lframe);
        cout << "Reference intensity saved to " << acquis_path << endl;

        arv_stream_push_buffer(stream, buffer);
    }
    else
    {
        cout << "Failed to acquire reference image" << endl;
        if (buffer)
            arv_stream_push_buffer(stream, buffer);
    }

    StopAcquisition(camera, stream);
}
