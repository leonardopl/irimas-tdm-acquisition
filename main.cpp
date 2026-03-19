// TDM holographic tomography acquisition - IRIMAS

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;
#include <string>
#include <sstream>

// rc_genicam_api
#include <rc_genicam_api/system.h>
#include <rc_genicam_api/interface.h>
#include <rc_genicam_api/device.h>
#include <rc_genicam_api/stream.h>
#include <rc_genicam_api/buffer.h>
#include <rc_genicam_api/config.h>
using namespace GenApi;

#include "Ljack.h"
#include "Ljack_DAC.h"
#include "macros.h"
#include "vChronos.h"
#include <boost/thread.hpp>
#include <boost/date_time.hpp>

#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <termios.h>
#include <fcntl.h>
#include <signal.h>

#include "functions.h"
#include "scan_functions.h"

#include <list>

float tiptilt_factor_x = 0;
float tiptilt_factor_y = 0;
float2D VfOffset = {0, 0};
unsigned int b_AmpliRef = 0;
float NAcondLim = 1; // condenser NA scanning limit coefficient, 0 < value <= 1
size_t MAX_IMAGES = 0;

// Global pointer for SIGINT cleanup
static Ljack_DAC* g_DAC_ptr = NULL;

void sigint_handler(int sig)
{
    cout << "\nSIGINT received, resetting DAC and shutting down..." << endl;
    if (g_DAC_ptr != NULL)
    {
        g_DAC_ptr->set_A_output(0.0);
        g_DAC_ptr->set_B_output(0.0);
    }
    rcg::System::clearSystems();
    exit(1);
}

// Function Prototypes
std::shared_ptr<rcg::Device> SelectAndConnectCamera();
void ConfigureCamera(std::shared_ptr<rcg::Device> device, int dimROI);
void AcquireImages(std::shared_ptr<rcg::Device> device, std::shared_ptr<rcg::Stream> stream,
                   Ljack_DAC *DAC_tiptilt,
                   string acquis_path, int dimROI, vector<float2D> &Vout_table);
void AcquireIntensityRef(std::shared_ptr<rcg::Device> device, std::shared_ptr<rcg::Stream> stream,
                         Ljack_DAC *DAC_tiptilt,
                         string acquis_path, int dimROI);

int main(int argc, char *argv[])
{
    cout << "########################## ACQUISITION ##################################" << endl;

    // Initialize paths and configuration
    string gui_config_path, recon_path, manip_config_path, acquis_path, config_dir;
    string home = getenv("HOME");
    string gui_tomo_suffix = "/.config/gui_tomo.conf";
    gui_config_path = getenv("HOME") + gui_tomo_suffix;
    cout << "Reading paths from " << gui_config_path << endl;

    config_dir = extract_string("CHEMIN_CONFIG", home + gui_tomo_suffix);
    acquis_path = extract_string("CHEMIN_ACQUIS", home + gui_tomo_suffix);
    recon_path = config_dir + "/recon.txt";
    manip_config_path = config_dir + "/config_manip.txt";
    cout << "Config file: " << manip_config_path << endl;

    int dimROI = extract_val("DIM_ROI", manip_config_path);
    int NXMAX = extract_val("NXMAX", manip_config_path);

    string acquis_dir = acquis_path;
    Var2D holo_dim = {2*NXMAX, 2*NXMAX};
    clear_acquisition(acquis_dir, manip_config_path, recon_path);

    // Extract configuration parameters
    MAX_IMAGES = extract_val("NB_HOLO", manip_config_path);
    NAcondLim = extract_val("NA_COND_LIM", manip_config_path);
    tiptilt_factor_x = (abs(extract_val("VXMIN", manip_config_path)) +
                        abs(extract_val("VXMAX", manip_config_path))) * NAcondLim / 20;
    tiptilt_factor_y = (abs(extract_val("VYMIN", manip_config_path)) +
                        abs(extract_val("VYMAX", manip_config_path))) * NAcondLim / 20;

    VfOffset.x = (extract_val("VXMIN", manip_config_path) * NAcondLim + tiptilt_factor_x * 10);
    VfOffset.y = (extract_val("VYMIN", manip_config_path) * NAcondLim + tiptilt_factor_y * 10) * NAcondLim;
    b_AmpliRef = extract_val("AMPLI_REF", recon_path);

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
    string scan_pattern_str = extract_string("SCAN_PATTERN", manip_config_path);
    int NbCirclesAnnular = extract_val("NB_CIRCLES_ANNULAR", manip_config_path);
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

    std::shared_ptr<rcg::Device> device;
    std::shared_ptr<rcg::Stream> stream;

    try
    {
        cout << "GenICam Camera Acquisition:" << endl << endl;

        device = SelectAndConnectCamera();

        if (device)
        {
            // Open the first available stream
            std::vector<std::shared_ptr<rcg::Stream>> streams = device->getStreams();
            if (streams.empty())
            {
                cout << "No streams available on device" << endl;
                device->close();
                rcg::System::clearSystems();
                return -1;
            }
            stream = streams[0];
            stream->open();

            ConfigureCamera(device, dimROI);

            // Acquire hologram images
            AcquireImages(device, stream, &ljDAC_flower, acquis_path, dimROI, Vout_table);

            // Acquire reference intensity if needed
            if(b_AmpliRef == 1)
                AcquireIntensityRef(device, stream, &ljDAC_flower, acquis_path, dimROI);

            cout << "Closing camera" << endl;
            stream->close();
            device->close();
        }
        else
        {
            cout << "No camera connected" << endl;
        }
    }
    catch (exception& e)
    {
        cout << "Exception: " << e.what() << endl;
        if (stream) stream->close();
        if (device) device->close();
        rcg::System::clearSystems();
        return -1;
    }

    rcg::System::clearSystems();

    // Reset DAC outputs
    ljDAC_flower.set_A_output(0.0);
    ljDAC_flower.set_B_output(0.0);

    return 0;
}

// Select and connect to GenICam camera
std::shared_ptr<rcg::Device> SelectAndConnectCamera()
{
    try
    {
        // Enumerate all devices across all systems and interfaces
        std::vector<std::shared_ptr<rcg::Device>> allDevices;

        std::vector<std::shared_ptr<rcg::System>> systems = rcg::System::getSystems();
        for (auto& system : systems)
        {
            system->open();
            std::vector<std::shared_ptr<rcg::Interface>> interfaces = system->getInterfaces();
            for (auto& iface : interfaces)
            {
                iface->open();
                std::vector<std::shared_ptr<rcg::Device>> devices = iface->getDevices();
                for (auto& dev : devices)
                {
                    allDevices.push_back(dev);
                }
                iface->close();
            }
        }

        if (allDevices.empty())
        {
            cout << "No cameras found!" << endl;
            return nullptr;
        }

        cout << "Found " << allDevices.size() << " camera(s):" << endl;
        for (size_t i = 0; i < allDevices.size(); i++)
        {
            cout << "  [" << i << "] " << allDevices[i]->getVendor()
                 << " " << allDevices[i]->getModel()
                 << " (" << allDevices[i]->getSerialNumber() << ")" << endl;
        }

        int selectedCamera = 0;
        if (allDevices.size() > 1)
        {
            cout << "Select camera [0-" << allDevices.size()-1 << "]: ";
            cin >> selectedCamera;
            if (selectedCamera < 0 || selectedCamera >= (int)allDevices.size())
                selectedCamera = 0;
        }

        cout << "Connecting to camera " << selectedCamera << "..." << endl;

        std::shared_ptr<rcg::Device> device = allDevices[selectedCamera];
        device->open(rcg::Device::CONTROL);

        cout << "Connected to: " << device->getVendor() << " " << device->getModel() << endl;

        return device;
    }
    catch (exception& e)
    {
        cout << "Error selecting camera: " << e.what() << endl;
        return nullptr;
    }
}

// Configure camera ROI and settings
void ConfigureCamera(std::shared_ptr<rcg::Device> device, int dimROI)
{
    try
    {
        auto nodemap = device->getRemoteNodeMap();

        rcg::setInteger(nodemap, "Width", dimROI, true);
        cout << "Set Width to: " << rcg::getInteger(nodemap, "Width") << endl;

        rcg::setInteger(nodemap, "Height", dimROI, true);
        cout << "Set Height to: " << rcg::getInteger(nodemap, "Height") << endl;

        rcg::setInteger(nodemap, "OffsetX", dimROI / 2, true);
        cout << "Set OffsetX to: " << rcg::getInteger(nodemap, "OffsetX") << endl;

        rcg::setInteger(nodemap, "OffsetY", dimROI / 2, true);
        cout << "Set OffsetY to: " << rcg::getInteger(nodemap, "OffsetY") << endl;

        rcg::setEnum(nodemap, "PixelFormat", "Mono8", true);
        cout << "Pixel Format: " << rcg::getEnum(nodemap, "PixelFormat") << endl;
    }
    catch (exception& e)
    {
        cout << "Error configuring camera: " << e.what() << endl;
        throw;
    }
}

// Acquire hologram images
void AcquireImages(std::shared_ptr<rcg::Device> device, std::shared_ptr<rcg::Stream> stream,
                   Ljack_DAC *DAC_tiptilt,
                   string acquis_path, int dimROI, vector<float2D> &Vout_table)
{
    try
    {
        auto nodemap = device->getRemoteNodeMap();

        cout << "Starting acquisition..." << endl;

        stream->startStreaming();

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
            try
            {
                const rcg::Buffer* buffer = stream->grab(5000);

                if (buffer == nullptr)
                {
                    cout << "Error: grab timeout" << endl;
                    break;
                }

                if (!buffer->getIsIncomplete())
                {
                    size_t lWidth = buffer->getWidth(0);
                    size_t lHeight = buffer->getHeight(0);

                    double t_now = (double)cv::getTickCount();
                    double dt = (t_now - t_last) / cv::getTickFrequency();
                    t_last = t_now;

                    if (dt > 0)
                    {
                        double instant_fps = 1.0 / dt;
                        if (current_fps == 0) current_fps = instant_fps;
                        else current_fps = (1.0 - alpha) * current_fps + alpha * instant_fps;

                        // Bandwidth in MB/s (1 pixel = 1 byte for Mono8)
                        double frameBytes = (double)lWidth * (double)lHeight;
                        bandwidth_MBps = (frameBytes * current_fps) / 1000000.0;
                    }

                    cv::Mat lframe(lHeight, lWidth, CV_8UC1, (uint8_t*)buffer->getBase(0));

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

                ++lDoodleIndex %= 6;
            }
            catch (exception& e)
            {
                cout << "Error grabbing image: " << e.what() << endl;
                break;
            }
        }

        cout << endl;

        cout << "Stopping acquisition..." << endl;
        stream->stopStreaming();

        // Reset ROI to maximum
        int64_t vmax;
        rcg::getInteger(nodemap, "Width", nullptr, &vmax);
        rcg::setInteger(nodemap, "Width", vmax, true);

        rcg::getInteger(nodemap, "Height", nullptr, &vmax);
        rcg::setInteger(nodemap, "Height", vmax, true);
    }
    catch (exception& e)
    {
        cout << "Error during acquisition: " << e.what() << endl;
        stream->stopStreaming();
        throw;
    }
}

// Acquire reference intensity image
void AcquireIntensityRef(std::shared_ptr<rcg::Device> device, std::shared_ptr<rcg::Stream> stream,
                         Ljack_DAC *DAC_tiptilt,
                         string acquis_path, int dimROI)
{
    // Set reference voltages
    DAC_tiptilt->set_A_output(0);
    DAC_tiptilt->set_B_output(8);

    try
    {
        ConfigureCamera(device, dimROI);
        auto nodemap = device->getRemoteNodeMap();

        cout << endl << "##-- Reference Acquisition --##" << endl;
        stream->startStreaming();

        // Wait for DAC to settle
        boost::posix_time::milliseconds delay_cam(100);
        boost::this_thread::sleep(delay_cam);

        const rcg::Buffer* buffer = stream->grab(5000);

        if (buffer != nullptr && !buffer->getIsIncomplete())
        {
            DAC_tiptilt->set_A_output(9.5);
            DAC_tiptilt->set_B_output(9.5);

            size_t lWidth = buffer->getWidth(0);
            size_t lHeight = buffer->getHeight(0);

            cout << "  W: " << lWidth << " H: " << lHeight << endl;

            cv::Mat lframe(lHeight, lWidth, CV_8UC1, (uint8_t*)buffer->getBase(0));

            imwrite(acquis_path + "/Intensite_ref.pgm", lframe);
            cout << "Reference intensity saved to " << acquis_path << endl;
        }
        else
        {
            cout << "Failed to acquire reference image" << endl;
        }

        stream->stopStreaming();

        // Reset ROI to maximum
        int64_t vmax;
        rcg::getInteger(nodemap, "Width", nullptr, &vmax);
        rcg::setInteger(nodemap, "Width", vmax, true);

        rcg::getInteger(nodemap, "Height", nullptr, &vmax);
        rcg::setInteger(nodemap, "Height", vmax, true);
    }
    catch (exception& e)
    {
        cout << "Error during reference acquisition: " << e.what() << endl;
        throw;
    }
}

