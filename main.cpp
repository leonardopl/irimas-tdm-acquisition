// TDM holographic tomography acquisition - IRIMAS

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;
#include <string>
#include <sstream>

// Basler Pylon API
#include <pylon/PylonIncludes.h>
using namespace Pylon;
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

#include "fonctions.h"
#include "scan_functions.h"

#include <list>

float tiptilt_factor_x = 0;
float tiptilt_factor_y = 0;
float2D VfOffset = {0, 0};
unsigned int b_AmpliRef = 0;
float NAcondLim = 1; // condenser NA scanning limit coefficient, 0 < value <= 1
size_t MAX_IMAGES = 0;

// Function Prototypes
CInstantCamera* SelectAndConnectCamera();
void ConfigureCamera(CInstantCamera* camera, int dimROI);
void AcquireImages(CInstantCamera* camera, Ljack_DAC *DAC_tiptilt, string result_path,
                   string acquis_path, int dimROI, vector<float2D> &Vout_table);
void AcquireIntensiteRef(CInstantCamera* camera, Ljack_DAC *DAC_tiptilt,
                         string result_path, string acquis_path, int dimROI);
string IntToString(int number);

int main(int argc, char *argv[])
{
    cout << "########################## ACQUISITION ##################################" << endl;

    // Initialize paths and configuration
    string gui_config_path, recon_path, manip_config_path, acquis_path, result_path, config_dir;
    string home = getenv("HOME");
    string gui_tomo_suffix = "/.config/gui_tomo.conf";
    gui_config_path = getenv("HOME") + gui_tomo_suffix;
    cout << "Reading paths from " << gui_config_path << endl;

    config_dir = extract_string("CHEMIN_CONFIG", home + gui_tomo_suffix);
    acquis_path = extract_string("CHEMIN_ACQUIS", home + gui_tomo_suffix);
    result_path = extract_string("CHEMIN_RESULT", home + gui_tomo_suffix);
    recon_path = config_dir + "/recon.txt";
    manip_config_path = config_dir + "/config_manip.txt";
    cout << "Config file: " << manip_config_path << endl;

    int dimROI = extract_val("DIM_ROI", manip_config_path);
    int NXMAX = extract_val("NXMAX", manip_config_path);

    string repAcquis = acquis_path;
    Var2D dimHolo = {2*NXMAX, 2*NXMAX};
    efface_acquis(repAcquis, manip_config_path, recon_path);

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

    // Generate voltage lookup table for scan pattern
    string scan_pattern_str = extract_string("SCAN_PATTERN", manip_config_path);
    int NbCirclesAnnular = extract_val("NB_CIRCLES_ANNULAR", manip_config_path);
    cout << "Scan pattern = " << scan_pattern_str << endl;

    int const rho = 10; // max voltage for LabJack
    int const nbHolo = MAX_IMAGES;
    vector<float2D> Vout_table(nbHolo);
    bool scan_pattern_exist = 0;

    if(scan_pattern_str == "ROSACE") { scan_fleur(rho, nbHolo, Vout_table); scan_pattern_exist = 1; }
    if(scan_pattern_str == "FERMAT") { scan_fermat(rho, nbHolo, Vout_table, dimHolo); scan_pattern_exist = 1; }
    if(scan_pattern_str == "ARCHIMEDE") { scan_spiralOS(rho, nbHolo, Vout_table, 4); scan_pattern_exist = 1; }
    if(scan_pattern_str == "ANNULAR") { scan_annular(rho, NbCirclesAnnular, nbHolo, Vout_table); scan_pattern_exist = 1; }
    if(scan_pattern_str == "UNIFORM3D") { scan_uniform3D(rho, nbHolo, Vout_table); scan_pattern_exist = 1; }
    if(scan_pattern_str == "RANDOM_POLAR") { scan_random_polar3D(rho, nbHolo, Vout_table, dimHolo); scan_pattern_exist = 1; }
    if(scan_pattern_str == "STAR") { scan_etoile(rho, nbHolo, 3, Vout_table); scan_pattern_exist = 1; }

    cout << "scan_pattern_exist=" << scan_pattern_exist << endl;
    if(scan_pattern_exist == 0)
        cout << "Cannot handle this scan pattern" << endl;

    // Initialize Pylon runtime
    PylonInitialize();

    CInstantCamera* camera = NULL;

    try
    {
        cout << "Basler Pylon Camera Acquisition:" << endl << endl;

        camera = SelectAndConnectCamera();

        if (camera != NULL && camera->IsOpen())
        {
            ConfigureCamera(camera, dimROI);

            // Acquire hologram images
            AcquireImages(camera, &ljDAC_flower, result_path, acquis_path, dimROI, Vout_table);

            // Acquire reference intensity if needed
            if(b_AmpliRef == 1)
                AcquireIntensiteRef(camera, &ljDAC_flower, result_path, acquis_path, dimROI);

            cout << "Closing camera" << endl;
            camera->Close();
            delete camera;
        }
        else
        {
            cout << "No camera connected" << endl;
        }
    }
    catch (const GenericException &e)
    {
        cout << "Pylon Exception: " << e.GetDescription() << endl;
        if (camera != NULL)
        {
            if (camera->IsOpen())
                camera->Close();
            delete camera;
        }
        PylonTerminate();
        return -1;
    }
    catch (exception& e)
    {
        cout << "Exception: " << e.what() << endl;
        if (camera != NULL)
        {
            if (camera->IsOpen())
                camera->Close();
            delete camera;
        }
        PylonTerminate();
        return -1;
    }

    PylonTerminate();

    // Reset DAC outputs
    ljDAC_flower.set_A_output(0.0);
    ljDAC_flower.set_B_output(0.0);

    return 0;
}

// Select and connect to Basler camera
CInstantCamera* SelectAndConnectCamera()
{
    try
    {
        CTlFactory& tlFactory = CTlFactory::GetInstance();

        DeviceInfoList_t devices;
        if (tlFactory.EnumerateDevices(devices) == 0)
        {
            cout << "No cameras found!" << endl;
            return NULL;
        }

        cout << "Found " << devices.size() << " camera(s):" << endl;
        for (size_t i = 0; i < devices.size(); i++)
        {
            cout << "  [" << i << "] " << devices[i].GetModelName()
                 << " (" << devices[i].GetSerialNumber() << ")" << endl;
        }

        int selectedCamera = 0;
        if (devices.size() > 1)
        {
            cout << "Select camera [0-" << devices.size()-1 << "]: ";
            cin >> selectedCamera;
            if (selectedCamera < 0 || selectedCamera >= (int)devices.size())
                selectedCamera = 0;
        }

        cout << "Connecting to camera " << selectedCamera << "..." << endl;

        CInstantCamera* camera = new CInstantCamera(tlFactory.CreateDevice(devices[selectedCamera]));
        camera->Open();

        if (camera->IsOpen())
        {
            cout << "Connected to: " << devices[selectedCamera].GetModelName() << endl;
        }

        return camera;
    }
    catch (const GenericException &e)
    {
        cout << "Error selecting camera: " << e.GetDescription() << endl;
        return NULL;
    }
}

// Configure camera ROI and settings
void ConfigureCamera(CInstantCamera* camera, int dimROI)
{
    try
    {
        GenApi::INodeMap& nodemap = camera->GetNodeMap();

        CIntegerPtr width(nodemap.GetNode("Width"));
        if (IsWritable(width))
        {
            width->SetValue(dimROI);
            cout << "Set Width to: " << width->GetValue() << endl;
        }

        CIntegerPtr height(nodemap.GetNode("Height"));
        if (IsWritable(height))
        {
            height->SetValue(dimROI);
            cout << "Set Height to: " << height->GetValue() << endl;
        }

        CIntegerPtr offsetX(nodemap.GetNode("OffsetX"));
        if (IsWritable(offsetX))
        {
            int64_t offsetXValue = dimROI / 2;
            offsetX->SetValue(offsetXValue);
            cout << "Set OffsetX to: " << offsetX->GetValue() << endl;
        }

        CIntegerPtr offsetY(nodemap.GetNode("OffsetY"));
        if (IsWritable(offsetY))
        {
            int64_t offsetYValue = dimROI / 2;
            offsetY->SetValue(offsetYValue);
            cout << "Set OffsetY to: " << offsetY->GetValue() << endl;
        }

        CEnumerationPtr pixelFormat(nodemap.GetNode("PixelFormat"));
        if (IsWritable(pixelFormat))
        {
            pixelFormat->FromString("Mono8");
            cout << "Pixel Format: " << pixelFormat->ToString() << endl;
        }

    }
    catch (const GenericException &e)
    {
        cout << "Error configuring camera: " << e.GetDescription() << endl;
        throw;
    }
}

// Acquire hologram images
void AcquireImages(CInstantCamera* camera, Ljack_DAC *DAC_tiptilt, string result_path,
                   string acquis_path, int dimROI, vector<float2D> &Vout_table)
{
    int nbHolo = MAX_IMAGES;

    try
    {
        GenApi::INodeMap& nodemap = camera->GetNodeMap();

        CIntegerPtr width(nodemap.GetNode("Width"));
        if (IsWritable(width))
            width->SetValue(dimROI);

        CIntegerPtr height(nodemap.GetNode("Height"));
        if (IsWritable(height))
            height->SetValue(dimROI);

        cout << "Starting acquisition..." << endl;

        // OneByOne ensures synchronization with mirror steps
        camera->StartGrabbing(GrabStrategy_OneByOne, GrabLoop_ProvidedByUser);

        CImageFormatConverter formatConverter;
        formatConverter.OutputPixelFormat = PixelType_Mono8;
        CPylonImage pylonImage;
        CGrabResultPtr ptrGrabResult;

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

        while (img_count < MAX_IMAGES && camera->IsGrabbing())
        {
            try
            {
                camera->RetrieveResult(5000, ptrGrabResult, TimeoutHandling_ThrowException);

                if (ptrGrabResult->GrabSucceeded())
                {
                    double t_now = (double)cv::getTickCount();
                    double dt = (t_now - t_last) / cv::getTickFrequency();
                    t_last = t_now;

                    if (dt > 0)
                    {
                        double instant_fps = 1.0 / dt;
                        if (current_fps == 0) current_fps = instant_fps;
                        else current_fps = (1.0 - alpha) * current_fps + alpha * instant_fps;

                        // Bandwidth in MB/s (1 pixel = 1 byte for Mono8)
                        double frameBytes = (double)ptrGrabResult->GetWidth() * (double)ptrGrabResult->GetHeight();
                        bandwidth_MBps = (frameBytes * current_fps) / 1000000.0;
                    }

                    formatConverter.Convert(pylonImage, ptrGrabResult);
                    cv::Mat lframe(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC1, (uint8_t*)pylonImage.GetBuffer());

                    imwrite(acquis_path + format("/i%03d.pgm", img_count), lframe);

                    // Update mirrors (DAC) for next image
                    Vscan.x = Vout_table[img_count].x;
                    Vscan.y = Vout_table[img_count].y;
                    DAC_tiptilt->set_A_output(Vscan.x*tiptilt_factor_x + VfOffset.x);
                    DAC_tiptilt->set_B_output(Vscan.y*tiptilt_factor_y + VfOffset.y);

                    img_count++;

                    cout << fixed << setprecision(1);
                    cout << lDoodle[lDoodleIndex]
                         << " W: " << ptrGrabResult->GetWidth()
                         << " H: " << ptrGrabResult->GetHeight()
                         << " | " << current_fps << " FPS "
                         << bandwidth_MBps << " MB/s "
                         << "(" << img_count << "/" << MAX_IMAGES << ")\r";
                    cout.flush();
                }
                else
                {
                    cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << endl;
                }

                ++lDoodleIndex %= 6;
            }
            catch (const GenericException &e)
            {
                cout << "Error grabbing image: " << e.GetDescription() << endl;
                break;
            }
        }

        cout << endl;

        cout << "Stopping acquisition..." << endl;
        camera->StopGrabbing();

        // Reset ROI to maximum
        CIntegerPtr widthMax(nodemap.GetNode("Width"));
        if (IsWritable(widthMax)) widthMax->SetValue(widthMax->GetMax());

        CIntegerPtr heightMax(nodemap.GetNode("Height"));
        if (IsWritable(heightMax)) heightMax->SetValue(heightMax->GetMax());
    }
    catch (const GenericException &e)
    {
        cout << "Error during acquisition: " << e.GetDescription() << endl;
        if(camera->IsGrabbing()) camera->StopGrabbing();
        throw;
    }
}

// Acquire reference intensity image
void AcquireIntensiteRef(CInstantCamera* camera, Ljack_DAC *DAC_tiptilt,
                         string result_path, string acquis_path, int dimROI)
{
    // Set reference voltages
    DAC_tiptilt->set_A_output(0);
    DAC_tiptilt->set_B_output(8);

    try
    {
        GenApi::INodeMap& nodemap = camera->GetNodeMap();

        CIntegerPtr width(nodemap.GetNode("Width"));
        if (IsWritable(width))
            width->SetValue(dimROI);

        CIntegerPtr height(nodemap.GetNode("Height"));
        if (IsWritable(height))
            height->SetValue(dimROI);

        CIntegerPtr offsetX(nodemap.GetNode("OffsetX"));
        if (IsWritable(offsetX))
            offsetX->SetValue(dimROI / 2);

        CIntegerPtr offsetY(nodemap.GetNode("OffsetY"));
        if (IsWritable(offsetY))
            offsetY->SetValue(dimROI / 2);

        cout << endl << "##-- Reference Acquisition --##" << endl;
        camera->StartGrabbing(GrabStrategy_OneByOne);

        CImageFormatConverter formatConverter;
        formatConverter.OutputPixelFormat = PixelType_Mono8;
        CPylonImage pylonImage;

        // Wait for DAC to settle
        boost::posix_time::milliseconds delay_cam(100);
        boost::this_thread::sleep(delay_cam);

        CGrabResultPtr ptrGrabResult;
        camera->RetrieveResult(5000, ptrGrabResult, TimeoutHandling_ThrowException);

        if (ptrGrabResult->GrabSucceeded())
        {
            DAC_tiptilt->set_A_output(9.5);
            DAC_tiptilt->set_B_output(9.5);

            size_t lWidth = ptrGrabResult->GetWidth();
            size_t lHeight = ptrGrabResult->GetHeight();

            cout << "  W: " << lWidth << " H: " << lHeight << endl;

            formatConverter.Convert(pylonImage, ptrGrabResult);
            cv::Mat lframe(lHeight, lWidth, CV_8UC1, (uint8_t*)pylonImage.GetBuffer());

            imwrite(acquis_path + "/Intensite_ref.pgm", lframe);
            cout << "Reference intensity saved to " << acquis_path << endl;
        }
        else
        {
            cout << "Failed to acquire reference image: " << ptrGrabResult->GetErrorDescription() << endl;
        }

        camera->StopGrabbing();

        // Reset ROI to maximum
        CIntegerPtr widthMax(nodemap.GetNode("Width"));
        if (IsWritable(widthMax))
            widthMax->SetValue(widthMax->GetMax());

        CIntegerPtr heightMax(nodemap.GetNode("Height"));
        if (IsWritable(heightMax))
            heightMax->SetValue(heightMax->GetMax());
    }
    catch (const GenericException &e)
    {
        cout << "Error during reference acquisition: " << e.GetDescription() << endl;
        throw;
    }
}

std::string IntToString(int number)
{
    std::ostringstream oss;
    oss << number;
    return oss.str();
}
