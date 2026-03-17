// *****************************************************************************
//
// Modified to use Basler Pylon instead of Pleora SDK
// Basler Pylon Camera Acquisition with LabJack DAC control
//
// *****************************************************************************

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
#include "vectra.h"
#include "string.h"
#include "vChronos.h"
#include <boost/thread.hpp>
#include <boost/date_time.hpp>
#include "vPlotterT.h"

#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include "fonctions.h"
#include "scan_functions.h"

#include <list>

float tiptilt_factor_x = 0; // init rayon X en volt
float tiptilt_factor_y = 0; // init rayon Y en volt
float2D VfOffset = {-0, -0};
unsigned int b_AmpliRef = 0;
float NAcondLim = 1; // coefficient de limitation de l'ouverture numerique de balayage 0<valeur<=1
float flower_x(float t);
float flower_y(float t);
size_t MAX_IMAGES = 0;

///
/// Function Prototypes
///
CInstantCamera* SelectAndConnectCamera();
void ConfigureCamera(CInstantCamera* camera, int dimROI);
void AcquireImages(CInstantCamera* camera, Ljack_DAC *DAC_tiptilt, string chemin_result, 
                   string chemin_acquis, int dimROI, vector<float2D> &Vout_table);
void AcquireIntensiteRef(CInstantCamera* camera, Ljack_DAC *DAC_tiptilt, 
                         string chemin_result, string chemin_acquis, int dimROI);
string IntToString(int number);

// Main function
int main(int argc, char *argv[])
{
    cout << "########################## ACQUISITION ##################################" << endl;
    
    // Initialize paths and configuration
    string chemin_config_GUI, chemin_recon, chemin_config_manip, chemin_acquis, chemin_result, repertoire_config;
    string home = getenv("HOME");
    string fin_chemin_gui_tomo = "/.config/gui_tomo.conf";
    chemin_config_GUI = getenv("HOME") + fin_chemin_gui_tomo;
    cout << "lecture des chemins dans le fichier " << chemin_config_GUI << endl;

    repertoire_config = extract_string("CHEMIN_CONFIG", home + fin_chemin_gui_tomo);
    chemin_acquis = extract_string("CHEMIN_ACQUIS", home + fin_chemin_gui_tomo);
    chemin_result = extract_string("CHEMIN_RESULT", home + fin_chemin_gui_tomo);
    chemin_recon = repertoire_config + "/recon.txt";
    chemin_config_manip = repertoire_config + "/config_manip.txt";
    cout << "fichier config : " << chemin_config_manip << endl;

    int dimROI = extract_val("DIM_ROI", chemin_config_manip);
    int NXMAX = extract_val("NXMAX", chemin_config_manip);

    string repAcquis = chemin_acquis;
    Var2D dimHolo = {2*NXMAX, 2*NXMAX};
    efface_acquis(repAcquis, chemin_config_manip, chemin_recon);

    // Extract configuration parameters
    MAX_IMAGES = extract_val("NB_HOLO", chemin_config_manip);
    NAcondLim = extract_val("NA_COND_LIM", chemin_config_manip);
    tiptilt_factor_x = (abs(extract_val("VXMIN", chemin_config_manip)) + 
                        abs(extract_val("VXMAX", chemin_config_manip))) * NAcondLim / 20;
    tiptilt_factor_y = (abs(extract_val("VYMIN", chemin_config_manip)) + 
                        abs(extract_val("VYMAX", chemin_config_manip))) * NAcondLim / 20;

    VfOffset.x = (extract_val("VXMIN", chemin_config_manip) * NAcondLim + tiptilt_factor_x * 10);
    VfOffset.y = (extract_val("VYMIN", chemin_config_manip) * NAcondLim + tiptilt_factor_y * 10) * NAcondLim;
    b_AmpliRef = extract_val("AMPLI_REF", chemin_recon);

    cout << "  ####################################" << endl;
    cout << "  #  tension MAX x=" << tiptilt_factor_x*10 << "                   #" << endl;
    cout << "  #  tension MAX y=" << tiptilt_factor_y*10 << "                     #" << endl;
    cout << "  #  VfOffset.x=" << VfOffset.x << " | VfOffset.y=" << VfOffset.y << "  #" << endl;
    cout << "  #  Nombre d'hologrammes=" << MAX_IMAGES << "        #" << endl;
    cout << "  ####################################" << endl;

    // Initialize LabJack DAC
    Ljack LU3HV;
    Ljack_DAC ljDAC_flower(LU3HV);
    ASSERT(ljDAC_flower.connect(FIO5_4));

    ljDAC_flower.set_A_output(VfOffset.x);
    ljDAC_flower.set_B_output(VfOffset.y);

    // Generate voltage lookup table for scan pattern
    string scan_pattern_str = extract_string("SCAN_PATTERN", chemin_config_manip);
    int NbCirclesAnnular = extract_val("NB_CIRCLES_ANNULAR", chemin_config_manip);
    cout << "Scan pattern = " << scan_pattern_str << endl;
    
    int const rho = 10; // Vmax admissible par labjack
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

        // Select and connect to camera
        camera = SelectAndConnectCamera();
        
        if (camera != NULL && camera->IsOpen())
        {
            // Configure camera with ROI settings
            ConfigureCamera(camera, dimROI);
            
            // Acquire hologram images
            AcquireImages(camera, &ljDAC_flower, chemin_result, chemin_acquis, dimROI, Vout_table);
            
            // Acquire reference intensity if needed
            if(b_AmpliRef == 1)
                AcquireIntensiteRef(camera, &ljDAC_flower, chemin_result, chemin_acquis, dimROI);
            
            // Close camera
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

    // Terminate Pylon runtime
    PylonTerminate();

    // Reset DAC outputs
    ljDAC_flower.set_A_output(0.0);
    ljDAC_flower.set_B_output(0.0);
    
    ofstream file;
    file.open("codebind.txt");
    file << "Please write this text to a file.\n this text is written using C++\n";
    file.close();
    
    return 0;
}

///
/// Select and connect to Basler camera
///
CInstantCamera* SelectAndConnectCamera()
{
    try
    {
        // Get the transport layer factory
        CTlFactory& tlFactory = CTlFactory::GetInstance();
        
        // Get all attached devices
        DeviceInfoList_t devices;
        if (tlFactory.EnumerateDevices(devices) == 0)
        {
            cout << "No cameras found!" << endl;
            return NULL;
        }
        
        // Display available cameras
        cout << "Found " << devices.size() << " camera(s):" << endl;
        for (size_t i = 0; i < devices.size(); i++)
        {
            cout << "  [" << i << "] " << devices[i].GetModelName() 
                 << " (" << devices[i].GetSerialNumber() << ")" << endl;
        }
        
        // Select camera (first one by default, or user can select)
        int selectedCamera = 0;
        if (devices.size() > 1)
        {
            cout << "Select camera [0-" << devices.size()-1 << "]: ";
            cin >> selectedCamera;
            if (selectedCamera < 0 || selectedCamera >= (int)devices.size())
                selectedCamera = 0;
        }
        
        cout << "Connecting to camera " << selectedCamera << "..." << endl;
        
        // Create an instant camera object with the first found camera device
        CInstantCamera* camera = new CInstantCamera(tlFactory.CreateDevice(devices[selectedCamera]));
        
        // Open the camera
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

///
/// Configure camera ROI and other settings
///
void ConfigureCamera(CInstantCamera* camera, int dimROI)
{
    try
    {
        // Get the camera's node map
        GenApi::INodeMap& nodemap = camera->GetNodeMap();
        
        // Set ROI Width
        CIntegerPtr width(nodemap.GetNode("Width"));
        if (IsWritable(width))
        {
            width->SetValue(dimROI);
            cout << "Set Width to: " << width->GetValue() << endl;
        }
        
        // Set ROI Height
        CIntegerPtr height(nodemap.GetNode("Height"));
        if (IsWritable(height))
        {
            height->SetValue(dimROI);
            cout << "Set Height to: " << height->GetValue() << endl;
        }
        
        // Set ROI OffsetX (centered)
        CIntegerPtr offsetX(nodemap.GetNode("OffsetX"));
        if (IsWritable(offsetX))
        {
            int64_t offsetXValue = dimROI / 2;
            offsetX->SetValue(offsetXValue);
            cout << "Set OffsetX to: " << offsetX->GetValue() << endl;
        }
        
        // Set ROI OffsetY (centered)
        CIntegerPtr offsetY(nodemap.GetNode("OffsetY"));
        if (IsWritable(offsetY))
        {
            int64_t offsetYValue = dimROI / 2;
            offsetY->SetValue(offsetYValue);
            cout << "Set OffsetY to: " << offsetY->GetValue() << endl;
        }
        
        // Set pixel format to Mono8 (or Mono12/Mono16 depending on your needs)
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

///
/// Acquire hologram images
///
void AcquireImages(CInstantCamera* camera, Ljack_DAC *DAC_tiptilt, string chemin_result, 
                   string chemin_acquis, int dimROI, vector<float2D> &Vout_table)
{
    int nbHolo = MAX_IMAGES;
    
    try
    {
        // Get the camera's node map
        GenApi::INodeMap& nodemap = camera->GetNodeMap();
        
        // Configure camera ROI
        CIntegerPtr width(nodemap.GetNode("Width"));
        if (IsWritable(width))
            width->SetValue(dimROI);
        
        CIntegerPtr height(nodemap.GetNode("Height"));
        if (IsWritable(height))
            height->SetValue(dimROI);
        
        // Start grabbing
        cout << "Starting acquisition..." << endl;
        
        // Use OneByOne to ensure synchronization with the mirror steps
        camera->StartGrabbing(GrabStrategy_OneByOne, GrabLoop_ProvidedByUser);
        
        // Tools for image conversion
        CImageFormatConverter formatConverter;
        formatConverter.OutputPixelFormat = PixelType_Mono8;
        CPylonImage pylonImage;
        CGrabResultPtr ptrGrabResult;

        // Initialize DAC
        float2D Vfleur = {0*tiptilt_factor_x, 0*tiptilt_factor_y};
        DAC_tiptilt->set_A_output(Vfleur.x*tiptilt_factor_x + VfOffset.x);
        DAC_tiptilt->set_B_output(Vfleur.y*tiptilt_factor_y + VfOffset.y);
        
        cout << endl << "##-- Acquisition Hologrammes --##" << endl;
        
        // Visualization characters
        char lDoodle[] = "|\\-|-/";
        int lDoodleIndex = 0;
        size_t cpt_img = 0;
        
        // --- FPS Calculation Variables ---
        double t_last = (double)cv::getTickCount();
        double current_fps = 0.0;
        double bandwidth_MBps = 0.0; // Changed variable name to MBps
        double alpha = 0.1; // Smoothing factor
        // ---------------------------------

        vChronos vTime("prise d'images--");
        vTime.clear();
        vTime.start();
        
        // Acquisition loop
        while (cpt_img < MAX_IMAGES && camera->IsGrabbing())
        {
            try
            {
                // Wait for image (Timeout 5000ms)
                camera->RetrieveResult(5000, ptrGrabResult, TimeoutHandling_ThrowException);
                
                if (ptrGrabResult->GrabSucceeded())
                {
                    // 1. Calculate Timing Stats
                    double t_now = (double)cv::getTickCount();
                    double dt = (t_now - t_last) / cv::getTickFrequency();
                    t_last = t_now;

                    if (dt > 0)
                    {
                        double instant_fps = 1.0 / dt;
                        // Smooth the FPS for readable display
                        if (current_fps == 0) current_fps = instant_fps;
                        else current_fps = (1.0 - alpha) * current_fps + alpha * instant_fps;
                        
                        // Calculate Bandwidth in MB/s (Megabytes per second)
                        // 1 Pixel = 1 Byte (Mono8)
                        double frameBytes = (double)ptrGrabResult->GetWidth() * (double)ptrGrabResult->GetHeight();
                        bandwidth_MBps = (frameBytes * current_fps) / 1000000.0; 
                    }

                    // 2. Process Image
                    formatConverter.Convert(pylonImage, ptrGrabResult);
                    cv::Mat lframe(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC1, (uint8_t*)pylonImage.GetBuffer());
                    
                    // Save image
                    imwrite(chemin_acquis + format("/i%03d.pgm", cpt_img), lframe);
                    
                    // 3. Update Mirrors (DAC) for NEXT image
                    Vfleur.x = Vout_table[cpt_img].x;
                    Vfleur.y = Vout_table[cpt_img].y;
                    DAC_tiptilt->set_A_output(Vfleur.x*tiptilt_factor_x + VfOffset.x);
                    DAC_tiptilt->set_B_output(Vfleur.y*tiptilt_factor_y + VfOffset.y);
                    
                    cpt_img++;
                    
                    // 4. Display Status
                    cout << fixed << setprecision(1);
                    cout << lDoodle[lDoodleIndex] 
                         << " W: " << ptrGrabResult->GetWidth() 
                         << " H: " << ptrGrabResult->GetHeight()
                         << " | " << current_fps << " FPS " 
                         << bandwidth_MBps << " MB/s "
                         << "(" << cpt_img << "/" << MAX_IMAGES << ")\r";
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
        
        cout << endl; // Clear line
        
        // Stop grabbing
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

///
/// Acquire reference intensity image
///
void AcquireIntensiteRef(CInstantCamera* camera, Ljack_DAC *DAC_tiptilt, 
                         string chemin_result, string chemin_acquis, int dimROI)
{
    // Set reference voltages
    DAC_tiptilt->set_A_output(0);
    DAC_tiptilt->set_B_output(8);
    
    try
    {
        // Get the camera's node map
        GenApi::INodeMap& nodemap = camera->GetNodeMap();
        
        // Configure camera ROI
        CIntegerPtr width(nodemap.GetNode("Width"));
        if (IsWritable(width))
            width->SetValue(dimROI);
        
        CIntegerPtr height(nodemap.GetNode("Height"));
        if (IsWritable(height))
            height->SetValue(dimROI);
        
        // Set ROI offsets (centered)
        CIntegerPtr offsetX(nodemap.GetNode("OffsetX"));
        if (IsWritable(offsetX))
            offsetX->SetValue(dimROI / 2);
        
        CIntegerPtr offsetY(nodemap.GetNode("OffsetY"));
        if (IsWritable(offsetY))
            offsetY->SetValue(dimROI / 2);
        
        // Start acquisition
        cout << endl << "##-- Acquisition Référence --##" << endl;
        camera->StartGrabbing(GrabStrategy_OneByOne);
        
        // Create a Pylon image format converter
        CImageFormatConverter formatConverter;
        formatConverter.OutputPixelFormat = PixelType_Mono8;
        
        // Create a Pylon image
        CPylonImage pylonImage;
        
        // Wait a bit for DAC to settle
        boost::posix_time::milliseconds delay_cam(100);
        boost::this_thread::sleep(delay_cam);
        
        // This smart pointer will receive the grab result data
        CGrabResultPtr ptrGrabResult;
        
        // Grab reference image
        camera->RetrieveResult(5000, ptrGrabResult, TimeoutHandling_ThrowException);
        
        if (ptrGrabResult->GrabSucceeded())
        {
            // Set DAC to reference values
            DAC_tiptilt->set_A_output(9.5);
            DAC_tiptilt->set_B_output(9.5);
            
            size_t lWidth = ptrGrabResult->GetWidth();
            size_t lHeight = ptrGrabResult->GetHeight();
            
            cout << "  W: " << lWidth << " H: " << lHeight << endl;
            
            // Convert the grabbed buffer to a pylon image
            formatConverter.Convert(pylonImage, ptrGrabResult);
            
            // Create an OpenCV image from the pylon image
            cv::Mat lframe(lHeight, lWidth, CV_8UC1, (uint8_t*)pylonImage.GetBuffer());
            
            // Save reference image
            imwrite(chemin_acquis + "/Intensite_ref.pgm", lframe);
            cout << "Acquisition intensite reference dans " << chemin_acquis << endl;
        }
        else
        {
            cout << "Failed to acquire reference image: " << ptrGrabResult->GetErrorDescription() << endl;
        }
        
        // Stop acquisition
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

///
/// Convert integer to string
///
std::string IntToString(int number)
{
    std::ostringstream oss;
    oss << number;
    return oss.str();
}

///
/// Flower pattern functions
///
float rayon_balayage = 10.00f;

float flower_x(float t)
{
    float c4t10 = rayon_balayage * cos(4 * t);
    return c4t10 * cos(t);
}

float flower_y(float t)
{
    float c4t10 = rayon_balayage * cos(4 * t);
    return c4t10 * sin(t);
}
