# Basler Pylon API Migration Guide

## Overview
This version replaces Pleora SDK with Basler Pylon API for camera control. The code maintains the same functionality for holographic tomography acquisition with LabJack DAC control.

## Key Changes

### 1. **Camera API Migration**
**Pleora SDK → Basler Pylon API**

| Pleora SDK | Basler Pylon API |
|------------|------------------|
| `PvDevice`, `PvStream`, `PvPipeline` | `CInstantCamera` |
| `PvDeviceFinderWnd::ShowModal()` | `CTlFactory::EnumerateDevices()` |
| `PvDevice::CreateAndConnect()` | `camera->Open()` |
| `PvPipeline::RetrieveNextBuffer()` | `camera->RetrieveResult()` |
| `PvBuffer::GetImage()` | `ptrGrabResult->GrabSucceeded()` |
| `PvGenInteger`, `PvGenParameter` | `CIntegerPtr`, `CEnumerationPtr` |

### 2. **Simplified Architecture**
- **Before**: Separate Device, Stream, and Pipeline objects
- **After**: Single `CInstantCamera` object handles all operations
- **Buffer management**: Uses smart pointers (`CGrabResultPtr`)

### 3. **Feature Access**
```cpp
// Pleora SDK
PvGenParameterArray* params = device->GetParameters();
PvGenParameter* param = params->Get("Width");
PvGenInteger* width = dynamic_cast<PvGenInteger*>(param);
width->SetValue(1024);

// Basler Pylon API
GenApi::INodeMap& nodemap = camera->GetNodeMap();
CIntegerPtr width(nodemap.GetNode("Width"));
if (IsWritable(width))
    width->SetValue(1024);
```

### 4. **Image Acquisition**
```cpp
// Pleora SDK
PvBuffer* buffer;
pipeline->RetrieveNextBuffer(&buffer, 1000, &opResult);
PvImage* image = buffer->GetImage();
unsigned char* data = image->GetDataPointer();

// Basler Pylon API
CGrabResultPtr ptrGrabResult;
camera->RetrieveResult(5000, ptrGrabResult, TimeoutHandling_ThrowException);
if (ptrGrabResult->GrabSucceeded()) {
    const uint8_t* data = (uint8_t*)ptrGrabResult->GetBuffer();
}
```

## Installation Requirements

### 1. Install Basler Pylon SDK
```bash
# Download from Basler website
# https://www.baslerweb.com/en/downloads/software-downloads/

# Install (example for Ubuntu)
sudo dpkg -i pylon_*.deb
```

### 2. Verify Installation
```bash
# Check installation path
ls /opt/pylon/include/pylon/
ls /opt/pylon/lib/libpylonbase.so

# Test pylon-config
/opt/pylon/bin/pylon-config --version
```

### 3. Setup Environment
```bash
# Add Pylon and LabJack to library path
export PYLON_ROOT=/opt/pylon
export LD_LIBRARY_PATH=$PYLON_ROOT/lib:/opt/lib_tomo/labjack:$LD_LIBRARY_PATH

# Add to ~/.bashrc for permanent setup
echo 'export PYLON_ROOT=/opt/pylon' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$PYLON_ROOT/lib:/opt/lib_tomo/labjack:$LD_LIBRARY_PATH' >> ~/.bashrc
```

## Compilation

### Quick Setup (Recommended)
```bash
# Run the setup script to check environment
source setup_env.sh

# Build
make -f Makefile_basler tomo_basler
```

### Build Basler Version
```bash
make -f Makefile_basler tomo_basler
```

### Build Original Pleora Version
```bash
make tomo_manip
```

### Clean Build
```bash
make -f Makefile_basler clean
```

## Usage

### Run Acquisition
```bash
./bin/Release/tomo_basler
```

### Camera Selection
- If multiple cameras detected, you'll be prompted to select one
- Single camera: automatically selected

## Configuration

The code uses the same configuration files as the original:
- `~/.config/gui_tomo.conf` - Paths configuration
- `config_manip.txt` - Acquisition parameters (ROI, patterns, voltages)
- `recon.txt` - Reconstruction parameters

## Supported Features

✅ **Implemented:**
- Camera discovery and connection
- ROI configuration (Width, Height, OffsetX, OffsetY)
- Continuous acquisition
- Multiple scan patterns (ROSACE, FERMAT, ARCHIMEDE, etc.)
- LabJack DAC control for beam steering
- Reference intensity acquisition
- PGM image output

⚠️ **Notes:**
- Currently configured for Mono8 pixel format
- For 12-bit or 16-bit cameras, change `PixelFormat` to "Mono12" or "Mono16"
- Adjust OpenCV Mat type accordingly (CV_16UC1 for 16-bit)

## Pixel Format Configuration

### For 8-bit Cameras (default)
```cpp
CEnumerationPtr pixelFormat(nodemap.GetNode("PixelFormat"));
pixelFormat->FromString("Mono8");
cv::Mat lframe(lHeight, lWidth, CV_8UC1, (uint8_t*)pylonImage.GetBuffer());
```

### For 12-bit/16-bit Cameras
```cpp
CEnumerationPtr pixelFormat(nodemap.GetNode("PixelFormat"));
pixelFormat->FromString("Mono12"); // or "Mono16"
// Use Pylon's image format converter for proper conversion
formatConverter.OutputPixelFormat = PixelType_Mono16;
formatConverter.Convert(pylonImage, ptrGrabResult);
cv::Mat lframe(lHeight, lWidth, CV_16UC1, (uint16_t*)pylonImage.GetBuffer());
```

## Troubleshooting

### Camera Not Found
```bash
# Check USB connection
lsusb | grep Basler

# Check permissions
sudo usermod -a -G plugdev $USER
# Logout and login again
```

### Library Not Found
```bash
# Add to library path
export LD_LIBRARY_PATH=/opt/pylon/lib:/opt/lib_tomo/labjack:$LD_LIBRARY_PATH

# Or add permanently to ~/.bashrc
echo 'export LD_LIBRARY_PATH=/opt/pylon/lib:/opt/lib_tomo/labjack:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

### Compilation Errors
```bash
# Install development packages
sudo apt-get install libopencv-dev libboost-all-dev

# Check Pylon installation
/opt/pylon/bin/pylon-config --cflags
/opt/pylon/bin/pylon-config --libs

# If OpenCV pkg-config not available, libraries are linked directly in Makefile
```

## Performance Considerations

1. **Grab Strategy**: 
   - `GrabStrategy_LatestImageOnly`: Discards old images (best for real-time)
   - `GrabStrategy_OneByOne`: Process all images in order
2. **Packet Size**: GigE Vision packet size auto-negotiated for GigE cameras
3. **Timeout**: Set in `RetrieveResult(timeout_ms, ...)` based on frame rate
4. **Image Converter**: Uses `CImageFormatConverter` for pixel format conversion

## Example Output
```
########################## ACQUISITION ##################################
lecture des chemins dans le fichier /home/user/.config/gui_tomo.conf
fichier config : /path/to/config_manip.txt
Scan pattern = ROSACE
Basler Pylon Camera Acquisition:

Found 1 camera(s):
  [0] acA2000-165um (40123456)
Connecting to camera 0...
Connected to: acA2000-165um
Set Width to: 1024
Set Height to: 1024
Set OffsetX to: 512
Set OffsetY to: 512
Pixel Format: Mono8
Starting acquisition...

##-- Acquisition Hologrammes --##
|  W: 1024 H: 1024  Image 1/100
...
```

## Migration Checklist

- [x] Replace Pleora headers with Pylon API
- [x] Migrate device discovery using `CTlFactory::EnumerateDevices()`
- [x] Simplify camera connection with `CInstantCamera`
- [x] Convert ROI configuration using `CIntegerPtr` and `INodeMap`
- [x] Update acquisition loop with `StartGrabbing()/RetrieveResult()`
- [x] Adapt image buffer handling with `CGrabResultPtr`
- [x] Update Makefile with Pylon libraries via `pylon-config`
- [x] Test compilation successfully

## Known Limitations

1. **GUI**: No built-in device selection GUI (command-line selection only)
2. **GigE Vision**: Automatic packet size negotiation (manual control available via GenICam)
3. **Trigger**: External trigger support needs additional GenICam configuration

## Further Enhancements

Consider adding:
- Hardware trigger configuration (`TriggerMode`, `TriggerSource`)
- Frame rate monitoring with Pylon statistics
- Camera temperature monitoring via `DeviceTemperature` feature
- Auto-exposure/gain control using `ExposureAuto`, `GainAuto`
- Multi-camera support with camera arrays
- Chunk data for embedded timestamp/frame counter

## Support

For Basler Pylon documentation:
- https://docs.baslerweb.com/
- Pylon Programmer's Guide and API Reference
- Sample code in `/opt/pylon/share/pylon/Samples/`
- GenICam SFNC (Standard Features Naming Convention)
