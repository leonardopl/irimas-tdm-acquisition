# Basler Pylon Migration - Summary

## Successfully Completed ✓

The code has been successfully migrated from Pleora SDK to **Basler Pylon API** and compiled successfully.

### Files Created/Modified

1. **`main_basler.cpp`** - Complete rewrite using Basler Pylon API
   - Uses `CInstantCamera` instead of PvDevice/PvStream/PvPipeline
   - GenICam node map access for camera features
   - Smart pointer-based image grabbing (`CGrabResultPtr`)
   - Image format conversion with `CImageFormatConverter`

2. **`Makefile_basler`** - Build configuration for Pylon
   - Uses `pylon-config` for proper includes and libraries
   - Direct OpenCV library linking (core, imgcodecs, highgui, imgproc)
   - Target: `tomo_basler`

3. **`README_BASLER.md`** - Comprehensive migration guide
   - API comparison tables (Pleora vs Pylon)
   - Installation instructions
   - Compilation guide
   - Troubleshooting tips
   - Code examples

### Binary Output
- **Executable**: `bin/Release/tomo_basler` (325 KB)
- **Status**: Successfully compiled and ready to run

## Key API Differences

### Camera Management
| Aspect | Pleora SDK | Basler Pylon |
|--------|-----------|--------------|
| Main Object | PvDevice + PvStream + PvPipeline | CInstantCamera |
| Discovery | PvDeviceFinderWnd | CTlFactory::EnumerateDevices() |
| Connection | CreateAndConnect() | Open() |
| Acquisition | RetrieveNextBuffer() | StartGrabbing() + RetrieveResult() |

### Feature Access
```cpp
// Pleora: Multi-step casting
PvGenParameter* p = params->Get("Width");
PvGenInteger* width = dynamic_cast<PvGenInteger*>(p);
width->SetValue(1024);

// Pylon: Direct node access
CIntegerPtr width(nodemap.GetNode("Width"));
width->SetValue(1024);
```

### Image Grabbing
```cpp
// Pleora: Manual buffer management
PvBuffer* buffer;
pipeline->RetrieveNextBuffer(&buffer, timeout, &result);
pipeline->ReleaseBuffer(buffer);

// Pylon: Smart pointers (automatic cleanup)
CGrabResultPtr ptrGrabResult;
camera->RetrieveResult(timeout, ptrGrabResult, TimeoutHandling_ThrowException);
// Automatically released when out of scope
```

## Code Structure

### Main Components

1. **`SelectAndConnectCamera()`**
   - Enumerates available Basler cameras
   - User selection (if multiple cameras)
   - Opens and returns `CInstantCamera*`

2. **`ConfigureCamera()`**
   - Sets ROI (Width, Height, OffsetX, OffsetY)
   - Configures pixel format (Mono8)
   - Uses GenICam node map

3. **`AcquireImages()`**
   - Starts continuous grabbing
   - Loops through MAX_IMAGES acquisitions
   - Updates LabJack DAC voltages per scan pattern
   - Converts images to OpenCV Mat
   - Saves as PGM files

4. **`AcquireIntensiteRef()`**
   - Single reference image capture
   - Fixed DAC voltage (9.5V)
   - Saves as `Intensite_ref.pgm`

### Scan Pattern Integration
- Same scan pattern generation as original
- Supports: ROSACE, FERMAT, ARCHIMEDE, ANNULAR, UNIFORM3D, RANDOM_POLAR, STAR
- Voltage tables applied via LabJack DAC during acquisition

## Compilation Instructions

```bash
# Build
make -f Makefile_basler tomo_basler

# Clean
make -f Makefile_basler clean

# Run
./bin/Release/tomo_basler
```

## Environment Setup

```bash
# Add to ~/.bashrc
export PYLON_ROOT=/opt/pylon
export LD_LIBRARY_PATH=$PYLON_ROOT/lib:/opt/lib_tomo/labjack:$LD_LIBRARY_PATH
```

## Testing Checklist

Before running with hardware:

- [ ] Ensure Basler camera is connected (USB3/GigE)
- [ ] Pylon Viewer works (`/opt/pylon/bin/pylonviewer`)
- [ ] LabJack device connected
- [ ] Configuration files exist:
  - `~/.config/gui_tomo.conf`
  - `config_manip.txt`
  - `recon.txt`
- [ ] Output directories writable
- [ ] Library path includes `/opt/pylon/lib` and `/opt/lib_tomo/labjack`

## Next Steps

1. **Test with actual camera**: Run `./bin/Release/tomo_basler`
2. **Verify image quality**: Check generated PGM files
3. **Calibrate DAC voltages**: Ensure tip-tilt mirror control is correct
4. **Performance tuning**: Adjust grab strategy if needed
5. **Add features**: Consider trigger modes, exposure control, etc.

## To Run

```bash
# Set library paths
export LD_LIBRARY_PATH=/opt/pylon/lib:/opt/lib_tomo/labjack:$LD_LIBRARY_PATH

# Execute
./bin/Release/tomo_basler
```

## Advantages Over Pleora Version

✓ **Open Source**: Pylon SDK is free (vs Pleora's commercial SDK)  
✓ **Wider Compatibility**: Works with all Basler cameras (GigE, USB3, Camera Link)  
✓ **Better Documentation**: Extensive examples and API reference  
✓ **Active Support**: Regular updates and community support  
✓ **Simpler API**: Less boilerplate code required  
✓ **Smart Pointers**: Automatic memory management  

## File Sizes
- Source: `main_basler.cpp` - 581 lines
- Original: `main.cpp` - 892 lines
- **Reduction**: ~35% less code for same functionality

## Compilation Time
- First build: ~3-4 seconds
- Incremental: ~1-2 seconds

---

**Status**: Ready for hardware testing ✓  
**Compilation**: Success ✓  
**Documentation**: Complete ✓
