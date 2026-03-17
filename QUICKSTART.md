# Quick Reference - Basler Pylon Version

## Setup (One Time)
```bash
# Add to ~/.bashrc for permanent setup
echo 'export PYLON_ROOT=/opt/pylon' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$PYLON_ROOT/lib:/opt/lib_tomo/labjack:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

## Compile
```bash
make -f Makefile_basler tomo_basler
```

## Run
```bash
# If not in ~/.bashrc:
source setup_env.sh

# Execute
./bin/Release/tomo_basler
```

## Clean Build
```bash
make -f Makefile_basler clean
make -f Makefile_basler tomo_basler
```

## Files
- **Source**: `main_basler.cpp`
- **Makefile**: `Makefile_basler`
- **Binary**: `bin/Release/tomo_basler`
- **Docs**: `README_BASLER.md`

## Environment Check
```bash
# Check Pylon installation
/opt/pylon/bin/pylon-config --version

# Check libraries
ls /opt/pylon/lib/libpylonbase.so
ls /opt/lib_tomo/labjack/

# Test camera detection
/opt/pylon/bin/pylonviewer
```

## Common Issues

### Library not found
```bash
export LD_LIBRARY_PATH=/opt/pylon/lib:/opt/lib_tomo/labjack:$LD_LIBRARY_PATH
```

### Camera not detected
```bash
# Check USB permissions
lsusb | grep Basler
sudo usermod -a -G plugdev $USER
# Logout and login again
```

### LabJack not found
```bash
# Check LabJack library path
ls /opt/lib_tomo/labjack/
# Ensure library path is set correctly
```

## Key Differences from Pleora Version

| Aspect | Pleora | Basler Pylon |
|--------|--------|--------------|
| Binary | `tomo_manip` | `tomo_basler` |
| Makefile | `Makefile` | `Makefile_basler` |
| SDK Cost | Commercial | Free |
| Lines of Code | 892 | 581 |
| API Objects | 3 (Device/Stream/Pipeline) | 1 (CInstantCamera) |

## Configuration Files (Same as Original)
- `~/.config/gui_tomo.conf` - Paths
- `config_manip.txt` - Acquisition parameters
- `recon.txt` - Reconstruction settings

## Output Files
- `i000.pgm, i001.pgm, ...` - Acquired holograms
- `Intensite_ref.pgm` - Reference intensity (if enabled)
