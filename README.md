# TDM Acquisition

Hologram acquisition software for Tomographic Diffraction Microscopy (TDM).
Controls a GenICam-compatible camera and LabJack U3-HV DAC for automated beam steering
during off-axis holographic tomography experiments.

Adapted from the acquisition module of [MTD_transmission](https://github.com/madeba/MTD_transmission) by Matthieu Debailleul (IRIMAS, Université de Haute-Alsace). Ported from the Pleora eBUS SDK to [rc_genicam_api](https://github.com/roboception/rc_genicam_api) for multi-vendor camera support.

## Changes from upstream

Key differences from the original [MTD_transmission](https://github.com/madeba/MTD_transmission) acquisition module:

- **SDK migration**: Pleora eBUS SDK → rc_genicam_api; supports any GenICam-compatible camera (GigE Vision, USB3 Vision)
- **DAC settling fix**: Reference acquisition now waits for DAC settling (100 ms) before image capture, rather than after
- **Explicit pixel format**: Camera is configured to Mono8, removing reliance on device defaults
- **SIGINT handling**: Added signal handler to reset DAC outputs on interrupt (Ctrl+C)
- **Error handling**: Structured exception handling with proper camera cleanup on failure
- **Codebase cleanup**: Translated from French to English; removed unused functions and dead code

## Hardware

- GenICam-compatible area-scan camera (GigE Vision or USB3 Vision)
- LabJack U3-HV (DAC for tip-tilt mirror control)
- Tip-tilt mirror (galvanometric or piezo)

## Dependencies

- [rc_genicam_api](https://github.com/roboception/rc_genicam_api) (>= 2.0)
- LabJack driver libraries
- OpenCV >= 4.x
- Boost (system, filesystem, chrono, thread)
- libtiff

### Installing rc_genicam_api

**Option A — .deb package (recommended):**

Download the `.deb` for your distribution from [rc_genicam_api releases](https://github.com/roboception/rc_genicam_api/releases) and install:

```bash
sudo dpkg -i rc-genicam-api_*.deb
```

**Option B — build from source:**

```bash
git clone https://github.com/roboception/rc_genicam_api.git
cd rc_genicam_api
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

The `GENICAM_GENTL64_PATH` environment variable must point to the directory containing the GenTL producer `.cti` file supplied by your camera vendor's SDK/driver. The exact path depends on the vendor and installation; consult your camera's documentation. It must be set system-wide (e.g. in `~/.bashrc` or `/etc/environment`).

## Build

```bash
make
```

## Configuration

Paths are read from `~/.config/gui_tomo.conf`:

| Key | Description |
|-----|-------------|
| CHEMIN_CONFIG | Configuration directory |
| CHEMIN_ACQUIS | Hologram output directory |
| CHEMIN_RESULT | Results directory |

Acquisition parameters in `config_manip.txt`:

| Parameter | Description |
|-----------|-------------|
| DIM_ROI | Camera ROI dimension (px) |
| NB_HOLO | Number of holograms |
| SCAN_PATTERN | Scan pattern name |
| VXMIN/VXMAX/VYMIN/VYMAX | Mirror voltage range |
| NA_COND_LIM | Condenser NA limit (0-1] |

## Scan Patterns

| Pattern | Description |
|---------|-------------|
| ROSACE | Rose (rhodonea) curve |
| FERMAT | Fermat spiral with golden angle |
| ARCHIMEDE | Archimedean spiral |
| ANNULAR | Concentric circles |
| UNIFORM3D | Uniform spherical sampling |
| RANDOM_POLAR | Random polar on sphere |
| STAR | Radial star axes |

## Usage

```bash
./bin/tdm_acquisition
# or
./run.sh
```
