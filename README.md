# TDM Acquisition

Hologram acquisition software for Tomographic Diffraction Microscopy (TDM).
Controls a GenICam-compatible camera and LabJack U3-HV DAC for automated beam steering
during off-axis holographic tomography experiments.

Adapted from the acquisition module of [MTD_transmission](https://github.com/madeba/MTD_transmission) by Matthieu Debailleul (IRIMAS, Université de Haute-Alsace). Ported from the Pleora eBUS SDK to [Aravis](https://github.com/AravisProject/aravis) for multi-vendor camera support without vendor CTI files.

## Changes from upstream

Key differences from the original [MTD_transmission](https://github.com/madeba/MTD_transmission) acquisition module:

- **SDK migration**: From Pleora eBUS SDK to Aravis; supports any GenICam-compatible camera (GigE Vision, USB3 Vision) without vendor SDK or CTI files
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

- [Aravis](https://github.com/AravisProject/aravis) (0.8.x)
- LabJack driver libraries
- OpenCV >= 4.x
- libtiff

### Installing Aravis

On Ubuntu/Debian:

```bash
sudo apt install libaravis-dev
```

On other distributions, or to build from source, see the [Aravis documentation](https://aravisproject.github.io/aravis/aravis-stable/building.html).

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
