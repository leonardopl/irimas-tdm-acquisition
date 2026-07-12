# TDM Acquisition

Hologram acquisition software for Tomographic Diffraction Microscopy (TDM).
Controls a GenICam-compatible camera and LabJack U3-HV DAC for automated beam steering
during off-axis holographic tomography experiments.

Adapted from the acquisition module of [MTD_transmission](https://github.com/madeba/MTD_transmission) by Matthieu Debailleul (IRIMAS, Université de Haute-Alsace). Ported from the Pleora eBUS SDK to [Aravis](https://github.com/AravisProject/aravis) for multi-vendor camera support without vendor CTI files.

## Changes from upstream

Key differences from the original [MTD_transmission](https://github.com/madeba/MTD_transmission) acquisition module:

- **SDK migration**: From Pleora eBUS SDK to Aravis; supports any GenICam-compatible camera (GigE Vision, USB3 Vision) without vendor SDK or CTI files
- **DAC settling fix**: Reference acquisition now waits for DAC settling (100 ms) before image capture, rather than after
- **Configurable pixel format**: Camera pixel format is configurable (Mono8, Mono12, Mono16) via config file, defaulting to Mono8
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
make test
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
| DIM_ROI | Camera ROI dimension (px). Set to -1 with OFFSET_ROI_X/Y to skip ROI config |
| OFFSET_ROI_X | Camera ROI X offset (px) |
| OFFSET_ROI_Y | Camera ROI Y offset (px) |
| PIXEL_FORMAT | Pixel format: Mono8, Mono12, or Mono16 (default: Mono8) |
| NB_HOLO | Number of holograms |
| SCAN_PATTERN | Scan pattern name |
| VXMIN/VXMAX/VYMIN/VYMAX | Mirror voltage range |
| NA_COND_LIM | Condenser NA limit (0-1] |
| FSM_SETTLE_MS | Delay between a DAC command and its software-triggered exposure. If omitted, the generated schedule uses 10 ms when every consecutive command step is at most 0.55 V, otherwise 100 ms. |
| SCAN_SEED | Optional unsigned seed used to reproduce RANDOM_POLAR schedules |

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

Before capture, the program writes `fsm_frame_commands.csv` beside the images.
It contains one row per image (`i000` is the centre) and is the authoritative
record of normalized scan coordinates and commanded DAC voltages. Holograms are
software-triggered only after the corresponding DAC command has settled.
The automatic 10 ms path is calibrated for the live 400-frame `UNIFORM3D`
schedule (0.530 V maximum step). Larger-step schedules retain the conservative
100 ms default; the `(0, 8 V)` intensity-reference move is included when enabled.
An explicit `FSM_SETTLE_MS` always overrides this selection.

## Usage

```bash
./bin/tdm_acquisition
# or
./run.sh
```
