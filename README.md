# TDM Acquisition

Hologram acquisition software for Tomographic Diffraction Microscopy (TDM).
Controls a Basler camera and LabJack U3-HV DAC for automated beam steering
during off-axis holographic tomography experiments.

Developed at IRIMAS, Universite de Haute-Alsace.

## Hardware

- Basler area-scan camera (GigE or USB3)
- LabJack U3-HV (DAC for tip-tilt mirror control)
- Tip-tilt mirror (galvanometric or piezo)

## Dependencies

- [Basler Pylon SDK](https://www.baslerweb.com/en/downloads/software-downloads/) (>= 6.x)
- LabJack driver libraries
- OpenCV >= 4.x
- Boost (system, filesystem, chrono, thread)
- libtiff

## Build

```bash
source setup_env.sh
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

## License

[TBD]
