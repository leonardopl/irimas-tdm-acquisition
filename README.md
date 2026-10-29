# TDM Acquisition

Hologram acquisition software for Tomographic Diffraction Microscopy (TDM).
Controls a GenICam-compatible camera and LabJack U3 (LV or HV) with LJTick-DAC for automated beam steering
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
- LabJack U3 (LV or HV) with LJTick-DAC on FIO4/FIO5
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

Within `CHEMIN_CONFIG`, acquisition parameters are read from `acquisition.conf`,
falling back to `config_manip.txt`; processing parameters (including `AMPLI_REF`)
are read from `processing.conf`, falling back to `recon.txt`. If both names for
either role exist, the program stops with an error naming both files. It prints
the selected path for each role and never searches `legacy_schema1/` backups.
If neither name exists, the existing missing-file diagnostics and defaults apply.
An inline `#` comment after a config value is ignored, and a key with no value (a bare key or blanks only) counts as
absent, as in the GUI's numeric reads. The GUI still keeps a comment in its text values, so give a text key
(`SCAN_PATTERN`, `PIXEL_FORMAT`, ...) its comment on the line above. A path is cut at a `#`: never use one in
`CHEMIN_ACQUIS`, the folder whose files the program clears before a scan.

The key-value reader ignores unused keys such as `CONFIG_SCHEMA`, `MODALITY`, and
`QWLSI_MASK_*`; an absent `CONFIG_SCHEMA` remains compatible with schema 1.
Command-line `--KEY value` overrides still take precedence, including
`--CHEMIN_CONFIG path`.

Acquisition parameters in `acquisition.conf` (or legacy `config_manip.txt`):

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
| FSM_SETTLE_MS | Delay between a DAC command and its software-triggered exposure. If omitted, the generated schedule uses 5 ms when every consecutive command step is at most 0.75 V and 95 % of them at most 0.5 V, 10 ms up to 6.1 V, otherwise 100 ms. |
| FSM_OVERLAP | 0 (default): sequential acquisition; 1: overlap the next mirror move and previous image write with the current frame transfer. Other values are errors. |
| FSM_OVERLAP_MARGIN_US | Exposure guard added before the next mirror move, in microseconds (default 500; integer range 0-100000). |
| SCAN_SEED | Optional unsigned seed used to reproduce RANDOM_POLAR schedules |

## Scan Patterns

| Pattern | Description |
|---------|-------------|
| ROSACE | Rose (rhodonea) curve |
| FERMAT | Golden-angle Fermat points, scanned in equal-area rings from the centre |
| ARCHIMEDE | Archimedean spiral |
| ANNULAR | Concentric circles |
| UNIFORM3D | Uniform spherical sampling |
| RANDOM_POLAR | Random polar on sphere |
| STAR | Radial star axes |

Since 2026-10-29, FERMAT uses unsnapped spiral points with no grid and walks
equal-area rings from the centre outwards, alternating direction in each ring.
The commands are the same at every wavelength, independent of NXMAX.
Recorded `fsm_frame_commands.csv` files keep older datasets correct by retaining
their original coordinates and voltages.

The cross-repository fixture `tests/fixtures/fermat_rings_rig_400.csv` records
400 frames at VXMIN=-3.7759, VXMAX=3.9288, VYMIN=-2.7571, VYMAX=2.5481 and
NA_COND_LIM=0.978, including the centre frame. Regenerate it from the repository
root with `make fermat-fixture`; the hardware-free scan-schedule test checks
coordinates and voltages against it within 1e-9.

Before capture, the program writes `fsm_frame_commands.csv` beside the images.
It contains one row per image (`i000` is the centre) and is the authoritative
record of normalized scan coordinates and commanded DAC voltages. Holograms are
software-triggered only after the corresponding DAC command has settled.
The automatic settle comes from a rig series (same bead at 5/10/100/300 ms,
2026-10-28): 5 ms matched 300 ms on a 400-frame `UNIFORM3D` scan (95 % of the
steps 0.40-0.46 V, at most 0.73 V), and 10 ms matched 300 ms on `FERMAT` (jumps up
to 6.1 V), where 5 ms left about 5 % of each jump of 1 V or more. Steps beyond 6.1 V were not tested and keep 100 ms; the
`(0, 8 V)` intensity-reference move is included when enabled.
An explicit `FSM_SETTLE_MS` always overrides this selection.

With `FSM_OVERLAP=0` (or unset), each frame keeps the sequential order: DAC A+B,
relative settle sleep, trigger, pop, write, return buffer. With `FSM_OVERLAP=1`,
command 0 starts the first settle before the loop. Each iteration waits until its
DAC command has settled, triggers frame k, waits until trigger return plus exposure
plus margin, commands k+1 (if any), writes and returns the held buffer for k-1
(if any), then pops and holds frame k. The last frame is written and returned after
the loop. Thus mirror movement, settling and disk writes can overlap the USB
transfer. The next trigger still waits for the current frame to arrive. There are
no worker threads, and image filenames and pixel data are unchanged.

At startup the program prints the effective mode, margin and camera exposure in
microseconds. It reads the GUI's retained exposure without changing it. Overlap
requires a readable positive exposure, `ExposureAuto` off, `SequencerMode` Off
and `TriggerDelay` zero (microseconds). Absent auto/sequencer/delay nodes are allowed;
unreadable or incompatible values warn and select the sequential loop.
When sequential mode was requested, a nonfixed exposure produces a note that slack
checks are skipped, without fallback wording. The separate
intensity-reference capture is unchanged. Trigger failures, timeouts, incomplete
buffers and image write failures return all held buffers, save diagnostics and
stop acquisition.

A failed FSM DAC write (USB transfer or I2C error) is otherwise silent: the LabJack
library only prints it and counts it in its DAC handle, which the program reads
around each write, and the program checks the I2C acknowledgements of each axis (a
NACK, e.g. a loose LJTick-DAC on a working U3, is printed but not counted). The frame loop repeats a failed write once, immediately;
`dac_start_ms` keeps the first attempt's start and `dac_done_ms` follows the repeat,
so the settle wait counts from the write that landed. A second failure stops the
scan at that image like a failed trigger (in overlap the frame before it, already
exposed, is still saved), prints `FSM DAC write failed twice at image k: stopping;
check the LabJack U3 USB connection` and the run exits non-zero. The scan-centre
move before the scan and the move to the intensity reference get the same retry and
stop the run with an error before their frames are taken; the move away after the
reference frame and the 0 V parks at exit and on Ctrl+C only warn (the reference
frame is kept; `mirror not parked at 0 V`).

After the frames (also after a failed frame) the program writes `frame_timing.csv`
beside them: schema version 2 retains the existing columns and their order, with
one row per commanded frame and the host time of each step of the frame loop in
ms from a steady-clock origin just before the first DAC command (`dac_start_ms`, `dac_done_ms`, `trigger_start_ms`,
`trigger_done_ms`, `buffer_ms` when the frame arrived, `written_ms` when its file was
written; buffer/written are -1 when unavailable), the settle, and the camera's `frame_id`,
`camera_timestamp_ns` (camera clock) and `system_timestamp_ns` (host clock at the
buffer's arrival). Two columns are appended: `exposure_start_host_ms`, the camera
timestamp mapped to the host clock, and `next_dac_slack_ms`, next DAC start minus
this frame's exposure end. They are empty without a clock mapping; slack is also
empty for the last frame or when exposure is unavailable/automatic/sequenced. On failure,
overlap may include the already commanded next frame with no received buffer, and a
DAC failure leaves its image's row with no trigger or buffer.
Older acquisitions retain their schema 1 CSV (or have no timing CSV); no reader
depends on these diagnostics. Failure to write diagnostic files only warns.

When the camera provides `TimestampLatch` and `TimestampLatchValue` (nanoseconds),
the program records a camera/host latch before and after the loop. Host times just
before and after each command bound its uncertainty. The two window midpoints
give the clock offset and linear drift. Exposure timestamps are interpreted as
exposure starts. Next-DAC slack below minus the latch uncertainty, or an exposure
starting before its own DAC completion plus settle minus that uncertainty, marks
the scan suspect. The raw minimum slack is still reported. These checks use midpoint
estimates; the latch uncertainty is reported on stdout and in the JSON. Without these nodes,
latch checks are skipped and the summary says `latch=absent`. Present nodes whose
latch or clock mapping fails report `latch=failed`. Uncertainty above 0.5 ms, or
any finite `next_dac_slack_ms` in `[-uncertainty, uncertainty)`, makes the summary suspect
with the note `latch_uncertain`: latch checks are inconclusive. This applies in both
modes. Slack below minus the uncertainty is a proven negative-slack violation and
does not itself set `latch_uncertain`. The absolute 0.5 ms cap also guards the settle
check, whose margin cannot be validated from next-DAC slack.

The summary also reports median/minimum/maximum trigger-to-exposure delays in ms,
relative to both `trigger_start_ms` (`trigger_start_delay_*_ms`) and
`trigger_done_ms` (`trigger_done_delay_*_ms`). The expected exposure start is between
those host trigger times. If the median delay from trigger start is below minus
the latch uncertainty or above the median trigger-call duration plus uncertainty
plus 1 ms, the summary is suspect and notes that the timestamp meaning is not the
exposure start, making slack checks untrustworthy.

Both modes check consecutive frame ids (including duplicates/backward ids),
report `spacing_dev_median_ms` and `spacing_dev_max_ms` as the signed median and
maximum absolute excess of camera timestamp spacing outside the allowed trigger-call
interval `[trigger_start[k] - trigger_done[k-1], trigger_done[k] - trigger_start[k-1]]`.
Excess is zero inside the interval, negative below it, and positive above it; this
check needs no clock mapping. A maximum above 0.5 ms warns. All 15 good runs on
29 Oct had zero excess. Overlap also checks every next DAC
command against trigger return plus exposure plus margin. The summary on stdout is:

```text
[frame-timing] mode=overlap frames=400 id_gaps=0 dac_retries=0 spacing_dev_max_ms=0.000000 min_slack_ms=0.500000 trigger_start_delay_median_ms=0.150000 trigger_start_delay_min_ms=0.050000 trigger_start_delay_max_ms=0.250000 trigger_done_delay_median_ms=-0.130000 trigger_done_delay_min_ms=-0.230000 trigger_done_delay_max_ms=-0.030000 latch_uncertainty_ms=0.200000 latch=ok verdict=ok
```

`frame_timing_summary.json` contains the same fields, plus the median deviation,
violation counts, affected image indices, clock offset, drift in ppm, latch
uncertainty in ms and both raw camera/host latch windows.
Its first key, `rules_version`, is the version of the verdict rules that wrote it
(`frame_timing_rules_version` in `include/frame_timing.h`; 2 since 29 Oct 2026: the
latch and spacing rules of that day and the `dac_*` keys). Summaries without it were
written under version 1 (before those rules), and a reader can re-check them:
`read_frame_timing_csv` reads `frame_timing.csv` back and `recheck_frame_timing`
runs today's check on its rows with the summary's settings and latch windows (the
GUI shows that verdict and never rewrites the file). Bump
`frame_timing_rules_version` whenever `check_frame_timing`'s verdict rules change;
the header is copied byte-identical into the GUI repository (its
`tools/check_frame_timing_sync.sh` compares them).
The JSON also records `requested_mode` (from `FSM_OVERLAP`), `mode` (actually run),
`fallback_reason` (`null` without a fallback), `settle_ms`, `exposure_us` (`null`
when unavailable or unreliable), and `margin_us`. A local invalid-exposure check
in the loop also falls back to sequential order and records its reason.
`id_gaps` counts nonconsecutive transitions; `frames` counts received buffers.
`dac_write_retries` (`dac_retries` on stdout) counts frames whose first DAC write
failed and whose retry landed, with their indices in `dac_retry_images`; a retry
alone leaves the verdict,
since the settle checks count from the repeat. `dac_write_failed` is true when a
repeat failed too, and makes the timing suspect (the scan is incomplete anyway).
Summaries written before 29 Oct 2026 lack these three keys: read them as 0, `[]`
and false.
Unavailable numbers are `null` in JSON (`na` on
stdout). A suspect verdict warns and keeps the scan; it does not change the exit
code. An available but failed/invalid clock latch also warns and marks the timing
suspect. Hardware timing and camera timestamp semantics still need validation on
the rig; hardware-free tests cover ordering, buffer cleanup and synthetic clocks.

## Usage

```bash
./bin/tdm_acquisition
# or
./run.sh
```
