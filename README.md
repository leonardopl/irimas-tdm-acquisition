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
- **Signal handling**: SIGINT (Ctrl+C) and SIGTERM request cooperative shutdown, restoring free run and parking the DAC in normal program context
- **Error handling**: Structured exception handling with proper camera cleanup on failure
- **Codebase cleanup**: Translated from French to English; removed unused functions and dead code

## Hardware

- GenICam-compatible area-scan camera (GigE Vision or USB3 Vision)
- LabJack U3 (LV or HV) with LJTick-DAC on FIO4/FIO5
- Tip-tilt mirror (galvanometric or piezo)

Cameras are looked for on USB3 Vision first; GigE Vision cameras are found when no USB3 Vision camera is connected, or when the requested `--CAMERA_ID` is absent from the USB3 list (Aravis's GigE discovery waits about a second on every search). `--CAMERA_ID identity` matches the serial number or Aravis device id, and never substitutes another camera. Without that option, CLI mode prompts when several cameras are found; session mode requires an identity unless exactly one camera is found.

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
falling back to `config_manip.txt`; processing parameters (including `AMPLI_REF`
before CONFIG_SCHEMA 3, below) are read from `processing.conf`, falling back to `recon.txt`. If both names for
either role exist, the program stops with an error naming both files. It prints
the selected path for each role and never searches `legacy_schema1/` backups.
If neither name exists, the existing missing-file diagnostics and defaults apply.
A config line is `KEY value`: the key is the line's first word (leading blanks allowed), separated from the value by
spaces or tabs (aligned columns work); the value is the rest of the line up to a `#`, without the blanks around it
(internal ones kept). `KEY=value` is not read. A key with no value (a bare key, blanks or a comment only) counts as
absent, as in the GUI's numeric reads. The GUI still keeps a comment in its text values, so give a text key
(`SCAN_PATTERN`, `PIXEL_FORMAT`, ...) its comment on the line above. A path is cut at a `#`: never use one in
`CHEMIN_ACQUIS`, the output folder where the program deletes only its earlier acquisition files before a scan.

The key-value reader ignores unused keys such as `MODALITY` and `QWLSI_MASK_*`; `CONFIG_SCHEMA` only chooses the
`AMPLI_REF` file, and an absent `CONFIG_SCHEMA` remains compatible with schema 1.
Command-line `--KEY value` overrides still take precedence, including
`--CHEMIN_CONFIG path`.

CONFIG_SCHEMA 3 (the GUI's B213/B211 configs): `NXMAX` is no longer a setting. Without an `NXMAX` line or `--NXMAX`,
the program derives it as the GUI does, `round(CROP_NA / LAMBDA * TPCAM / Gt * DIM_ROI)` clamped to [1, DIM_ROI/2],
with `CROP_NA` absent meaning `NA` and Gt = (F_TUBE / F_OBJ) / RF (each input `--KEY` overridable; 73 at 633 nm and
94 at 488 nm with the 2048 ROI and the live optics), prints it with its inputs, and stops before anything moves when
an input is missing or not positive. An existing `NXMAX` or `--NXMAX` is used as before. `AMPLI_REF` is read from
`acquisition.conf` when it says `CONFIG_SCHEMA 3`, from `processing.conf` otherwise (`--AMPLI_REF` overrides both).

Acquisition parameters in `acquisition.conf` (or legacy `config_manip.txt`):

| Parameter | Description |
|-----------|-------------|
| DIM_ROI | Camera ROI dimension (px). Set to -1 with OFFSET_ROI_X/Y to skip ROI config |
| OFFSET_ROI_X | Camera ROI X offset (px) |
| OFFSET_ROI_Y | Camera ROI Y offset (px) |
| PIXEL_FORMAT | Pixel format: Mono8, Mono12, or Mono16 (default: Mono8) |
| NB_HOLO | Number of holograms |
| NXMAX | Crop half-width in Fourier bins; RANDOM_POLAR snaps to its 2 NXMAX grid. Absent: derived (CONFIG_SCHEMA 3, above) |
| SCAN_PATTERN | Scan pattern name |
| VXMIN/VXMAX/VYMIN/VYMAX | Mirror voltage range |
| NA_COND_LIM | Condenser NA limit (0-1] |
| FSM_SETTLE_MS | Delay between a DAC command and its software-triggered exposure. If omitted, the generated schedule uses 5 ms when every consecutive command step is at most 0.75 V and 95 % of them at most 0.5 V, 10 ms up to 6.1 V, otherwise 100 ms. |
| FSM_OVERLAP | 0 (default): sequential acquisition; 1: overlap the next mirror move and previous image write with the current frame transfer. Other values are errors. |
| FSM_OVERLAP_MARGIN_US | Exposure guard added before the next mirror move, in microseconds (default 500; integer range 0-100000). |
| SCAN_SEED | Optional unsigned seed used to reproduce RANDOM_POLAR schedules |
| FSM_SCHEDULE | Command line only (`--FSM_SCHEDULE path`, the GUI's one-shot probe field): an FSM schedule (below) that replaces the scan pattern and NB_HOLO for that run. A key in acquisition.conf is ignored. |

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
`fallback_reason` (`null` without a fallback), `settle_ms` (`null` with
`settle_ms_min`/`settle_ms_max` for an FSM schedule), `exposure_us` (`null`
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

## FSM schedule (settle probe)

`--FSM_SCHEDULE path` (command line only; a config key is ignored) takes the frames at a list
of mirror commands, each with its own settle time, instead of a generated scan
pattern: the FSM settle probe (qwlsi-gui B199) writes such a schedule. The file is
CSV (UTF-8) with the header `vx_v,vy_v,settle_ms,label` and one row per frame, in
order: the command in volts, the settle after it in whole milliseconds (0 to 1000)
and a label without commas (may be empty). Lines starting with `#` and blank lines
are skipped, also after the header and between rows.

```text
# 2 V steps, a reference frame after a long wait
vx_v,vy_v,settle_ms,label
0.075,-0.102,1000,reference
2.075,-0.102,2,step 2 V settle 2 ms
```

The whole file is read and checked before earlier acquisition files are removed and before
the mirror moves: 1 to 4000 rows, every voltage finite and inside `VXMIN..VXMAX` and
`VYMIN..VYMAX` (1 µV of float rounding allowed; such a value is clamped onto the
limit), every settle an integer from 0 to 1000, valid limits and `NA_COND_LIM`.
Any error stops the run with exit code 1 and one stderr line naming the file's
line (`Invalid FSM schedule <path>: line 7: vy_v 9 V is outside VYMIN..VYMAX
(-2.7571..2.5481 V); nothing moved`); nothing moves and no frame is taken.

The row count replaces `NB_HOLO` (stdout says so) and `SCAN_PATTERN`, `SCAN_SEED`,
`NB_CIRCLES_ANNULAR` and `FSM_SETTLE_MS` are not used. The mirror moves to the
scan centre first, as for a scan, then frame k goes to row k's command and sleeps
row k's settle after that DAC write before its trigger. The loop is always
sequential: in overlap the trigger also waits for the previous frame's transfer,
so a short settle (the probe measures 2-20 ms) would not be the real wait. With
`FSM_OVERLAP=1` stdout says so, and the summary has `mode` sequential,
`requested_mode` overlap and the schedule as `fallback_reason`. No intensity
reference is taken (`AMPLI_REF` is ignored), so only the schedule's commands, the
scan centre and the 0 V parks move the mirror.

`fsm_frame_commands.csv` keeps its schema and pairs each frame with its command:
row k has `image_index` and `scan_command_index` k (a scan's row k has k - 1, so
readers that expect a scan refuse the table), the row's voltages and the
normalized coordinates a scan command at those voltages would have. A copy of the
schedule, byte for byte as checked, is saved as `fsm_schedule.csv` beside the
frames. `frame_timing.csv` records each frame's own settle in its `settle_ms`
column, and the settle check (unsettled frames) uses it. The summary then has
`"settle_ms": null` with `"settle_ms_min"` and `"settle_ms_max"`, the range of
the frames' settles; a scan's summary keeps `"settle_ms"` and has neither key.
`read_frame_timing_csv` gives every row its own settle when the column changes
from row to row (and -1 for the file's settle). The verdict rules are unchanged
(`rules_version` 2): with one settle the check is the same.

The GUI's acquisition panel starts such a run once (Advanced > FSM schedule) and
records `SCAN_PATTERN SCHEDULE` and the schedule's file name in the run's config;
it never processes the run as a tomography scan.

## Usage

```bash
./bin/tdm_acquisition --help  # or -h; prints all keys without reading config or using hardware
./bin/tdm_acquisition
# or
./run.sh
```

One acquisition uses the camera, the FSM DAC and the config named by
`~/.config/gui_tomo.conf`, and writes to `CHEMIN_ACQUIS` (default from that file).
Overrides must be known `--KEY VALUE` pairs, once per key; `--KEY=VALUE`, bare
words, unknown keys, duplicate keys and missing values are usage errors.
Values are kept verbatim, including blanks, `#`, negative numbers and empty strings;
a value starting with `--` counts as missing. `--help` or `-h` anywhere takes
precedence over all other arguments. Exit codes: 0 success/help, 1 refused/stopped,
2 usage error, 255 failure.

Before each run, cleanup deletes only regular files or symlinks named `i` followed
by at least three ASCII digits and `.pgm`, `Intensite_ref.pgm`,
`fsm_frame_commands.csv`, `fsm_frame_commands.csv.tmp`, `fsm_schedule.csv`,
`fsm_schedule.csv.tmp`, `frame_timing.csv` or `frame_timing_summary.json`.
Other entries, including configs, hidden files and directories, are kept. One
stderr warning lists the kept entries in sorted order, with `/` after directories,
up to 20 names then a count of the rest. Cleanup never follows symlinks or recurses.
Archived frames named `i` plus at least three digits and `.png` or `.jxl` refuse the
run without deleting anything; choose an empty folder. So does an entry with one of
the acquisition output names above that is not a regular file or symlink (a
directory, a FIFO), since the run would write that name. Failure to remove an
earlier acquisition file
also refuses the run before hardware use; cleanup may already have removed other
acquisition files.

## Session mode

`--SESSION 1` opens one camera and one LabJack connection for multiple acquisitions.
Session launch accepts only `SESSION` (exactly `1`) and connection keys:
`CHEMIN_CONFIG`, `CAMERA_ID`, `DIM_ROI`, `OFFSET_ROI_X`, `OFFSET_ROI_Y`, `PIXEL_FORMAT`.
Run keys belong in `RUN` lines; passing one at session launch is a usage error.
A usage error at a session launch also replies `ERROR message=...` on stdout and exits 2.
Configuration files and `--KEY VALUE` overrides resolve as for the CLI. For example:

```bash
./bin/tdm_acquisition --SESSION 1 --CAMERA_ID 12345678 \
  --DIM_ROI 1024 --OFFSET_ROI_X 0 --OFFSET_ROI_Y 0 --PIXEL_FORMAT Mono8
```

`--CHEMIN_CONFIG dir` remains optional. `--CAMERA_ID` accepts a serial number or
Aravis device id; without it, exactly one discovered camera is required. Session
mode never reads stdin for a camera choice. The program configures ROI, format
and software triggering once, then sends `HELLO protocol=1 camera=... serial=...
model=... vendor=... roi=1024,0,0 format=Mono8 exposure_us=...`, or `ERROR message=...` and exits
nonzero on startup failure. `exposure_us` reads the configured camera's current
exposure in microseconds with 17 significant digits, as in the EXPOSURE reply;
if this read fails, HELLO is still sent with `exposure_us=nan`.
Startup does not clear an acquisition folder or move
the mirror. Human-readable diagnostics go to stderr; stdout contains only protocol replies.
The binary embeds `TDM_SESSION_PROTOCOL=1` for detection without launching hardware.

The shared header `include/acquisition_protocol.h` (namespace `acquisition_protocol`)
defines the acquisition program line protocol.

Commands and replies are newline-terminated UTF-8 byte strings. A nonempty word
without spaces, tabs, quotes or backslashes is bare; otherwise use double quotes,
escaping only `"` and `\` as `\"` and `\\`. Empty strings are `""`. CR/LF and
other control characters (except literal tabs) are forbidden inside words. CRLF
line endings are accepted; incomplete lines wait for their newline.

| Command | Reply and effect |
|---|---|
| `EXPOSURE 1234.6` | Turns exposure auto off when supported, writes microseconds, then reads back: `EXPOSURE us=1234 auto=0 error=""`. A failure still replies with read-back values and an error. An absent ExposureAuto node counts as off. |
| `RUN --CHEMIN_ACQUIS "/data/run 1" --NXMAX 128 --NB_HOLO 400` | Accepts the CLI's `--KEY value` pairs, resolving configuration anew. Replies `DONE status=0 frames=400 error=""`; frames counts saved holograms, excluding the optional intensity reference. |
| `QUIT` | Restores camera free run, releases the camera, parks the mirror at 0 V and closes the LabJack, then replies `BYE` and exits 0. |

Each RUN must resolve the session's actual DIM_ROI, OFFSET_ROI_X, OFFSET_ROI_Y and
PIXEL_FORMAT. A mismatch replies `DONE status=1 frames=0 error=...` before clearing
files or moving the mirror. Each accepted RUN removes only its earlier acquisition
outputs under the cleanup rule above, keeps and warns about other entries, and
starts fresh frame numbering, schedule, stream buffers, clock origin and timing
state, and writes the same files as a CLI acquisition. Exposure is read again per
RUN for the fixed-exposure/overlap checks. The connection remains open between RUNs;
there is no end-of-run park until session cleanup. Invalid configuration and scan
schedules are validated before clearing or moving in both modes.

Empty lines are ignored. Malformed lines and unknown commands receive `ERROR
message=...` and leave the session usable. RUN uses the same strict parser:
unknown or duplicate keys, missing values and `--SESSION` receive `ERROR` before
config resolution, cleanup or motion. A valid RUN can follow a refused line.
Commands received during RUN are queued
until its DONE. A failed acquisition returns its existing process status in DONE,
mapped to the shell's exit value (`return_value & 0xff`: -1 becomes 255, 256 becomes 0).
Failure of one RUN leaves the session available for QUIT or another RUN.

In both modes SIGINT and SIGTERM only set a stop flag. The frame loop saves the
current frame, returns its buffers, stops its stream and writes partial timing;
the reference capture checks the flag between steps. Normal cleanup restores free
run, closes the camera, parks at 0 V and releases the LabJack. A stopped run exits
with status 1, matching the former SIGINT behavior. The existing 5 s buffer wait
and the current frame's exposure/settle can delay the stop.

In session mode stdin EOF, including an incomplete last line, requests the same
stop; that incomplete line is never executed. EOF while idle cleans up and exits 0
without BYE. EOF or a signal during RUN produces DONE with status 1, cleans up and
exits 1. SIGPIPE is ignored so a disconnected reply pipe cannot bypass cleanup.
Only SIGKILL bypasses cooperative cleanup.

`make test` includes the shared-header tests, cooperative-stop frame-loop tests,
a binary marker check, and session/CLI integration tests with only the Aravis Fake
interface enabled and a simulated LabJack. The integration test isolates HOME/XDG
configuration in a temporary directory and skips explicitly with status 77 only
if the Fake interface is missing. Fake lacks ExposureAuto and TimestampLatch nodes;
their absent-node behavior is tested. Actual hardware timing still needs the rig.
