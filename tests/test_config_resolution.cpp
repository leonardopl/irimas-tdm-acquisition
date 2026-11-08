#include "functions.h"
#include "acquisition_protocol.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cerrno>
#include <sys/stat.h>
#include <unistd.h>

namespace {

void write_config(const string& path, const string& contents)
{
    ofstream output(path.c_str());
    assert(output);
    output << contents;
    output.close();
    assert(output);
}

void check_role(const string& dir, const string& canonical, const string& legacy)
{
    const string canonical_path = dir + "/" + canonical;
    const string legacy_path = dir + "/" + legacy;
    string path, error;

    // Neither: preserve the legacy reader's messages and return values.
    assert(resolve_config_path(dir, canonical, legacy, path, error));
    assert(path == legacy_path);
    assert(error.empty());
    ostringstream messages, errors;
    streambuf* old_out = cout.rdbuf(messages.rdbuf());
    streambuf* old_err = cerr.rdbuf(errors.rdbuf());
    const map<string, string> no_overrides;
    const float number = get_val("DIM_ROI", path, no_overrides);
    const string text = get_string("PIXEL_FORMAT", path, no_overrides);
    cout.rdbuf(old_out);
    cerr.rdbuf(old_err);
    assert(number == 0);
    assert(text.empty());
    assert(errors.str() == "Cannot open file: " + legacy_path + "\n" +
                           "Cannot open file: " + legacy_path + "\n");
    assert(messages.str() == "Key PIXEL_FORMAT not found in file " + legacy_path + "\n");

    // Backups never satisfy resolution or create ambiguity.
    const string backup = dir + "/legacy_schema1";
    write_config(backup + "/" + canonical, "DIM_ROI 999\n");
    write_config(backup + "/" + legacy, "DIM_ROI 888\n");
    assert(resolve_config_path(dir, canonical, legacy, path, error));
    assert(path == legacy_path);
    assert(error.empty());

    // Legacy only; schema metadata is optional.
    write_config(legacy_path, "DIM_ROI 256\nPIXEL_FORMAT Mono8\nAMPLI_REF 0\n");
    assert(resolve_config_path(dir, canonical, legacy, path, error));
    assert(path == legacy_path);
    assert(error.empty());
    assert(get_val("DIM_ROI", path, no_overrides) == 256);
    assert(get_string("PIXEL_FORMAT", path, no_overrides) == "Mono8");
    assert(get_val("AMPLI_REF", path, no_overrides) == 0);

    // A canonical stat error must not silently fall back to the existing legacy file.
    assert(symlink(canonical.c_str(), canonical_path.c_str()) == 0);
    assert(!resolve_config_path(dir, canonical, legacy, path, error));
    assert(path.empty());
    assert(error == "Cannot inspect config file: " + canonical_path + ": " + strerror(ELOOP));
    assert(std::remove(canonical_path.c_str()) == 0);

    // Both: reject ambiguity and name both conflicting files.
    write_config(canonical_path,
                 "CONFIG_SCHEMA 2\nMODALITY TDM\nQWLSI_MASK_PATH mask.tif\n"
                 "QWLSI_MASK_ENABLED 1\nDIM_ROI 512\nPIXEL_FORMAT Mono16\nAMPLI_REF 1\n");
    assert(!resolve_config_path(dir, canonical, legacy, path, error));
    assert(path.empty());
    assert(error == "Ambiguous config files: " + canonical_path + " and " + legacy_path);

    // Canonical only: unknown metadata leaves values and overrides intact.
    assert(std::remove(legacy_path.c_str()) == 0);
    assert(resolve_config_path(dir, canonical, legacy, path, error));
    assert(path == canonical_path);
    assert(error.empty());
    assert(get_val("DIM_ROI", path, no_overrides) == 512);
    assert(get_string("PIXEL_FORMAT", path, no_overrides) == "Mono16");
    assert(get_val("AMPLI_REF", path, no_overrides) == 1);
    const map<string, string> overrides = {
        {"DIM_ROI", "1024"}, {"PIXEL_FORMAT", "Mono12"}, {"AMPLI_REF", "0"}
    };
    assert(get_val("DIM_ROI", path, overrides) == 1024);
    assert(get_string("PIXEL_FORMAT", path, overrides) == "Mono12");
    assert(get_val("AMPLI_REF", path, overrides) == 0);

    // A legacy stat error must also fail, even when the canonical file exists.
    assert(symlink(legacy.c_str(), legacy_path.c_str()) == 0);
    assert(!resolve_config_path(dir, canonical, legacy, path, error));
    assert(path.empty());
    assert(error == "Cannot inspect config file: " + legacy_path + ": " + strerror(ELOOP));
    assert(std::remove(legacy_path.c_str()) == 0);

    assert(std::remove(canonical_path.c_str()) == 0);
    assert(std::remove((backup + "/" + canonical).c_str()) == 0);
    assert(std::remove((backup + "/" + legacy).c_str()) == 0);
}

void check_inline_comments(const string& dir)
{
    const string path = dir + "/inline_comments.conf";
    write_config(path,
                 "\n# PIXEL_FORMAT Mono8\n \t# FSM_SETTLE_MS 99\r\n"
                 "PIXEL_FORMAT Mono12   # note\n"
                 "FSM_SETTLE_MS 10 # auto\n"
                 "SCAN_PATTERN \tFERMAT\t# pattern\n"
                 "LJ_DAC FIO45 # pins\r\n"
                 "SCAN_SEED 42 # reproducible\n"
                 "TRIMMED \t\r 31 \t\r# blanks\r\n"
                 "REPEATED 4 # first\nREPEATED 8 # last real value\n"
                 "REPEATED # nothing\nREPEATED \t\r\n"
                 "EMPTY_ONLY # nothing\nEMPTY_ONLY \t\r\n"
                 "UNCOMMENTED 17\n"
                 "INTERNAL_SPACES /tmp/acquisition with  spaces\n"
                 "RAW_SPACES  value with  spaces \t\n"
                 "CHEMIN_CONFIG /tmp/config#suffix\n"
                 "TAB_SEPARATOR\t99\n"
                 "ALIGNED_PATTERN      FERMAT\n"
                 "TRAILING_BLANKS Mono16  \t \r\n"
                 "  INDENTED\t\t12.5\n"
                 "EQUALS=7\nEQUALS_SPACED= 8\n"
                 "HASH_KEY#9\n"
                 "BARE 5\nBARE\n"
                 "LAST_VALUE 99 # no final newline");

    struct Case {
        const char* key;
        const char* text;
        float number;
    };
    const Case cases[] = {
        {"PIXEL_FORMAT", "Mono12", 0},
        {"FSM_SETTLE_MS", "10", 10},
        {"SCAN_PATTERN", "FERMAT", 0},
        {"LJ_DAC", "FIO45", 0},
        {"SCAN_SEED", "42", 42},
        {"TRIMMED", "31", 31},
        {"REPEATED", "8", 8},
        {"EMPTY_ONLY", "", 0},
        {"UNCOMMENTED", "17", 17},
        // Values are trimmed whether or not a comment follows; internal blanks stay.
        {"INTERNAL_SPACES", "/tmp/acquisition with  spaces", 0},
        {"RAW_SPACES", "value with  spaces", 0},
        {"CHEMIN_CONFIG", "/tmp/config", 0},
        // Keys are separated by spaces or tabs, after any leading blanks (aligned columns).
        {"TAB_SEPARATOR", "99", 99},
        {"ALIGNED_PATTERN", "FERMAT", 0},
        {"TRAILING_BLANKS", "Mono16", 0},
        {"INDENTED", "12.5", 12.5f},
        // '=' is not a separator (CONFIG_SCHEMA 3 forbids it): no value for EQUALS or EQUALS_SPACED.
        {"EQUALS", "", 0},
        {"EQUALS_SPACED", "", 0},
        {"EQUALS=7", "", 0},
        // A '#' ends the line, key included.
        {"HASH_KEY", "", 0},
        {"LAST_VALUE", "99", 99},
        {"MISSING", "", 0},
        // A bare key or a blank value is no value (as in the GUI's loader): the earlier value stays.
        {"BARE", "5", 5},
    };
    const map<string, string> no_overrides;
    ostringstream messages;
    streambuf* old_out = cout.rdbuf(messages.rdbuf());
    for (const Case& test : cases) {
        assert(get_string(test.key, path, no_overrides) == test.text);
        assert(get_val(test.key, path, no_overrides) == test.number);
    }
    cout.rdbuf(old_out);
    assert(messages.str().find("PIXEL_FORMAT=Mono12\n") != string::npos);
    assert(messages.str().find("FSM_SETTLE_MS=10\n") != string::npos);
    assert(messages.str().find('#') == string::npos);
    assert(messages.str().find("EMPTY_ONLY=") == string::npos);
    assert(messages.str().find("Key EMPTY_ONLY not found in file " + path + "\n") != string::npos);

    // CLI values bypass config cleanup, even when they contain blanks or '#'.
    const string pixel_value = " Mono16 # literal \t", settle_value = " 25 # literal ";
    map<string, string> overrides;
    string error;
    assert(acquisition_protocol::parseArguments({"--PIXEL_FORMAT", pixel_value, "--FSM_SETTLE_MS", settle_value},
        acquisition_protocol::ArgumentOrigin::Launch, overrides, error));
    assert(get_string("PIXEL_FORMAT", path, overrides) == pixel_value);
    assert(get_string("FSM_SETTLE_MS", path, overrides) == settle_value);
    assert(get_val("FSM_SETTLE_MS", path, overrides) == 25);

    assert(std::remove(path.c_str()) == 0);
}

// The GUI's qwlsi_geometry::pupilRadiusBins with manip's Gt (TPCAM / ((F_TUBE / F_OBJ) / RF)), written out here so
// that the derived NXMAX is checked against the GUI's formula, not against itself.
int gui_pupil_radius_bins(double na, double lambda, double tpcam, double rf, double f_obj, double f_tube, int n)
{
    const double pitch = tpcam / ((f_tube / f_obj) / rf);
    return std::max(1, std::min((int)std::round(na / lambda * pitch * n), n / 2));
}

// Returns resolve_nxmax's result; nxmax is -1 and error set on failure; printed collects stdout.
int resolve_quietly(const string& path, const map<string, string>& overrides, string& error, string& printed)
{
    ostringstream messages;
    streambuf* old_out = cout.rdbuf(messages.rdbuf());
    int nxmax = -1;
    const bool resolved = resolve_nxmax(path, overrides, nxmax, error);
    cout.rdbuf(old_out);
    printed = messages.str();
    assert(resolved == error.empty());
    return resolved ? nxmax : -1;
}

int resolve_quietly(const string& path, const map<string, string>& overrides = map<string, string>())
{
    string error, printed;
    const int nxmax = resolve_quietly(path, overrides, error, printed);
    assert(nxmax > 0);
    return nxmax;
}

void check_derived_nxmax(const string& dir)
{
    const string path = dir + "/acquisition.conf";
    // The live optics (CONFIG_SCHEMA 3: no NXMAX), aligned and commented as a hand edit may be.
    const string optics =
        "CONFIG_SCHEMA 3\nMODALITY QWLSI\n"
        "NA      1.3        # objective\n"
        "TPCAM\t3.45e-6\nRF 0.5\nF_OBJ 0.0018\nF_TUBE 0.18\n";
    const string lambda = "LAMBDA 6.330000e-07\n", dim = "DIM_ROI 2048\n";
    write_config(path, optics + lambda + dim);
    string error, printed;
    assert(resolve_quietly(path, map<string, string>(), error, printed) == 73);
    assert(printed.find("Key NXMAX not found in file " + path + "\n") != string::npos);
    assert(printed.find("NXMAX not set: derived 73 = round(NA (no CROP_NA) 1.3 / LAMBDA 6.33e-07 x pitch 1.725e-08 "
                        "x DIM_ROI 2048) clamped to [1, DIM_ROI / 2], pitch = TPCAM 3.45e-06 / ((F_TUBE 0.18 / "
                        "F_OBJ 0.0018) / RF 0.5)\n") != string::npos);

    // The GUI passes the run's wavelength and ROI on the command line: they override the file as for every key.
    const map<string, string> blue = {{"LAMBDA", "4.88e-07"}};
    assert(resolve_quietly(path, blue) == 94);
    assert(resolve_quietly(path, {{"LAMBDA", "5.32e-07"}}) == 86);
    assert(resolve_quietly(path, {{"LAMBDA", "4.5e-07"}}) == 102);
    assert(resolve_quietly(path, {{"DIM_ROI", "1024"}}) == 36);
    // The same NXMAX as the GUI's formula from 400 to 800 nm on three ROIs.
    for (int nm = 400; nm <= 800; ++nm)
        for (int roi : {256, 1024, 2048}) {
            char text[32];
            snprintf(text, sizeof text, "%.9g", nm * 1e-9);
            const map<string, string> overrides = {{"LAMBDA", text}, {"DIM_ROI", to_string(roi)}};
            assert(resolve_quietly(path, overrides) ==
                   gui_pupil_radius_bins(1.3, strtod(text, NULL), 3.45e-6, 0.5, 0.0018, 0.18, roi));
        }
    // Clamped to [1, DIM_ROI / 2].
    assert(resolve_quietly(path, {{"TPCAM", "1e-4"}}) == 1024);
    assert(resolve_quietly(path, {{"NA", "0.001"}}) == 1);

    // CROP_NA (absent: NA) is the crop: Matthieu's off-axis sets crop 1.2 of an objective's NA.
    write_config(path, optics + lambda + dim + "CROP_NA   1.2   # crop\n");
    assert(resolve_quietly(path, map<string, string>(), error, printed) == 67);
    assert(printed.find("derived 67 = round(CROP_NA 1.2 / LAMBDA") != string::npos);
    assert(resolve_quietly(path, blue) == gui_pupil_radius_bins(1.2, 4.88e-7, 3.45e-6, 0.5, 0.0018, 0.18, 2048));
    assert(resolve_quietly(path, {{"CROP_NA", "1.3"}}) == 73);
    // A CROP_NA with only a comment is absent.
    write_config(path, optics + lambda + dim + "CROP_NA # none\n");
    assert(resolve_quietly(path) == 73);

    // An NXMAX setting or --NXMAX is used as before, with no derivation.
    write_config(path, optics + lambda + dim + "NXMAX 80\n");
    assert(resolve_quietly(path, map<string, string>(), error, printed) == 80);
    assert(printed == "NXMAX=80\n");
    assert(resolve_quietly(path, {{"NXMAX", "94"}}) == 94);
    write_config(path, optics + lambda + dim);
    assert(resolve_quietly(path, {{"NXMAX", "74"}}, error, printed) == 74);
    assert(printed == "NXMAX=74 (from args)\n");

    // A missing or non-positive input stops the run with the reason.
    struct Refusal {
        const char* config;
        const char* reason;
    };
    const string no_f_obj = "CONFIG_SCHEMA 3\nNA 1.3\nTPCAM 3.45e-6\nRF 0.5\nF_TUBE 0.18\n";
    const Refusal refusals[] = {
        {"CONFIG_SCHEMA 3\nTPCAM 3.45e-6\nRF 0.5\nF_OBJ 0.0018\nF_TUBE 0.18\nLAMBDA 6.33e-7\nDIM_ROI 2048\n",
         "NA is missing"},
        {"CONFIG_SCHEMA 3\nNA 1.3\nTPCAM 3.45e-6\nRF 0.5\nF_TUBE 0.18\nLAMBDA 6.33e-7\nDIM_ROI 2048\n",
         "F_OBJ is missing"},
        {"NA 1.3\nTPCAM 3.45e-6\nRF 0\nF_OBJ 0.0018\nF_TUBE 0.18\nLAMBDA 6.33e-7\nDIM_ROI 2048\n",
         "RF is not a positive number"},
        {"NA 1.3\nTPCAM 3.45e-6\nRF 0.5\nF_OBJ 0.0018\nF_TUBE 0.18\nLAMBDA 633nm\nDIM_ROI 2048\n",
         "LAMBDA is not a positive number"},
        {"NA 1.3\nCROP_NA -1.2\nTPCAM 3.45e-6\nRF 0.5\nF_OBJ 0.0018\nF_TUBE 0.18\nLAMBDA 6.33e-7\nDIM_ROI 2048\n",
         "CROP_NA is not a positive number"},
        {"NA 1.3\nTPCAM 3.45e-6\nRF 0.5\nF_OBJ 0.0018\nF_TUBE 0.18\nLAMBDA 6.33e-7\nDIM_ROI -1\n",
         "DIM_ROI is not a positive number"},
        {"NA 1.3\nTPCAM 3.45e-6\nRF 0.5\nF_OBJ 0.0018\nF_TUBE 0.18\nLAMBDA 6.33e-7\nDIM_ROI 2048.5\n",
         "DIM_ROI is not a whole number of pixels"},
        {"NA 1.3\nTPCAM 3.45e-6\nRF 0.5\nF_OBJ 0.0018\nF_TUBE 0.18\nLAMBDA nan\nDIM_ROI 2048\n",
         "LAMBDA is not a positive number"},
    };
    for (const Refusal& refusal : refusals) {
        write_config(path, refusal.config);
        assert(resolve_quietly(path, map<string, string>(), error, printed) == -1);
        assert(error.find("NXMAX is not set (no NXMAX in " + path + ", no --NXMAX) and cannot be derived: " +
                          refusal.reason + " (NXMAX = round(CROP_NA (absent: NA)") == 0);
        assert(printed.find("NXMAX not set: derived") == string::npos);
    }
    // An override can supply the missing input.
    write_config(path, no_f_obj + lambda + dim);
    assert(resolve_quietly(path, {{"F_OBJ", "0.0018"}}) == 73);

    assert(std::remove(path.c_str()) == 0);
}

void check_ampli_ref_source(const string& dir)
{
    const string acquisition = dir + "/acquisition.conf", processing = dir + "/processing.conf";
    const map<string, string> no_overrides;
    ostringstream messages;
    streambuf* old_out = cout.rdbuf(messages.rdbuf());
    write_config(processing, "AMPLI_REF 0\n");
    // CONFIG_SCHEMA 3: acquisition.conf only (B213 D4).
    for (const char* schema : {"CONFIG_SCHEMA 3\n", "CONFIG_SCHEMA   3   # current\n", "  CONFIG_SCHEMA\t3\r\n"}) {
        write_config(acquisition, string(schema) + "AMPLI_REF 1\n");
        assert(ampli_ref_config_path(acquisition, processing) == acquisition);
        assert(get_val("AMPLI_REF", ampli_ref_config_path(acquisition, processing), no_overrides) == 1);
    }
    assert(messages.str().find("AMPLI_REF from " + acquisition + " (CONFIG_SCHEMA 3)\n") != string::npos);
    // ... where an absent AMPLI_REF is absent, whatever processing.conf says.
    write_config(processing, "AMPLI_REF 1\n");
    write_config(acquisition, "CONFIG_SCHEMA 3\n");
    assert(get_val("AMPLI_REF", ampli_ref_config_path(acquisition, processing), no_overrides) == 0);
    // Schemas 1 and 2: processing.conf, as before.
    write_config(processing, "AMPLI_REF 0\n");
    for (const char* schema : {"", "CONFIG_SCHEMA 2\n", "CONFIG_SCHEMA 1\n", "CONFIG_SCHEMA # none\n"}) {
        write_config(acquisition, string(schema) + "AMPLI_REF 1\n");
        assert(ampli_ref_config_path(acquisition, processing) == processing);
        assert(get_val("AMPLI_REF", ampli_ref_config_path(acquisition, processing), no_overrides) == 0);
    }
    // --AMPLI_REF overrides either file.
    const map<string, string> overrides = {{"AMPLI_REF", "1"}};
    assert(get_val("AMPLI_REF", ampli_ref_config_path(acquisition, processing), overrides) == 1);
    write_config(acquisition, "CONFIG_SCHEMA 3\nAMPLI_REF 1\n");
    assert(get_val("AMPLI_REF", ampli_ref_config_path(acquisition, processing), {{"AMPLI_REF", "0"}}) == 0);
    cout.rdbuf(old_out);
    assert(std::remove(acquisition.c_str()) == 0);
    assert(std::remove(processing.c_str()) == 0);
}

} // namespace

int main()
{
    char temp_dir[] = "/tmp/tdm_config_resolution_XXXXXX";
    assert(mkdtemp(temp_dir) != NULL);
    const string dir = temp_dir;
    assert(mkdir((dir + "/legacy_schema1").c_str(), 0700) == 0);
    check_role(dir, "acquisition.conf", "config_manip.txt");
    check_role(dir, "processing.conf", "recon.txt");
    check_inline_comments(dir);
    check_derived_nxmax(dir);
    check_ampli_ref_source(dir);
    assert(rmdir((dir + "/legacy_schema1").c_str()) == 0);
    assert(rmdir(dir.c_str()) == 0);
    return 0;
}
