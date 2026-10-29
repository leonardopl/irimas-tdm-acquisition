#include "functions.h"

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
        {"INTERNAL_SPACES", "/tmp/acquisition with  spaces", 0},
        {"RAW_SPACES", " value with  spaces \t", 0},
        {"CHEMIN_CONFIG", "/tmp/config", 0},
        {"TAB_SEPARATOR", "", 0},
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
    char program[] = "test_config_resolution";
    char pixel_key[] = "--PIXEL_FORMAT";
    char pixel_value[] = " Mono16 # literal \t";
    char settle_key[] = "--FSM_SETTLE_MS";
    char settle_value[] = " 25 # literal ";
    char* argv[] = {program, pixel_key, pixel_value, settle_key, settle_value};
    const map<string, string> overrides = parse_args(5, argv);
    assert(get_string("PIXEL_FORMAT", path, overrides) == pixel_value);
    assert(get_string("FSM_SETTLE_MS", path, overrides) == settle_value);
    assert(get_val("FSM_SETTLE_MS", path, overrides) == 25);

    assert(std::remove(path.c_str()) == 0);
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
    assert(rmdir((dir + "/legacy_schema1").c_str()) == 0);
    assert(rmdir(dir.c_str()) == 0);
    return 0;
}
