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

} // namespace

int main()
{
    char temp_dir[] = "/tmp/tdm_config_resolution_XXXXXX";
    assert(mkdtemp(temp_dir) != NULL);
    const string dir = temp_dir;
    assert(mkdir((dir + "/legacy_schema1").c_str(), 0700) == 0);
    check_role(dir, "acquisition.conf", "config_manip.txt");
    check_role(dir, "processing.conf", "recon.txt");
    assert(rmdir((dir + "/legacy_schema1").c_str()) == 0);
    assert(rmdir(dir.c_str()) == 0);
    return 0;
}
