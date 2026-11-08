#ifndef __FUNCTIONS__
#define __FUNCTIONS__

#include <time.h>
#include <math.h>
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <assert.h>
#include <vector>
#include <map>

using namespace std;

#include "projet.h"
#include "run_folder.h"

// If neither name exists, keep the legacy path and its existing reader diagnostics.
bool resolve_config_path(const string& config_dir, const string& canonical_name,
                         const string& legacy_name, string& resolved_path,
                         string& error);

typedef struct{
clock_t start, end;
float   total;
}timer_info;

float extract_val(string token,  string file_path);
float write_val(string token, float token_value, string file_path);
string extract_string(string token,  string file_path);
float get_val(const string& key, const string& file_path, const map<string, string>& overrides);
string get_string(const string& key, const string& file_path, const map<string, string>& overrides);
// get_val in double precision: false when the key has no value (value untouched); a value that is not entirely a
// finite number reads as NaN.
bool get_double(const string& key, const string& file_path, const map<string, string>& overrides, double& value);
// The run's NXMAX: --NXMAX or the config's NXMAX as before; when neither is set (CONFIG_SCHEMA 3 has no NXMAX, B211),
// derived as the GUI does: round(crop NA / LAMBDA x TPCAM / Gt x DIM_ROI) clamped to [1, DIM_ROI / 2], crop NA =
// CROP_NA (absent: NA), Gt = (F_TUBE / F_OBJ) / RF. Prints the derivation; false with error when an input is missing
// or not positive.
bool resolve_nxmax(const string& file_path, const map<string, string>& overrides, int& nxmax, string& error);
// The file AMPLI_REF is read from: the acquisition config when it is CONFIG_SCHEMA 3 (AMPLI_REF lives only there),
// else the processing config as before (schemas 1 and 2 carry it in both files).
string ampli_ref_config_path(const string& acquisition_path, const string& processing_path);
bool is_acquisition_output(const string& name);
// Read the whole listing before deleting only regular files/symlinks with own-output names.
// Archived frames and own-output names that are not regular files/symlinks refuse without deletions.
// Other entries are returned sorted (directories end in '/').
// Uses the caller's open directory. Returns 1 on list/read/close/inspect/unlink failure, naming the folder/entry.
// Unlink may partially clear.
int clear_acquisition(const RunFolder& run_folder, vector<string>& left, string& error);
#endif
