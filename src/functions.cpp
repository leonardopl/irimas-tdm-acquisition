#include "functions.h"
#include <tiffio.h>
#include <dirent.h>
#include <algorithm>
#include <chrono>
#include <cerrno>
#include <climits>
#include <cmath>
#include <iomanip>
#include <sys/stat.h>
#define PI 3.14159

bool resolve_config_path(const string& config_dir, const string& canonical_name,
                         const string& legacy_name, string& resolved_path,
                         string& error)
{
    const string canonical_path = config_dir + "/" + canonical_name;
    const string legacy_path = config_dir + "/" + legacy_name;
    resolved_path.clear();
    error.clear();
    struct stat info;
    const bool canonical_exists = stat(canonical_path.c_str(), &info) == 0;
    if (!canonical_exists)
    {
        const int stat_error = errno;
        if (stat_error != ENOENT && stat_error != ENOTDIR)
        {
            error = "Cannot inspect config file: " + canonical_path + ": " + strerror(stat_error);
            return false;
        }
    }
    const bool legacy_exists = stat(legacy_path.c_str(), &info) == 0;
    if (!legacy_exists)
    {
        const int stat_error = errno;
        if (stat_error != ENOENT && stat_error != ENOTDIR)
        {
            error = "Cannot inspect config file: " + legacy_path + ": " + strerror(stat_error);
            return false;
        }
    }
    if (canonical_exists && legacy_exists)
    {
        error = "Ambiguous config files: " + canonical_path + " and " + legacy_path;
        return false;
    }
    resolved_path = canonical_exists ? canonical_path : legacy_path;
    return true;
}

map<string, string> parse_args(int argc, char *argv[])
{
    map<string, string> overrides;
    for (int i = 1; i < argc - 1; i += 2)
    {
        string arg = argv[i];
        if (arg.substr(0, 2) == "--")
        {
            overrides[arg.substr(2)] = argv[i + 1];
        }
    }
    return overrides;
}

float get_val(const string& key, const string& file_path, const map<string, string>& overrides)
{
    auto it = overrides.find(key);
    if (it != overrides.end())
    {
        cout << key << "=" << it->second << " (from args)" << endl;
        return atof(it->second.c_str());
    }
    return extract_val(key, file_path);
}

string get_string(const string& key, const string& file_path, const map<string, string>& overrides)
{
    auto it = overrides.find(key);
    if (it != overrides.end())
    {
        cout << key << "=" << it->second << " (from args)" << endl;
        return it->second;
    }
    return extract_string(key, file_path);
}

namespace {

// The whole text as a finite number (blanks around it allowed), else NaN: get_val's atof would read "1.3abc" as 1.3
// and "abc" as 0.
double parse_number(const string& text)
{
    const char* begin = text.c_str();
    char* end = NULL;
    const double number = strtod(begin, &end);
    if (end == begin)
        return NAN;
    while (*end == ' ' || *end == '\t' || *end == '\r')
        ++end;
    return *end == '\0' && std::isfinite(number) ? number : NAN;
}

} // namespace

bool get_double(const string& key, const string& file_path, const map<string, string>& overrides, double& value)
{
    const string text = get_string(key, file_path, overrides);
    if (text.empty())
        return false;
    value = parse_number(text);
    return true;
}

bool resolve_nxmax(const string& file_path, const map<string, string>& overrides, int& nxmax, string& error)
{
    error.clear();
    const string configured = get_string("NXMAX", file_path, overrides);
    if (!configured.empty())
    {
        nxmax = static_cast<int>(static_cast<float>(atof(configured.c_str())));  // get_val's float, as before
        return true;
    }

    auto fail = [&](const string& reason) -> bool {
        error = "NXMAX is not set (no NXMAX in " + file_path + ", no --NXMAX) and cannot be derived: " + reason +
                " (NXMAX = round(CROP_NA (absent: NA) / LAMBDA x TPCAM / Gt x DIM_ROI), Gt = (F_TUBE / F_OBJ) / RF)";
        return false;
    };
    auto positive = [&](const char* key, bool present, double value) -> bool {
        if (!present)
            return fail(string(key) + " is missing");
        return value > 0 || fail(string(key) + " is not a positive number");
    };
    auto read_positive = [&](const char* key, double& value) -> bool {
        const bool present = get_double(key, file_path, overrides, value);
        return positive(key, present, value);
    };
    double crop_na = 0, lambda = 0, tpcam = 0, rf = 0, f_obj = 0, f_tube = 0, dim = 0;
    const bool crop_na_set = get_double("CROP_NA", file_path, overrides, crop_na);
    if (!(crop_na_set ? positive("CROP_NA", true, crop_na) : read_positive("NA", crop_na)) ||
        !read_positive("LAMBDA", lambda) || !read_positive("TPCAM", tpcam) || !read_positive("RF", rf) ||
        !read_positive("F_OBJ", f_obj) || !read_positive("F_TUBE", f_tube) || !read_positive("DIM_ROI", dim))
        return false;
    if (dim != std::floor(dim) || dim > INT_MAX)
        return fail("DIM_ROI is not a whole number of pixels");
    // The GUI's qwlsi_geometry::pupilRadiusBins with manip's Gt, in the same order of operations, so that both
    // programs derive the same NXMAX; clamped before the int conversion (same result).
    const double gt = (f_tube / f_obj) / rf;
    const double pitch = tpcam / gt;
    if (!(pitch > 0) || !std::isfinite(pitch))
        return fail("the hologram pitch TPCAM / Gt is not a positive finite number");
    const int dim_roi = static_cast<int>(dim);
    const double bins = std::round(crop_na / lambda * pitch * dim_roi);
    const int half = dim_roi / 2;
    nxmax = std::max(1, bins < half ? static_cast<int>(bins) : half);

    ostringstream line;
    line << setprecision(9) << "NXMAX not set: derived " << nxmax << " = round("
         << (crop_na_set ? "CROP_NA " : "NA (no CROP_NA) ") << crop_na << " / LAMBDA " << lambda
         << " x pitch " << pitch << " x DIM_ROI " << dim_roi << ") clamped to [1, DIM_ROI / 2], pitch = TPCAM "
         << tpcam << " / ((F_TUBE " << f_tube << " / F_OBJ " << f_obj << ") / RF " << rf << ")";
    cout << line.str() << endl;
    return true;
}

string ampli_ref_config_path(const string& acquisition_path, const string& processing_path)
{
    if (extract_string("CONFIG_SCHEMA", acquisition_path) != "3")
        return processing_path;
    cout << "AMPLI_REF from " << acquisition_path << " (CONFIG_SCHEMA 3)" << endl;
    return acquisition_path;
}

int clear_acquisition(string  rep, string chemin_config_manip,string chemin_recon)
{
    const char * rep_c=rep.c_str();
    DIR *theFolder = opendir(rep_c);
    struct dirent *next_file;
    char filepath[512];

    while ( (next_file = readdir(theFolder)) != NULL )
    {
        snprintf(filepath, sizeof(filepath), "%s/%s", rep_c, next_file->d_name);
        if(filepath!=chemin_config_manip && filepath!=chemin_recon)
        remove(filepath);
    }
    closedir(theFolder);
    return 0;
}

namespace {

// The key is the line's first word (after leading blanks), separated from its value by spaces or tabs; the value is
// the rest of the line up to a '#', without the blanks around it (internal ones kept). A comment line, a bare key and a
// key followed only by a comment have no value. "KEY=value" is not read: '=' is not a separator (CONFIG_SCHEMA 3
// forbids it, and the GUI writes "KEY value").
bool config_line_value(const string& line, const string& token, string& value)
{
    static const char* const blanks = " \t\r";
    const string text = line.substr(0, line.find('#'));
    const size_t key_begin = text.find_first_not_of(blanks);
    if (key_begin == string::npos)
        return false;
    const size_t key_end = text.find_first_of(blanks, key_begin);
    if (key_end == string::npos || text.compare(key_begin, key_end - key_begin, token) != 0)
        return false;
    const size_t begin = text.find_first_not_of(blanks, key_end);
    if (begin == string::npos)
        return false;
    value = text.substr(begin, text.find_last_not_of(blanks) - begin + 1);
    return true;
}

string read_config_value(const string& token, const string& file_path)
{
    ifstream file(file_path.c_str(), ios::in);
    if (!file) {
        cerr << "Cannot open file: " << file_path << endl;
        return "";
    }

    string line, candidate, value;
    while (getline(file, line)) {
        if (config_line_value(line, token, candidate)) {
            cout << token << "=" << candidate << endl;
            value = candidate;
        }
    }
    return value;
}

} // namespace

// Extract a string value for the given key from a config file
string extract_string(std::string token, std::string file_path)
{
    const string value = read_config_value(token, file_path);
    if (value.empty())
        cout << "Key " << token << " not found in file " << file_path << endl;
    return value;
}

float extract_val(string token, string file_path)
{
    return atof(read_config_value(token, file_path).c_str());
}


float write_val(string token, float token_value, string file_path)
{
     // Backup the file first
     ifstream src(file_path.c_str() ,ios::binary);
     ofstream dst(file_path+"_SAV" ,ios::binary);
     dst<<src.rdbuf();
     src.close();
     dst.close();

     string line, keyword, value_str, new_line, string_val_token, separator=" ";
     std::ostringstream ss;
     ifstream file_stream(file_path.c_str(), ios::in);
     vector<std::string> file_buffer;
     if(file_stream)
     {    while(!file_stream.eof()){
            getline(file_stream,line,'\n');
            if(line[0]!='#') // not a comment line
            {
                int sep_pos=line.find(separator);
                int sep_len=separator.length();
                keyword = line.substr(0, sep_pos);
                if(keyword==token){
                ss << token_value;
                string_val_token=ss.str();
                new_line=token+" "+string_val_token;
                file_buffer.push_back(new_line);
                }
                else
                    file_buffer.push_back(line);
            }
            else{ // comment line, keep as-is
            file_buffer.push_back(line);
            }
        }
     }
     else
            cerr << "Error opening file: "<<file_path << endl;
    file_stream.close();
    ofstream write_stream(file_path.c_str(), ios::out);
    for(int i=0;i<file_buffer.size();i++)
    write_stream<<file_buffer[i]<<endl;
    write_stream.close();
     return 0;
}
