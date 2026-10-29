#include "functions.h"
#include <tiffio.h>
#include <dirent.h>
#include <chrono>
#include <cerrno>
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

bool config_line_value(const string& line, const string& token, string& value)
{
    const size_t first = line.find_first_not_of(" \t\r");
    if (first == string::npos || line[first] == '#')
        return false;

    const size_t separator = line.find(' ');
    if (separator == string::npos || line.substr(0, separator) != token)
        return false;

    string candidate = line.substr(separator + 1);
    const size_t comment = candidate.find('#');
    if (comment != string::npos)
        candidate.erase(comment);
    const size_t begin = candidate.find_first_not_of(" \t\r");
    if (begin == string::npos)
        return false;

    // Preserve uncommented values byte for byte, including internal spaces.
    if (comment != string::npos)
        candidate = candidate.substr(begin, candidate.find_last_not_of(" \t\r") - begin + 1);
    value = candidate;
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
