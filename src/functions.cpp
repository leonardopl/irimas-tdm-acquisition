#include "functions.h"
#include <tiffio.h>
#include <dirent.h>
#include <chrono>
#define PI 3.14159

int clear_acquisition(string  rep, string chemin_config_manip,string chemin_recon)
{
    const char * rep_c=rep.c_str();
    DIR *theFolder = opendir(rep_c);
    struct dirent *next_file;
    char filepath[256];

    while ( (next_file = readdir(theFolder)) != NULL )
    {
        sprintf(filepath, "%s/%s", rep_c, next_file->d_name);
        if(filepath!=chemin_config_manip && filepath!=chemin_recon)
        remove(filepath);
    }
    closedir(theFolder);
    return 0;
}

// Extract a string value for the given key from a config file
string extract_string(std::string token,  std::string file_path)
{
    ifstream file(file_path.c_str(), ios::in);
    string line, keyword, value_str, separator=" ";
    string value;
    vector<std::string> tokens;

    if(file)
    {
        while(!file.eof()){
            getline(file,line,'\n');
            if(line[0]!='#') // skip comment lines
            tokens.push_back(line);
        }
    }
    else
        cerr << "Cannot open file: "<< file_path<< endl;

    int num_tokens=tokens.size();
    for(int i=0;i<num_tokens;i++){
        line=tokens[i];
        if(line!=""){
            int sep_pos=line.find(separator);
            int sep_len=separator.length();
            keyword = line.substr(0, sep_pos);
            if(keyword==token){
            value_str=line.substr(sep_pos+sep_len,line.size()-(keyword.size()+sep_len));
            cout<<keyword<<"="<<value_str<<endl;
            value=value_str.c_str();
            }
        }
    }
    if(value.empty())
        cout<<"Key "<<token<<" not found in file "<<file_path<<endl;
    file.close();
    return value;
}

float extract_val(string token,  string file_path)
{
    ifstream file(file_path.c_str(), ios::in);
    string line, keyword, value_str, separator=" ";
    float value=0;
    vector<std::string> tokens;

    if(file)
    {
        while(!file.eof()){
            getline(file,line,'\n');
            if(line[0]!='#') // skip comment lines
            tokens.push_back(line);
        }
    }
    else
        cerr << "Cannot open file: "<< file_path<< endl;

    int num_tokens=tokens.size();
    for(int i=0;i<num_tokens;i++){
        line=tokens[i];
        if(line!=""){
            int sep_pos=line.find(separator);
            int sep_len=separator.length();
            keyword = line.substr(0, sep_pos);
            if(keyword==token){
            value_str=line.substr(sep_pos+sep_len,line.size()-(keyword.size()+sep_len));
            cout<<keyword<<"="<<value_str<<endl;
            value=atof(value_str.c_str());
            }
        }
    }
    file.close();
    return value;
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
