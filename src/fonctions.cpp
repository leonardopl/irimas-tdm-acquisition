#include "fonctions.h"
#include <tiffio.h>
#include <dirent.h>
#include <chrono>
#define PI 3.14159

int efface_acquis(string  rep, string chemin_config_manip,string chemin_recon)
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
string extract_string(std::string token,  std::string chemin_fic)
{
    ifstream file(chemin_fic.c_str(), ios::in);
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
        cerr << "Cannot open file: "<< chemin_fic<< endl;

    int nb_tok=tokens.size();
    for(int cpt=0;cpt<nb_tok;cpt++){
        line=tokens[cpt];
        if(line!=""){
            int pos_separ=line.find(separator);
            int long_separ=separator.length();
            keyword = line.substr(0, pos_separ);
            if(keyword==token){
            value_str=line.substr(pos_separ+long_separ,line.size()-(keyword.size()+long_separ));
            cout<<keyword<<"="<<value_str<<endl;
            value=value_str.c_str();
            }
        }
    }
    if(value.empty())
        cout<<"Key "<<token<<" not found in file "<<chemin_fic<<endl;
    file.close();
    return value;
}

float extract_val(string token,  string chemin_fic)
{
    ifstream file(chemin_fic.c_str(), ios::in);
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
        cerr << "Cannot open file: "<< chemin_fic<< endl;

    int nb_tok=tokens.size();
    for(int cpt=0;cpt<nb_tok;cpt++){
        line=tokens[cpt];
        if(line!=""){
            int pos_separ=line.find(separator);
            int long_separ=separator.length();
            keyword = line.substr(0, pos_separ);
            if(keyword==token){
            value_str=line.substr(pos_separ+long_separ,line.size()-(keyword.size()+long_separ));
            cout<<keyword<<"="<<value_str<<endl;
            value=atof(value_str.c_str());
            }
        }
    }
    file.close();
    return value;
}


float ecrire_val(string token, float valeur_token, string chemin_fic)
{
     // Backup the file first
     ifstream src(chemin_fic.c_str() ,ios::binary);
     ofstream dst(chemin_fic+"_SAV" ,ios::binary);
     dst<<src.rdbuf();
     src.close();
     dst.close();

     string line, keyword, value_str, new_line, string_val_token, separator=" ";
     std::ostringstream ss;
     ifstream file_stream(chemin_fic.c_str(), ios::in);
     vector<std::string> file_buffer;
     if(file_stream)
     {    while(!file_stream.eof()){
            getline(file_stream,line,'\n');
            if(line[0]!='#') // not a comment line
            {
                int pos_separ=line.find(separator);
                int long_separ=separator.length();
                keyword = line.substr(0, pos_separ);
                if(keyword==token){
                ss << valeur_token;
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
            cerr << "Error opening file: "<<chemin_fic << endl;
    file_stream.close();
    ofstream write_stream(chemin_fic.c_str(), ios::out);
    for(int cpt=0;cpt<file_buffer.size();cpt++)
    write_stream<<file_buffer[cpt]<<endl;
    write_stream.close();
     return 0;
}
