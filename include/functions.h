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

using namespace std;

#include "projet.h"

typedef struct{
clock_t start, end;
float   total;
}timer_info;

float extract_val(string token,  string file_path);
float write_val(string token, float token_value, string file_path);
string extract_string(string token,  string file_path);
int clear_acquisition(string  rep, string chemin_config_manip,string chemin_recon);
#endif
