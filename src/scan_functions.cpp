#include "scan_functions.h"
#include "math.h"
#include <vector>
#include <numeric>
#include <iostream>
#include <algorithm>
#include <cstdlib>
using namespace std;

// Rose (rhodonea) curve: update voltage for next point
float2D update_rose(float rho, int nbHolo, float *theta)
{
    float2D Vout={0,0};
    float t=*theta; // current polar angle
    float delta_theta=6.28/nbHolo;
    t=delta_theta+t; // next curvilinear abscissa
    Vout.x=rho*cos(4*t)*cos(t);
    Vout.y=rho*cos(4*t)*sin(t);
    *theta=t;
    return Vout;
}

// Rose (rhodonea) curve: fill voltage table
void scan_rose(float rho, int nbHolo, std::vector<float2D> &Vout_table)
{
    double theta=0; // polar angle
    double delta_theta=2*M_PI/nbHolo;
    for(int numHolo=0;numHolo<nbHolo;numHolo++){
        Vout_table[numHolo].x=rho*cos(4*theta)*cos(theta);
        Vout_table[numHolo].y=rho*cos(4*theta)*sin(theta);
        theta=delta_theta+theta;
    }
}

// Concentric circles: distribute points across circles proportionally to perimeter
void scan_annular(float Vmax, float nb_circle, int nbHolo, std::vector<float2D> &Vout_table)
{
    int const circle_count = max(1, static_cast<int>(lround(nb_circle)));
    vector<double> radii(circle_count);
    for (int circle = 0; circle < circle_count; ++circle)
        radii[circle] = Vmax * (circle_count - circle) / circle_count;
    double const radius_sum = accumulate(radii.begin(), radii.end(), 0.0);
    vector<int> points_per_circle(circle_count, 0);
    int remaining = nbHolo;
    for (int circle = circle_count - 1; circle > 0; --circle) {
        points_per_circle[circle] = static_cast<int>(lround(radii[circle] / radius_sum * nbHolo));
        remaining -= points_per_circle[circle];
    }
    points_per_circle[0] = max(0, remaining);

    int output = 0;
    for (int circle = 0; circle < circle_count && output < nbHolo; ++circle) {
        int const point_count = points_per_circle[circle];
        for (int point = 0; point < point_count && output < nbHolo; ++point) {
            double const theta = 2.0 * M_PI * point / point_count;
            Vout_table[output++] = {
                static_cast<float>(radii[circle] * cos(theta)),
                static_cast<float>(radii[circle] * sin(theta))
            };
        }
    }
}

// Fermat spiral with golden angle spacing
void scan_fermat(double Vmax, int nbHolo, std::vector<float2D> &Vout_table, Var2D dim)
{
    double theta = 0.0;
    double const theta_max = nbHolo;
    double const golden = 3.0 * sqrt(5.0) * M_PI;
    double const delta_theta = theta_max / nbHolo;
    double const pixels_per_volt = dim.x / (2.0 * Vmax);
    vector<int> center(static_cast<size_t>(dim.x) * dim.y + 1, 0);

    for (int index = 0; index < nbHolo; ++index) {
        Vout_table[index].x = sqrt(theta / theta_max) * Vmax * cos(theta * golden);
        Vout_table[index].y = sqrt(theta / theta_max) * Vmax * sin(theta * golden);
        int const sx = static_cast<int>(lround(Vout_table[index].x * pixels_per_volt));
        int const sy = static_cast<int>(lround(Vout_table[index].y * pixels_per_volt));
        int const x = sx + dim.x / 2;
        int const y = sy + dim.y / 2;
        if (x >= 0 && x < dim.x && y >= 0 && y < dim.y)
            center[static_cast<size_t>(y) * dim.x + x] = index;
        theta += delta_theta;
    }

    int output = 0;
    for (int flat = 0; flat < dim.x * dim.y && output < nbHolo; ++flat) {
        int const y = flat / dim.x;
        int const x = (y % 2 == 0) ? dim.x - flat % dim.x : flat % dim.x;
        size_t const lookup = static_cast<size_t>(y) * dim.x + x;
        if (center[lookup] != 0) {
            Vout_table[output].x = x / pixels_per_volt - Vmax;
            Vout_table[output].y = y / pixels_per_volt - Vmax;
            ++output;
        }
    }
}

// Random cartesian coordinates within a disk of radius Vmax
void scan_random_cartesian(double Vmax, int nbHolo, std::vector<float2D> &Vout_table, Var2D dim)
{
    double const nbPixVolt=(dim.x/(2*Vmax));
    srand (time(NULL));
    double nbAleaX=0, nbAleaY=0;
    vector<double> center(dim.x*dim.y);
    int numHolo=0;
    while(numHolo<nbHolo){
        nbAleaX=Vmax*(2*rand()/(double)RAND_MAX)-1;
        nbAleaY=Vmax*(2*rand()/(double)RAND_MAX)-1;
        if(nbAleaX*nbAleaX+nbAleaY*nbAleaY<Vmax*Vmax){
            Vout_table[numHolo].x=nbAleaX;
            Vout_table[numHolo].y=nbAleaY;
            numHolo++;
        }
    }
    Var2D spec={0,0},specI={0,0};
    // Place random points on the Vx/Vy grid
    for(int numHolo=0; numHolo<nbHolo; numHolo++){
        spec.x=round(Vout_table[numHolo].x*nbPixVolt);
        spec.y=round(Vout_table[numHolo].y*nbPixVolt);
        specI.x=spec.x+dim.x/2;
        specI.y=spec.y+dim.y/2;
        center[specI.y*dim.x+specI.x]=1;
    }
    // Reorder to minimize mirror jumps (zigzag raster)
     numHolo=0;
    // Scan even rows with increasing x
    for( int y=0; y<dim.y; y=y+2){
        for(int x=0; x<dim.x; x++){
            if(center[y*dim.x+x]==1){
                Vout_table[numHolo].x=x/nbPixVolt-Vmax;
                Vout_table[numHolo].y=y/nbPixVolt-Vmax;
                numHolo++;
            }
        }
    }
    // Scan odd rows with decreasing x
    for( int y=1; y<dim.y; y=y+2){
        for(int x=dim.x; x<dim.x; x--){
            if(center[y*dim.x+x]==1){
                Vout_table[numHolo].x=x/nbPixVolt-Vmax;
                Vout_table[numHolo].y=y/nbPixVolt-Vmax;
                numHolo++;
            }
        }
    }
}

// Random polar coordinates on a sphere, projected to 2D voltages
void scan_random_polar3D(double Vmax, int nbHolo, std::vector<float2D> &Vout_table, Var2D dim)
{
    double const nbPixVolt=(dim.x/(2*Vmax));
    double nbAleaPhi=0, nbAleaTheta=0;
    vector<int> center(dim.x*dim.y);
    int numHolo=0;
    // Generate unique random voltages and place on integer grid
    while(numHolo<nbHolo)
    {
        nbAleaPhi=acos(2.0*(rand()/(double)RAND_MAX)-1.0); // phi in [0, pi]
        nbAleaTheta=rand()/(double)RAND_MAX*2*M_PI;         // theta in [0, 2pi]
        Vout_table[numHolo].x = Vmax*cos(nbAleaTheta)*sin(nbAleaPhi);
        Vout_table[numHolo].y= Vmax*sin(nbAleaTheta)*sin(nbAleaPhi);

        int x=round((Vout_table[numHolo].x+Vmax)*nbPixVolt);
        int y=round((Vout_table[numHolo].y+Vmax)*nbPixVolt);
        x=std::max(0, std::min(dim.x-1, x));
        y=std::max(0, std::min(dim.y-1, y));
        if(center[y*dim.x+x]!=1) // ensure uniqueness
        {
        center[y*dim.x+x]=1;
        numHolo++;
        }
    }
    // Reorder to minimize mirror jumps (zigzag raster)
    numHolo=0;
    for( int y=0; y<dim.y; y=y+1){
        if(y%2==0) // even rows: increasing x
        {
            for( int x=0; x<dim.x; x++){
                if(center[y*dim.x+x]==1){
                    Vout_table[numHolo].x=x/nbPixVolt-Vmax;
                    Vout_table[numHolo].y=y/nbPixVolt-Vmax;
                    numHolo++;
                 }
            }
        }
        else // odd rows: decreasing x
        {
            for (int x=dim.x-1;x>=0;x--){
                if(center[y*dim.x+x]==1){
                Vout_table[numHolo].x=x/nbPixVolt-Vmax;
                Vout_table[numHolo].y=y/nbPixVolt-Vmax;
                numHolo++;
                }
            }
        }
    }
}

// Archimedean spiral: single-point update
float2D update_spiral(float rho, int nbHolo, float *theta)
{
    float2D Vout={0,0};
    double numbTurn=10;
    double thetaMax=2*numbTurn*M_PI,thetaMin=0;
    double t=*theta; // current polar angle
    double longTot=floor(0.5*(log(thetaMax+sqrt(thetaMax*thetaMax+1))+thetaMax*sqrt(thetaMax*thetaMax+1)-
                         log(thetaMin+sqrt(thetaMin*thetaMin+1))+thetaMin*sqrt(thetaMin*thetaMin+1)));
    double delta_theta=longTot/nbHolo*1/(sqrt(1+t*t));
    t=delta_theta+t;
    Vout.x = (rho*t/thetaMax)*sin(t);
    Vout.y =(rho*t/thetaMax)*cos(t);
    *theta=t;
    return Vout;
}

// Archimedean spiral with oversampling for uniform arc-length spacing
void scan_spiralOS(double Vmax, int nbHolo, vector<float2D> &Vout_table, unsigned short int nbTurn)
{
    double const numbTurn=nbTurn;
    int const nbSample=10000;
    vector<float2D> Vout_tableOS(nbSample);
    double thetaMax=2*numbTurn*M_PI,thetaMin=0, theta=0;
    double longTot=floor(0.5*(log(thetaMax+sqrt(thetaMax*thetaMax+1))+thetaMax*sqrt(thetaMax*thetaMax+1)-
                         log(thetaMin+sqrt(thetaMin*thetaMin+1))+thetaMin*sqrt(thetaMin*thetaMin+1)));

    for(int numHolo=0; numHolo<nbSample; numHolo++){
        double delta_theta=longTot/nbSample*1/(sqrt(1+theta*theta));
        theta=delta_theta+theta;
        Vout_tableOS[numHolo].x = (Vmax*theta/thetaMax)*sin(theta);
        Vout_tableOS[numHolo].y =(Vmax*theta/thetaMax)*cos(theta);
    }
    int ratioOS=nbSample/nbHolo; // oversampling ratio
    for(int numHolo=0; numHolo<nbHolo; numHolo++){
        Vout_table[numHolo].x=Vout_tableOS[numHolo*ratioOS].x;
        Vout_table[numHolo].y=Vout_tableOS[numHolo*ratioOS].y;
    }
}

// Archimedean spiral with uniform arc-length spacing (no oversampling)
void scan_spiral(double Vmax, int nbHolo, std::vector<float2D> &Vout_table, Var2D dim, unsigned short int nbTurn)
{
    double numbTurn=nbTurn, thetaMax=2*numbTurn*M_PI,thetaMin=0, theta=0;
    double longTot=floor(0.5*(log(thetaMax+sqrt(thetaMax*thetaMax+1))+thetaMax*sqrt(thetaMax*thetaMax+1)-
                         log(thetaMin+sqrt(thetaMin*thetaMin+1))+thetaMin*sqrt(thetaMin*thetaMin+1)));

    for(int numHolo=0; numHolo<nbHolo; numHolo++){
        double delta_theta=longTot/nbHolo*1/(sqrt(1+theta*theta));
        theta=delta_theta+theta;
        Vout_table[numHolo].x = (Vmax*theta/thetaMax)*sin(theta);
        Vout_table[numHolo].y =(Vmax*theta/thetaMax)*cos(theta);
    }
}

// Uniform spherical sampling projected to 2D voltages
void scan_uniform3D(double Vmax, int nbHolo, std::vector<float2D> &Vout_table)
{
    double const average_surface_per_point = 2.0 * M_PI / nbHolo;
    double const average_distance = sqrt(average_surface_per_point);
    int const circle_count = max(1, static_cast<int>(lround((M_PI / 2.0) / average_distance)));
    double const delta_theta = (M_PI / 2.0) / circle_count;
    double total_circle_length = 0.0;
    for (int circle = 0; circle < circle_count; ++circle)
        total_circle_length += 2.0 * M_PI * sin(circle * delta_theta);
    double const recalculated_distance = total_circle_length / nbHolo;

    int output = 0;
    for (int circle = 0; circle < circle_count && output < nbHolo; ++circle) {
        double const theta = circle * delta_theta;
        int const point_count = static_cast<int>(lround(
            2.0 * M_PI * sin(theta) / max(recalculated_distance, 1e-12)));
        for (int point = 0; point < point_count && output < nbHolo; ++point) {
            double const phi = 2.0 * M_PI * point / point_count;
            Vout_table[output++] = {
                static_cast<float>(Vmax * sin(theta) * cos(phi)),
                static_cast<float>(Vmax * sin(theta) * sin(phi))
            };
        }
    }
}

// Star pattern: radial axes through the origin
void scan_star(double Vmax, int nbHolo, int nbAxe, vector<float2D> &Vout_table)
{
double delta_theta=M_PI/nbAxe;
int pts_per_axis=floor((double)nbHolo/(double)nbAxe);
vector<double> kiz(nbHolo);
vector<int> axis_point_count(nbAxe);
int numHolo=0;
int nbPtLeft=nbHolo-(pts_per_axis*nbAxe); // remainder points
vector<double> theta(nbAxe);
for(int i=0;i<nbAxe;i++){
theta[i]=i*delta_theta;
if(i>=(nbAxe-nbPtLeft)) // distribute remainder across last axes
    axis_point_count[i]=pts_per_axis+1;
else axis_point_count[i]=pts_per_axis;
}
for(int i=0;i<nbAxe;i++){
    double delta_rho=2*Vmax/axis_point_count[i];
    double rho=0;
    for(int numPt=0; numPt<axis_point_count[i];numPt++){
        rho=-Vmax+numPt*delta_rho; // sweep radius at fixed theta
        Vout_table[numHolo].x=rho*cos(theta[i]);
        Vout_table[numHolo].y=rho*sin(theta[i]);
        kiz[numHolo]=sqrt(Vmax*Vmax-pow(Vout_table[numHolo].x,2)-pow(Vout_table[numHolo].y,2));
        numHolo++;
    }
}
cout<<"Points placed="<<numHolo<<endl;
}
