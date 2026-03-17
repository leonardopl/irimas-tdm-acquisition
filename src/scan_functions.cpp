#include "scan_functions.h"
#include "math.h"
#include <vector>
#include <numeric>
#include <iostream>
using namespace std;

// Rose (rhodonea) curve: update voltage for next point
float2D maj_fleur(float rho, int nbHolo, float *theta)
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
void scan_fleur(float rho, int nbHolo, std::vector<float2D> &Vout_table)
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
    double delta_rho=(Vmax/nb_circle); // voltage step between successive radii
    cout<<"delta_rho="<<delta_rho<<endl;
    double total_length=0;
    double perimeter=0;
    int numHolo=0;
    vector<double> kiz(nbHolo);
    vector<size_t> pts_per_circle(nb_circle);
    vector<double> rho_voltage(nb_circle);
    size_t numCircle=0;
    // Build radius voltage table
    for(double rho=Vmax; rho>0; rho=rho-delta_rho){
            rho_voltage[numCircle]=rho;
            cout<<"rho_voltage="<<rho<<endl;
            numCircle++;
        }
    total_length=2*M_PI*accumulate(rho_voltage.begin(),rho_voltage.end(),0);

    numCircle=0;
    size_t pts_remaining=nbHolo;
    // Distribute points across circles proportionally to perimeter
    // Handle rounding by assigning remainder to innermost circle
    for(int numCircle=nb_circle-1; numCircle>=0; numCircle--)
        {
            perimeter=2*M_PI*rho_voltage[numCircle];
            if (numCircle!=0)
                {
                    pts_per_circle[numCircle]=round((perimeter/total_length)*nbHolo);
                    pts_remaining=pts_remaining-pts_per_circle[numCircle];
                }
            else pts_per_circle[numCircle]=pts_remaining;
        }
    numCircle=0;
    // Scan circle by circle
    for(int numCircle=0; numCircle<nb_circle; numCircle++)
        {
            int nbPoint=pts_per_circle[numCircle];
            double delta_theta=2*M_PI/nbPoint;
            cout<<"rho_voltage="<<rho_voltage[numCircle]<<endl;

            for(int cpt=0; cpt<nbPoint; cpt++)
                {
                    Vout_table[numHolo].x=rho_voltage[numCircle]*cos(delta_theta*cpt);
                    Vout_table[numHolo].y=rho_voltage[numCircle]*sin(delta_theta*cpt);
                    if(pow(Vmax,2)-pow(Vout_table[numHolo].x,2)-pow(Vout_table[numHolo].y,2)>0)
                    kiz[numHolo]=sqrt(pow(Vmax,2)-pow(Vout_table[numHolo].x,2)-pow(Vout_table[numHolo].y,2));
                    else{
                        kiz[numHolo]=0;
                    }
                    cout<<kiz[numHolo]<<endl;
                    numHolo++;
                }
        }
}

// Fermat spiral with golden angle spacing
void scan_fermat(double Vmax, int nbHolo, std::vector<float2D> &Vout_table, Var2D dim)
{
    double theta=0,theta_max=nbHolo; // theta_max in radians = number of holograms
    double angle_dor=(3*sqrt(5))*M_PI; // golden angle
    double delta_theta=theta_max/nbHolo;
    cout<<"delta_theta="<<delta_theta<<endl;
    vector<double> centre(dim.x*dim.y);
    double nbPixVolt=(dim.x/(2*Vmax));
    Var2D spec={0,0},specI={0,0};

    for(int numHolo=0;numHolo<nbHolo;numHolo++){
    Vout_table[numHolo].x=sqrt((theta)/theta_max)*Vmax*cos((theta)*angle_dor);
    Vout_table[numHolo].y=sqrt((theta)/theta_max)*Vmax*sin((theta)*angle_dor);

    spec.x=round(Vout_table[numHolo].x*nbPixVolt);
    spec.y=round(Vout_table[numHolo].y*nbPixVolt);
    specI.x=spec.x+dim.x/2;
    specI.y=spec.y+dim.y/2;
    centre[specI.y*dim.x+specI.x]=numHolo;

    theta=theta+delta_theta;
    }
    // Reorder points to minimize mirror jumps (zigzag raster scan)
    int numHolo=0;
    int x=0,y=0;
    for(int cpt=0; cpt<dim.x*dim.y; cpt++){
        y=cpt/dim.x;
        if(y%2==0) x=dim.x-cpt%dim.x; // even rows: decreasing x
        else x=cpt%dim.x;              // odd rows: increasing x
        if(centre[y*dim.x+x]!=0){
            Vout_table[numHolo].x=x/nbPixVolt-Vmax;
            Vout_table[numHolo].y=y/nbPixVolt-Vmax;
            numHolo++;
        }
    }
}

// Random cartesian coordinates within a disk of radius Vmax
void scan_random_cartesian(double Vmax, int nbHolo, std::vector<float2D> &Vout_table, Var2D dim)
{
    double const nbPixVolt=(dim.x/(2*Vmax));
    srand (time(NULL));
    double nbAleaX=0, nbAleaY=0;
    vector<double> centre(dim.x*dim.y);
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
        centre[specI.y*dim.x+specI.x]=1;
    }
    // Reorder to minimize mirror jumps (zigzag raster)
     numHolo=0;
    // Scan even rows with increasing x
    for( int y=0; y<dim.y; y=y+2){
        for(int x=0; x<dim.x; x++){
            if(centre[y*dim.x+x]==1){
                Vout_table[numHolo].x=x/nbPixVolt-Vmax;
                Vout_table[numHolo].y=y/nbPixVolt-Vmax;
                numHolo++;
            }
        }
    }
    // Scan odd rows with decreasing x
    for( int y=1; y<dim.y; y=y+2){
        for(int x=dim.x; x<dim.x; x--){
            if(centre[y*dim.x+x]==1){
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
    srand (time(NULL));
    double nbAleaPhi=0, nbAleaTheta=0;
    vector<int> centre(dim.x*dim.y);
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
        if(centre[y*dim.x+x]!=1) // ensure uniqueness
        {
        centre[y*dim.x+x]=1;
        numHolo++;
        }
    }
    // Reorder to minimize mirror jumps (zigzag raster)
    numHolo=0;
    for( int y=0; y<dim.y; y=y+1){
        if(y%2==0) // even rows: increasing x
        {
            for( int x=0; x<dim.x; x++){
                if(centre[y*dim.x+x]==1){
                    Vout_table[numHolo].x=x/nbPixVolt-Vmax;
                    Vout_table[numHolo].y=y/nbPixVolt-Vmax;
                    numHolo++;
                 }
            }
        }
        else // odd rows: decreasing x
        {
            for (int x=dim.x-1;x>=0;x--){
                if(centre[y*dim.x+x]==1){
                Vout_table[numHolo].x=x/nbPixVolt-Vmax;
                Vout_table[numHolo].y=y/nbPixVolt-Vmax;
                numHolo++;
                }
            }
        }
    }
}

// Archimedean spiral: single-point update
float2D maj_spiral(float rho, int nbHolo, float *theta)
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
    double surface_moyenne_pt=2*M_PI/nbHolo;
    double distance_moyenne=sqrt(surface_moyenne_pt);
    vector<double> kiz(nbHolo);
    cout<<"mean distance="<<distance_moyenne<<endl;

    double theta_max=M_PI/2,theta_min=0;
    int nbCercle=round((theta_max-theta_min)/distance_moyenne); // number of concentric circles
    cout<<"total circles="<<nbCercle<<endl;
    double delta_theta=(theta_max-theta_min)/nbCercle;
    double theta=0,phi=0,delta_curv_phi=0;
    int numHolo=0;
    double total_circle_length=0;
    for(int num_cercle=0;num_cercle<nbCercle;num_cercle++){
        theta=(num_cercle)*delta_theta;
        delta_curv_phi=surface_moyenne_pt/delta_theta;
        total_circle_length=total_circle_length+2*M_PI*sin(theta);
    }
    double dist_recalc=total_circle_length/nbHolo;

    for(int num_cercle=0;num_cercle<nbCercle;num_cercle++){
        theta=(num_cercle)*delta_theta;
        delta_curv_phi=dist_recalc; // arc-length step on concentric circle
        int nbPtPhi=round(2*M_PI*sin(theta)/(delta_curv_phi)); // points on this circle
        for(int numPt=0;numPt<nbPtPhi;numPt++){
            phi=2*M_PI*(numPt)/nbPtPhi;
            Vout_table[numHolo].x=Vmax*sin(theta)*cos(phi);
            Vout_table[numHolo].y=Vmax*sin(theta)*sin(phi);
            kiz[numHolo]=Vmax*cos(theta);
            numHolo++;
        }
    }
}

// Star pattern: radial axes through the origin
void scan_etoile(double Vmax, int nbHolo, int nbAxe, vector<float2D> &Vout_table)
{
double delta_theta=M_PI/nbAxe;
int nbPtAxe=floor((double)nbHolo/(double)nbAxe);
vector<double> kiz(nbHolo);
vector<int> nbPointAxe(nbAxe);
int numHolo=0;
int nbPtLeft=nbHolo-(nbPtAxe*nbAxe); // remainder points
vector<double> theta(nbAxe);
for(int cpt=0;cpt<nbAxe;cpt++){
theta[cpt]=cpt*delta_theta;
if(cpt>=(nbAxe-nbPtLeft)) // distribute remainder across last axes
    nbPointAxe[cpt]=nbPtAxe+1;
else nbPointAxe[cpt]=nbPtAxe;
}
for(int cpt=0;cpt<nbAxe;cpt++){
    double delta_rho=2*Vmax/nbPointAxe[cpt];
    double rho=0;
    for(int numPt=0; numPt<nbPointAxe[cpt];numPt++){
        rho=-Vmax+numPt*delta_rho; // sweep radius at fixed theta
        Vout_table[numHolo].x=rho*cos(theta[cpt]);
        Vout_table[numHolo].y=rho*sin(theta[cpt]);
        kiz[numHolo]=sqrt(Vmax*Vmax-pow(Vout_table[numHolo].x,2)-pow(Vout_table[numHolo].y,2));
        numHolo++;
    }
}
cout<<"Points placed="<<numHolo<<endl;
}
