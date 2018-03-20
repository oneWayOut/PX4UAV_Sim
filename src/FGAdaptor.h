#include "FGLocation.h"

#ifndef _FGADAPTOR_H_
#define _FGADAPTOR_H_

class FGLocation;

//tocheck define as a class???


double GetSeaLevelRadius0(void);
double GetTerrainGeoCentRadius(void);
double GetAltitude(const FGLocation& loc);
double GetAGLevel(const FGLocation& loc,
                  FGLocation& contact, FGColumnVector3& normal,
                  FGColumnVector3& vel, FGColumnVector3& angularVel);

const FGColumnVector3 GetOmegaPlanet();

double GetSemimajor(void);
double GetSemiminor(void);


FGColumnVector3 GetGravityJ2(const FGColumnVector3& position, double Latitude);



#endif
