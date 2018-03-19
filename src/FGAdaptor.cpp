#include "FGAdaptor.h"

//todo change the value
static double mSeaLevelRadius = 6131;
static double mTerrainLevelRadius = 6131;




// Earth defaults
static const double RotationRate    = 0.00007292115;     //rad/s
static const double GM              = 3.986004418E14; //14.0764417572E15;   // WGS84 value
static const double C2_0            = -4.84165371736E-04; // WGS84 value for the C2,0 coefficient
static const double J2              = 1.081874E-03;// 1.08262982E-03;     // WGS84 value for J2
static const double a               = 6378137.0;     // WGS84 semimajor axis length in meter
static const double b               = 6356752.314245;      // WGS84 semiminor axis length in meter
static const double RadiusReference = a;





double GetSeaLevelRadius0(void)
 {return mSeaLevelRadius; }

double GetTerrainGeoCentRadius(void)
{
	return mTerrainLevelRadius; 
}



double GetAltitude(const FGLocation& loc)
{
  return loc.GetRadius() - mSeaLevelRadius;
}


double GetAGLevel(const FGLocation& loc,
					FGLocation& contact, FGColumnVector3& normal,
					FGColumnVector3& vel, FGColumnVector3& angularVel)
{
  vel = FGColumnVector3(0.0, 0.0, 0.0);
  angularVel = FGColumnVector3(0.0, 0.0, 0.0);
  normal = FGColumnVector3(loc).Normalize();
  double loc_radius = loc.GetRadius();  // Get the radius of the given location
                                        // (e.g. the CG)
  double agl = loc_radius - mTerrainLevelRadius;
  contact = (mTerrainLevelRadius/loc_radius)*FGColumnVector3(loc);
  return agl;
}






const FGColumnVector3 GetOmegaPlanet()
{
    return FGColumnVector3( 0.0, 0.0, RotationRate );
}


double GetSemimajor(void)  {return a;}
double GetSemiminor(void)  {return b;}
