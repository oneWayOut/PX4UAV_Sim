#include "FGAdaptor.h"

#if 1  //sc uinit
// Earth defaults
static const double RotationRate    = 0.00007292115;     //rad/s
static const double GM              = 3.986004418E14; //14.0764417572E15;   // WGS84 value
static const double C2_0            = -4.84165371736E-04; // WGS84 value for the C2,0 coefficient
static const double J2              = 1.081874E-03;// 1.08262982E-03;     // WGS84 value for J2
static const double a               = 6378137.0;     // WGS84 semimajor axis length in meter
static const double b               = 6356752.314245;      // WGS84 semiminor axis length in meter
#else  //ft
static const double RotationRate    = 0.00007292115;
static const double GM              = 14.0764417572E15;   // WGS84 value
static const double C2_0            = -4.84165371736E-04; // WGS84 value for the C2,0 coefficient
static const double J2              = 1.08262982E-03;     // WGS84 value for J2
static const double a               = 20925646.32546;     // WGS84 semimajor axis length in feet
static const double b               = 20855486.5951;      // WGS84 semiminor axis length in feet
#endif




static const double RadiusReference = a;


static double mSeaLevelRadius = RadiusReference;
static double mTerrainLevelRadius = mSeaLevelRadius;




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


//NOTE: Function GetGravityJ2() is from FGInertial.cpp.
// Calculate the WGS84 gravitation value in ECEF frame. Pass in the ECEF position
// via the position parameter. The J2Gravity value returned is in ECEF frame,
// and therefore may need to be expressed (transformed) in another frame,
// depending on how it is used. See Stevens and Lewis eqn. 1.4-16.
FGColumnVector3 GetGravityJ2(const FGColumnVector3& position, double Latitude)
{
  FGColumnVector3 J2Gravity;

  // Gravitation accel
  double r = position.Magnitude();
  double sinLat = sin(Latitude);

  double adivr = a/r;
  double preCommon = 1.5*J2*adivr*adivr;
  double xy = 1.0 - 5.0*(sinLat*sinLat);
  double z = 3.0 - 5.0*(sinLat*sinLat);
  double GMOverr2 = GM/(r*r);

  J2Gravity(1) = -GMOverr2 * ((1.0 + (preCommon * xy)) * position(eX)/r);
  J2Gravity(2) = -GMOverr2 * ((1.0 + (preCommon * xy)) * position(eY)/r);
  J2Gravity(3) = -GMOverr2 * ((1.0 + (preCommon *  z)) * position(eZ)/r);

  return J2Gravity;
}
