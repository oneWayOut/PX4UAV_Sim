#include "FGAdaptor.h"

//todo change the value
static double mSeaLevelRadius = 6131;
static double mTerrainLevelRadius = 6131;

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
