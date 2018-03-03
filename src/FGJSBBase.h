
#ifndef FGJSBBASE_H
#define FGJSBBASE_H

#include <cmath>


#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif


extern const double radtodeg;
extern const double degtorad;
extern const double hptoftlbssec;
extern const double psftoinhg;
extern const double psftopa;
extern const double fpstokts;
extern const double ktstofps;
extern const double inchtoft;
extern const double in3tom3;
extern const double m3toft3;
extern const double inhgtopa;
extern const double fttom;
extern double Reng;         // Specific Gas Constant,ft^2/(sec^2*R)
extern double Rstar;
extern double Mair;
extern const double SHRatio;
extern const double lbtoslug;
extern const double slugtolb;
extern const double kgtolb;
extern const double kgtoslug;

/// Moments L, M, N
enum {eL     = 1, eM,     eN    };
/// Rates P, Q, R
enum {eP     = 1, eQ,     eR    };
/// Velocities U, V, W
enum {eU     = 1, eV,     eW    };
/// Positions X, Y, Z
enum {eX     = 1, eY,     eZ    };
/// Euler angles Phi, Theta, Psi
enum {ePhi   = 1, eTht,   ePsi  };
/// Stability axis forces, Drag, Side force, Lift
enum {eDrag  = 1, eSide,  eLift };
/// Local frame orientation Roll, Pitch, Yaw
enum {eRoll  = 1, ePitch, eYaw  };
/// Local frame position North, East, Down
enum {eNorth = 1, eEast,  eDown };
/// Locations Radius, Latitude, Longitude
enum {eLat = 1, eLong, eRad     };
/// Conversion specifiers
enum {inNone = 0, inDegrees, inRadians, inMeters, inFeet };


double sign(double num);

#endif

