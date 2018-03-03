#include "FGJSBBase.h"


const double radtodeg = 57.295779513082320876798154814105;
const double degtorad = 0.017453292519943295769236907684886;
const double hptoftlbssec = 550.0;
const double psftoinhg = 0.014138;
const double psftopa = 47.88;
const double ktstofps = 1.68781;
const double fpstokts = 1.0/ktstofps;
const double inchtoft = 0.08333333;
const double in3tom3 = 1.638706E-5;
const double m3toft3 = 1.0/(fttom*fttom*fttom);
const double inhgtopa = 3386.38;
const double fttom = 0.3048;
double Reng = 1716.56;   // Gas constant for Air (ft-lb/slug-R)
double Rstar = 1545.348; // Universal gas constant
double Mair = 28.9645;   //
const double SHRatio = 1.40;

// Note that definition of lbtoslug by the inverse of slugtolb and not
// to a different constant you can also get from some tables will make
// lbtoslug*slugtolb == 1 up to the magnitude of roundoff. So converting from
// slug to lb and back will yield to the original value you started with up
// to the magnitude of roundoff.
// Taken from units gnu commandline tool
const double slugtolb = 32.174049;
const double lbtoslug = 1.0/slugtolb;
const double kgtolb = 2.20462;
const double kgtoslug = 0.06852168;


double sign(double num) {return num>=0.0?1.0:-1.0;}


