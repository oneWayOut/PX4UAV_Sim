/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 Module:       FGPropagate.cpp
 Author:       Jon S. Berndt
 Date started: 01/05/99
 Purpose:      Integrate the EOM to determine instantaneous position
 Called by:    FGFDMExec

 ------------- Copyright (C) 1999  Jon S. Berndt (jon@jsbsim.org) -------------

 This program is free software; you can redistribute it and/or modify it under
 the terms of the GNU Lesser General Public License as published by the Free Software
 Foundation; either version 2 of the License, or (at your option) any later
 version.

 This program is distributed in the hope that it will be useful, but WITHOUT
 ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
 details.

 You should have received a copy of the GNU Lesser General Public License along with
 this program; if not, write to the Free Software Foundation, Inc., 59 Temple
 Place - Suite 330, Boston, MA  02111-1307, USA.

 Further information about the GNU Lesser General Public License can also be found on
 the world wide web at http://www.gnu.org.

FUNCTIONAL DESCRIPTION
--------------------------------------------------------------------------------
This class encapsulates the integration of rates and accelerations to get the
current position of the aircraft.

HISTORY
--------------------------------------------------------------------------------
01/05/99   JSB   Created

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
COMMENTS, REFERENCES,  and NOTES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[1] Cooke, Zyda, Pratt, and McGhee, "NPSNET: Flight Simulation Dynamic Modeling
    Using Quaternions", Presence, Vol. 1, No. 4, pp. 404-420  Naval Postgraduate
    School, January 1994
[2] D. M. Henderson, "Euler Angles, Quaternions, and Transformation Matrices",
    JSC 12960, July 1977
[3] Richard E. McFarland, "A Standard Kinematic Model for Flight Simulation at
    NASA-Ames", NASA CR-2497, January 1975
[4] Barnes W. McCormick, "Aerodynamics, Aeronautics, and Flight Mechanics",
    Wiley & Sons, 1979 ISBN 0-471-03032-5
[5] Bernard Etkin, "Dynamics of Flight, Stability and Control", Wiley & Sons,
    1982 ISBN 0-471-08936-2
[6] S. Buss, "Accurate and Efficient Simulation of Rigid Body Rotations",
    Technical Report, Department of Mathematics, University of California,
    San Diego, 1999
[7] Barker L.E., Bowles R.L. and Williams L.H., "Development and Application of
    a Local Linearization Algorithm for the Integration of Quaternion Rate
    Equations in Real-Time Flight Simulation Problems", NASA TN D-7347,
    December 1973
[8] Phillips W.F, Hailey C.E and Gebert G.A, "Review of Attitude Representations
    Used for Aircraft Kinematics", Journal Of Aircraft Vol. 38, No. 4,
    July-August 2001

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
INCLUDES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <iomanip>

//#include "initialization/FGInitialCondition.h"
#include "FGPropagate.h"
//#include "FGGroundReactions.h"
//#include "FGFDMExec.h"
//#include "input_output/FGPropertyManager.h"
//#include "simgear/io/iostreams/sgstream.hxx"

using namespace std;



/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
CLASS IMPLEMENTATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

FGPropagate::FGPropagate()
{

  /// These define the indices use to select the various integrators.
  // eNone = 0, eRectEuler, eTrapezoidal, eAdamsBashforth2, eAdamsBashforth3, eAdamsBashforth4};

  integrator_rotational_rate = eRectEuler;
  integrator_translational_rate = eAdamsBashforth2;
  integrator_rotational_position = eRectEuler;
  integrator_translational_position = eAdamsBashforth3;

  VState.dqPQRidot.resize(5, FGColumnVector3(0.0,0.0,0.0));
  VState.dqUVWidot.resize(5, FGColumnVector3(0.0,0.0,0.0));
  VState.dqInertialVelocity.resize(5, FGColumnVector3(0.0,0.0,0.0));
  VState.dqQtrndot.resize(5, FGQuaternion(0.0,0.0,0.0));

}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

FGPropagate::~FGPropagate(void)
{
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

bool FGPropagate::InitModel(void)
{
 // if (!FGModel::InitModel()) return false;

  // For initialization ONLY:
  VState.vLocation.SetEllipse(in.SemiMajor, in.SemiMinor);
  VState.vLocation.SetAltitudeAGL(4.0);

  VState.dqPQRidot.resize(5, FGColumnVector3(0.0,0.0,0.0));
  VState.dqUVWidot.resize(5, FGColumnVector3(0.0,0.0,0.0));
  VState.dqInertialVelocity.resize(5, FGColumnVector3(0.0,0.0,0.0));
  VState.dqQtrndot.resize(5, FGColumnVector3(0.0,0.0,0.0));

  integrator_rotational_rate = eRectEuler;
  integrator_translational_rate = eAdamsBashforth2;
  integrator_rotational_position = eRectEuler;
  integrator_translational_position = eAdamsBashforth3;

  return true;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void FGPropagate::SetInitialState()
{
  // Initialize the State Vector elements and the transformation matrices

  //caitodo set the initial state value!!!
  // Set the position lat/lon/radius
  VState.vLocation;// = FGIC->GetPosition();

  Ti2ec = VState.vLocation.GetTi2ec(); // ECI to ECEF transform
  Tec2i = Ti2ec.Transposed();          // ECEF to ECI frame transform

  VState.vInertialPosition = Tec2i * VState.vLocation;

  UpdateLocationMatrices();

  // Set the orientation from the euler angles (is normalized within the
  // constructor). The Euler angles represent the orientation of the body
  // frame relative to the local frame.
  VState.qAttitudeLocal; // = FGIC->GetOrientation();

  VState.qAttitudeECI = Ti2l.GetQuaternion()*VState.qAttitudeLocal;
  UpdateBodyMatrices();

  // Set the velocities in the instantaneus body frame
  VState.vUVW = FGColumnVector3(); // = FGIC->GetUVWFpsIC();

  // Compute the local frame ECEF velocity
  vVel = Tb2l * VState.vUVW;

  // Compute local terrain velocity
  RecomputeLocalTerrainVelocity();

  // Set the angular velocities of the body frame relative to the ECEF frame,
  // expressed in the body frame.
  VState.vPQR = FGColumnVector3(); // = FGIC->GetPQRRadpsIC();

  VState.vPQRi = VState.vPQR + Ti2b * in.vOmegaPlanet;

  CalculateInertialVelocity(); // Translational position derivative
  CalculateQuatdot();  // Angular orientation derivative
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Initialize the past value deques

void FGPropagate::InitializeDerivatives()
{
  VState.dqPQRidot.assign(5, in.vPQRidot);
  VState.dqUVWidot.assign(5, in.vUVWidot);
  VState.dqInertialVelocity.assign(5, VState.vInertialVelocity);
  VState.dqQtrndot.assign(5, VState.vQtrndot);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
/*
Purpose: Called on a schedule to perform EOM integration
Notes:   [JB] Run in standalone mode, SeaLevelRadius will be reference radius.
         In FGFS, SeaLevelRadius is stuffed from FGJSBSim in JSBSim.cxx each pass.

At the top of this Run() function, see several "shortcuts" (or, aliases) being
set up for use later, rather than using the longer class->function() notation.

This propagation is done using the current state values
and current derivatives. Based on these values we compute an approximation to the
state values for (now + dt).

In the code below, variables named beginning with a small "v" refer to a
a column vector, variables named beginning with a "T" refer to a transformation
matrix. ECEF refers to Earth Centered Earth Fixed. ECI refers to Earth Centered
Inertial.

*/

bool FGPropagate::Run(bool Holding)
{

   double rate = 1;
  double dt = in.DeltaT * rate;  // The 'stepsize'

  // Propagate rotational / translational velocity, angular /translational position, respectively.

  if (!(dt==0.0)) {
    Integrate(VState.qAttitudeECI,      VState.vQtrndot,      VState.dqQtrndot,          dt, integrator_rotational_position);
    Integrate(VState.vPQRi,             in.vPQRidot,          VState.dqPQRidot,          dt, integrator_rotational_rate);
    Integrate(VState.vInertialPosition, VState.vInertialVelocity, VState.dqInertialVelocity, dt, integrator_translational_position);
    Integrate(VState.vInertialVelocity, in.vUVWidot,          VState.dqUVWidot,          dt, integrator_translational_rate);
  }

  // CAUTION : the order of the operations below is very important to get transformation
  // matrices that are consistent with the new state of the vehicle

  // 1. Update the Earth position angle (EPA)
  VState.vLocation.IncrementEarthPositionAngle(in.vOmegaPlanet(eZ)*dt);

  // 2. Update the Ti2ec and Tec2i transforms from the updated EPA
  Ti2ec = VState.vLocation.GetTi2ec(); // ECI to ECEF transform
  Tec2i = Ti2ec.Transposed();          // ECEF to ECI frame transform

  // 3. Update the location from the updated Ti2ec and inertial position
  VState.vLocation = Ti2ec*VState.vInertialPosition;

  // 4. Update the other "Location-based" transformation matrices from the updated
  //    vLocation vector.
  UpdateLocationMatrices();

  // 5. Update the "Orientation-based" transformation matrices from the updated
  //    orientation quaternion and vLocation vector.
  UpdateBodyMatrices();

  // Translational position derivative (velocities are integrated in the inertial frame)
  CalculateUVW();

  // Set auxilliary state variables
  RecomputeLocalTerrainVelocity();

  VState.vPQR = VState.vPQRi - Ti2b * in.vOmegaPlanet;

  // Angular orientation derivative
  CalculateQuatdot();

  VState.qAttitudeLocal = Tl2b.GetQuaternion();

  // Compute vehicle velocity wrt ECEF frame, expressed in Local horizontal frame.
  vVel = Tb2l * VState.vUVW;


  return false;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void FGPropagate::SetHoldDown(bool hd)
{
  if (hd) {
    VState.vUVW.InitMatrix();
    CalculateInertialVelocity();
    VState.vPQR.InitMatrix();
    VState.vPQRi = Ti2b * in.vOmegaPlanet;
    CalculateQuatdot();
    InitializeDerivatives();
  }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Compute the quaternion orientation derivative
//
// vQtrndot is the quaternion derivative.
// Reference: See Stevens and Lewis, "Aircraft Control and Simulation",
//            Second edition (2004), eqn 1.5-16b (page 50)

void FGPropagate::CalculateQuatdot(void)
{
  // Compute quaternion orientation derivative on current body rates
  VState.vQtrndot = VState.qAttitudeECI.GetQDot(VState.vPQRi);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // Transform the velocity vector of the body relative to the origin (Earth
  // center) to be expressed in the inertial frame, and add the vehicle velocity
  // contribution due to the rotation of the planet.
  // Reference: See Stevens and Lewis, "Aircraft Control and Simulation",
  //            Second edition (2004), eqn 1.5-16c (page 50)

void FGPropagate::CalculateInertialVelocity(void)
{
  VState.vInertialVelocity = Tb2i * VState.vUVW + (in.vOmegaPlanet * VState.vInertialPosition);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // Transform the velocity vector of the inertial frame to be expressed in the
  // body frame relative to the origin (Earth center), and substract the vehicle
  // velocity contribution due to the rotation of the planet.

void FGPropagate::CalculateUVW(void)
{
  VState.vUVW = Ti2b * (VState.vInertialVelocity - (in.vOmegaPlanet * VState.vInertialPosition));
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void FGPropagate::Integrate( FGColumnVector3& Integrand,
                             FGColumnVector3& Val,
                             deque <FGColumnVector3>& ValDot,
                             double dt,
                             eIntegrateType integration_type)
{
  ValDot.push_front(Val);
  ValDot.pop_back();

  switch(integration_type) {
  case eRectEuler:       Integrand += dt*ValDot[0];
    break;
  case eTrapezoidal:     Integrand += 0.5*dt*(ValDot[0] + ValDot[1]);
    break;
  case eAdamsBashforth2: Integrand += dt*(1.5*ValDot[0] - 0.5*ValDot[1]);
    break;
  case eAdamsBashforth3: Integrand += (1/12.0)*dt*(23.0*ValDot[0] - 16.0*ValDot[1] + 5.0*ValDot[2]);
    break;
  case eAdamsBashforth4: Integrand += (1/24.0)*dt*(55.0*ValDot[0] - 59.0*ValDot[1] + 37.0*ValDot[2] - 9.0*ValDot[3]);
    break;
  case eAdamsBashforth5: Integrand += dt*((1901./720.)*ValDot[0] - (1387./360.)*ValDot[1] + (109./30.)*ValDot[2] - (637./360.)*ValDot[3] + (251./720.)*ValDot[4]);
    break;
  case eNone: // do nothing, freeze translational rate
    break;
  case eBuss1:
  case eBuss2:
  case eLocalLinearization:
    throw("Can only use Buss (1 & 2) or local linearization integration methods in for rotational position!");
  default:
    break;
  }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void FGPropagate::Integrate( FGQuaternion& Integrand,
                             FGQuaternion& Val,
                             deque <FGQuaternion>& ValDot,
                             double dt,
                             eIntegrateType integration_type)
{
  ValDot.push_front(Val);
  ValDot.pop_back();

  switch(integration_type) {
  case eRectEuler:       Integrand += dt*ValDot[0];
    break;
  case eTrapezoidal:     Integrand += 0.5*dt*(ValDot[0] + ValDot[1]);
    break;
  case eAdamsBashforth2: Integrand += dt*(1.5*ValDot[0] - 0.5*ValDot[1]);
    break;
  case eAdamsBashforth3: Integrand += (1/12.0)*dt*(23.0*ValDot[0] - 16.0*ValDot[1] + 5.0*ValDot[2]);
    break;
  case eAdamsBashforth4: Integrand += (1/24.0)*dt*(55.0*ValDot[0] - 59.0*ValDot[1] + 37.0*ValDot[2] - 9.0*ValDot[3]);
    break;
  case eAdamsBashforth5: Integrand += dt*((1901./720.)*ValDot[0] - (1387./360.)*ValDot[1] + (109./30.)*ValDot[2] - (637./360.)*ValDot[3] + (251./720.)*ValDot[4]);
    break;
  case eBuss1:
    {
      // This is the first order method as described in Samuel R. Buss paper[6].
      // The formula from Buss' paper is transposed below to quaternions and is
      // actually the exact solution of the quaternion differential equation
      // qdot = 1/2*w*q when w is constant.
      Integrand = Integrand * QExp(0.5 * dt * VState.vPQRi);
    }
    return; // No need to normalize since the quaternion exponential is always normal
  case eBuss2:
    {
      // This is the 'augmented second-order method' from S.R. Buss paper [6].
      // Unlike Runge-Kutta or Adams-Bashforth, it is a one-pass second-order
      // method (see reference [6]).
      FGColumnVector3 wi = VState.vPQRi;
      FGColumnVector3 wdoti = in.vPQRidot;
      FGColumnVector3 omega = wi + 0.5*dt*wdoti + dt*dt/12.*wdoti*wi;
      Integrand = Integrand * QExp(0.5 * dt * omega);
    }
    return; // No need to normalize since the quaternion exponential is always normal
  case eLocalLinearization:
    {
      // This is the local linearization algorithm of Barker et al. (see ref. [7])
      // It is also a one-pass second-order method. The code below is based on the
      // more compact formulation issued from equation (107) of ref. [8]. The
      // constants C1, C2, C3 and C4 have the same value than those in ref. [7] pp. 11
      FGColumnVector3 wi = 0.5 * VState.vPQRi;
      FGColumnVector3 wdoti = 0.5 * in.vPQRidot;
      double omegak2 = DotProduct(VState.vPQRi, VState.vPQRi);
      double omegak = omegak2 > 1E-6 ? sqrt(omegak2) : 1E-6;
      double rhok = 0.5 * dt * omegak;
      double C1 = cos(rhok);
      double C2 = 2.0 * sin(rhok) / omegak;
      double C3 = 4.0 * (1.0 - C1) / (omegak*omegak);
      double C4 = 4.0 * (dt - C2) / (omegak*omegak);
      FGColumnVector3 Omega = C2*wi + C3*wdoti + C4*wi*wdoti;
      FGQuaternion q;

      q(1) = C1 - C4*DotProduct(wi, wdoti);
      q(2) = Omega(eP);
      q(3) = Omega(eQ);
      q(4) = Omega(eR);

      Integrand = Integrand * q;

      /* Cross check with ref. [7] pp.11-12 formulas and code pp. 20
      double pk = VState.vPQRi(eP);
      double qk = VState.vPQRi(eQ);
      double rk = VState.vPQRi(eR);
      double pdotk = in.vPQRidot(eP);
      double qdotk = in.vPQRidot(eQ);
      double rdotk = in.vPQRidot(eR);
      double Ap = -0.25 * (pk*pdotk + qk*qdotk + rk*rdotk);
      double Bp = 0.25 * (pk*qdotk - qk*pdotk);
      double Cp = 0.25 * (pdotk*rk - pk*rdotk);
      double Dp = 0.25 * (qk*rdotk - qdotk*rk);
      double C2p = sin(rhok) / omegak;
      double C3p = 2.0 * (1.0 - cos(rhok)) / (omegak*omegak);
      double H = C1 + C4 * Ap;
      double G = -C2p*rk - C3p*rdotk + C4*Bp;
      double J = C2p*qk + C3p*qdotk - C4*Cp;
      double K = C2p*pk + C3p*pdotk - C4*Dp;

      cout << "q:       " << q << endl;

      // Warning! In the paper of Barker et al. the quaternion components are not
      // ordered the same way as in JSBSim (see equations (2) and (3) of ref. [7]
      // as well as the comment just below equation (3))
      cout << "FORTRAN: " << H << " , " << K << " , " << J << " , " << -G << endl;*/
    }
    break; // The quaternion q is not normal so the normalization needs to be done.
  case eNone: // do nothing, freeze rotational rate
    break;
  default:
    break;
  }

  Integrand.Normalize();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void FGPropagate::UpdateLocationMatrices(void)
{
  Tl2ec = VState.vLocation.GetTl2ec(); // local to ECEF transform
  Tec2l = Tl2ec.Transposed();          // ECEF to local frame transform
  Ti2l  = VState.vLocation.GetTi2l();  // ECI to local frame transform
  Tl2i  = Ti2l.Transposed();           // local to ECI transform
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void FGPropagate::UpdateBodyMatrices(void)
{
  Ti2b  = VState.qAttitudeECI.GetT(); // ECI to body frame transform
  Tb2i  = Ti2b.Transposed();          // body to ECI frame transform
  Tl2b  = Ti2b * Tl2i;                // local to body frame transform
  Tb2l  = Tl2b.Transposed();          // body to local frame transform
  Tec2b = Ti2b * Tec2i;               // ECEF to body frame transform
  Tb2ec = Tec2b.Transposed();         // body to ECEF frame tranform

  Qec2b = Tec2b.GetQuaternion();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void FGPropagate::SetInertialOrientation(const FGQuaternion& Qi)
{
  VState.qAttitudeECI = Qi;
  VState.qAttitudeECI.Normalize();
  UpdateBodyMatrices();
  VState.qAttitudeLocal = Tl2b.GetQuaternion();
  CalculateQuatdot();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void FGPropagate::SetInertialVelocity(const FGColumnVector3& Vi) {
  VState.vInertialVelocity = Vi;
  CalculateUVW();
  vVel = Tb2l * VState.vUVW;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void FGPropagate::SetInertialRates(const FGColumnVector3& vRates) {
  VState.vPQRi = Ti2b * vRates;
  VState.vPQR = VState.vPQRi - Ti2b * in.vOmegaPlanet;
  CalculateQuatdot();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void FGPropagate::RecomputeLocalTerrainVelocity()
{
  FGLocation contact;
  FGColumnVector3 normal;
  VState.vLocation.GetContactPoint(contact, normal, LocalTerrainVelocity,
                                   LocalTerrainAngularVelocity);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void FGPropagate::SetTerrainElevation(double terrainElev)
{
  double radius = terrainElev + VState.vLocation.GetSeaLevelRadius();
  //caitodo
  //FDMExec->GetGroundCallback()->SetTerrainGeoCentRadius(radius);
}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void FGPropagate::SetSeaLevelRadius(double tt)
{
    //todo
  //FDMExec->GetGroundCallback()->SetSeaLevelRadius(tt);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double FGPropagate::GetLocalTerrainRadius(void) const
{
  return VState.vLocation.GetTerrainRadius();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double FGPropagate::GetDistanceAGL(void) const
{
  return VState.vLocation.GetAltitudeAGL();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double FGPropagate::GetDistanceAGLKm(void) const
{
  return VState.vLocation.GetAltitudeAGL()*0.0003048;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void FGPropagate::SetDistanceAGL(double tt)
{
  VState.vLocation.SetAltitudeAGL(tt);
  UpdateVehicleState();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void FGPropagate::SetDistanceAGLKm(double tt)
{
  VState.vLocation.SetAltitudeAGL(tt*3280.8399);
  UpdateVehicleState();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void FGPropagate::SetVState(const VehicleState& vstate)
{
  //ToDo: Shouldn't all of these be set from the vstate vector passed in?
  VState.vLocation = vstate.vLocation;
  Ti2ec = VState.vLocation.GetTi2ec(); // useless ?
  Tec2i = Ti2ec.Transposed();
  UpdateLocationMatrices();
  SetInertialOrientation(vstate.qAttitudeECI);
  RecomputeLocalTerrainVelocity();
  VState.vUVW = vstate.vUVW;
  vVel = Tb2l * VState.vUVW;
  VState.vPQR = vstate.vPQR;
  VState.vPQRi = VState.vPQR + Ti2b * in.vOmegaPlanet;
  VState.vInertialPosition = vstate.vInertialPosition;
  CalculateQuatdot();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void FGPropagate::UpdateVehicleState(void)
{
  RecomputeLocalTerrainVelocity();
  VState.vInertialPosition = Tec2i * VState.vLocation;
  UpdateLocationMatrices();
  UpdateBodyMatrices();
  vVel = Tb2l * VState.vUVW;
  VState.qAttitudeLocal = Tl2b.GetQuaternion();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void FGPropagate::SetLocation(const FGLocation& l)
{
  VState.vLocation = l;
  Ti2ec = VState.vLocation.GetTi2ec(); // useless ?
  Tec2i = Ti2ec.Transposed();
  UpdateVehicleState();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

FGColumnVector3 FGPropagate::GetEulerDeg(void) const
{
  return VState.qAttitudeLocal.GetEuler() * radtodeg;
}


