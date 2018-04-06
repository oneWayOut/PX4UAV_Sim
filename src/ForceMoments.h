/**
 *
 * define aircraft parameters and calculate forces and moments act on it.
 * Refer to Randal W. Beard and Timothy W. McLain "Small Unmmaned Aircraft", 2012
 *
 */


#ifndef FORCEMOMENTS_H
#define FORCEMOMENTS_H

#include "FGColumnVector3.h"

class ForceMoments{
public:
    struct Inputs {
      FGColumnVector3 vPQR; //pqr
      double alpha;  //angle of attack
      double beta;   //side slip
      double Va;     //air speed
      double delta_a;  //aileron deflection
      double delta_e;   //elevator defection
      double delta_r;  //rudder deflection
      double delta_t;  //throttle command
    } in;

    ForceMoments();

    FGColumnVector3 GetForces();
    FGColumnVector3 GetMoments();


private:
    struct AERO_COEF {
        double S_wing;
        double b;
        double c;
        double S_prop;
        double rho;
        double k_motor;
        double k_T_P;
        double k_Omega;
        double e;

        double C_L_0;
        double C_L_alpha;
        double C_L_q;
        double C_L_delta_e;
        double C_D_0;
        double C_D_alpha;
        double C_D_p;
        double C_D_q;
        double C_D_delta_e;
        double C_m_0;
        double C_m_alpha;
        double C_m_q;
        double C_m_delta_e;
        double C_Y_0;
        double C_Y_beta;
        double C_Y_p;
        double C_Y_r;
        double C_Y_delta_a;
        double C_Y_delta_r;
        double C_ell_0;
        double C_ell_beta;
        double C_ell_p;
        double C_ell_r;
        double C_ell_delta_a;
        double C_ell_delta_r;
        double C_n_0;
        double C_n_beta;
        double C_n_p;
        double C_n_r;
        double C_n_delta_a;
        double C_n_delta_r;
        double C_prop;
        double M;
        double epsilon;
        double alpha0;
    };
    struct AERO_COEF aero_coef;



};

#endif
