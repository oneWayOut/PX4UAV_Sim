#include <cmath>

#include "ForceMoments.h"


ForceMoments::ForceMoments()
{
    //Refer to "Small Unmmaned Aircraft", 2012  Page276
    aero_coef.S_wing        = 0.55;
    aero_coef.b             = 2.8956;
    aero_coef.c             = 0.18994;
    aero_coef.S_prop        = 0.2027;
    aero_coef.rho           = 1.2682;
    aero_coef.k_motor       = 80;
    aero_coef.k_T_P         = 0;
    aero_coef.k_Omega       = 0;
    aero_coef.e             = 0.9;

    aero_coef.C_L_0         = 0.28;
    aero_coef.C_L_alpha     = 3.45;
    aero_coef.C_L_q         = 0.0;
    aero_coef.C_L_delta_e   = -0.36;
    aero_coef.C_D_0         = 0.03;
    aero_coef.C_D_alpha     = 0.30;
    aero_coef.C_D_p         = 0.0437;
    aero_coef.C_D_q         = 0.0;
    aero_coef.C_D_delta_e   = 0.0;
    aero_coef.C_m_0         = -0.02338;
    aero_coef.C_m_alpha     = -0.38;
    aero_coef.C_m_q         = -3.6;
    aero_coef.C_m_delta_e   = -0.5;
    aero_coef.C_Y_0         = 0.0;
    aero_coef.C_Y_beta      = -0.98;
    aero_coef.C_Y_p         = 0.0;
    aero_coef.C_Y_r         = 0.0;
    aero_coef.C_Y_delta_a   = 0.0;
    aero_coef.C_Y_delta_r   = -0.17;
    aero_coef.C_ell_0       = 0.0;
    aero_coef.C_ell_beta    = -0.12;
    aero_coef.C_ell_p       = -0.26;
    aero_coef.C_ell_r       = 0.14;
    aero_coef.C_ell_delta_a = 0.08;
    aero_coef.C_ell_delta_r = 0.105;
    aero_coef.C_n_0         = 0.0;
    aero_coef.C_n_beta      = 0.25;
    aero_coef.C_n_p         = 0.022;
    aero_coef.C_n_r         = -0.35;
    aero_coef.C_n_delta_a   = 0.06;
    aero_coef.C_n_delta_r   = -0.032;
    aero_coef.C_prop        = 1.0;
    aero_coef.M             = 50;
    aero_coef.epsilon       = 0.1592;
    aero_coef.alpha0        = 0.4712;
}

FGColumnVector3 ForceMoments::GetForces()
{
    //Refer to "Small Unmmaned Aircraft", 2012  Page57
    double sin_alpha = sin(in.alpha);
    double cos_alpha = cos(in.alpha);

    FGColumnVector3 Force;
    //todo calculate forces
    return Force;

}

FGColumnVector3 ForceMoments::GetMoments()
{
    //Refer to "Small Unmmaned Aircraft", 2012  Page58
    FGColumnVector3 Moment;
    //todo calculate moments
    return Moment;
}
