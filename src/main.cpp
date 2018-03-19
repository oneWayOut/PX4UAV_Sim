#include <iostream>
#include "FGPropagate.h"

using namespace std;

int main()
{
	FGPropagate* Propagate = new FGPropagate();

    //initialize all input values
	Propagate->in.vOmegaPlanet = GetOmegaPlanet();
    Propagate->in.SemiMajor = GetSemimajor();
    Propagate->in.SemiMinor = GetSemiminor();
    Propagate->in.vPQRidot  = FGColumnVector3();  //     = Accelerations->GetPQRidot();
    Propagate->in.vUVWidot  = FGColumnVector3();  //     = Accelerations->GetUVWidot();
    Propagate->in.DeltaT = 0.01;  //       = dT;

    //initialize modle state;
    Propagate->InitModel();

    Propagate->SetInitialState();

    Propagate->InitializeDerivatives();

	cout << "Hello World!" << endl;

    double duration = 0;
    double thisTime = 0;
    while(thisTime < duration)
    {
        //in.vPQRidot,          VState.dqPQRidot,          dt, integrator_rotational_rate);
        //Integrate(VState.vInertialPosition, VState.vInertialVelocity, VState.dqInertialVelocity, dt, integrator_translational_position);
        //Integrate(VState.vInertialVelocity, in.vUVWidot,

        //caitodo get in.vPQRidot and in.vUVWidot at each cycle;
        //or put the assignment inside Propagate->Run() function;



        Propagate->Run(true);
        thisTime += Propagate->in.DeltaT;
    }

    delete Propagate;
    return 0;
}

