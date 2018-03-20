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

    //todo other initialize

	cout << "Hello World!" << endl;

    FGColumnVector3 Forces;
    FGColumnVector3 Moments;
    double          Mass = 1.0;
    // The body inertia matrix expressed in the body frame
    FGMatrix33      J(0.8244, 0    , -0.1204,
                      0     , 1.135, 0,
                      0.1204, 0    , 1.759);

    double duration = 0;
    double thisTime = 0;
    while(thisTime < duration)
    {
        //caitodo Calculate forces and moments in body frame;
        //Using formulas from Small unmmaned aircraft.



        Propagate->Run(Mass, J, Forces, Moments);
        thisTime += Propagate->in.DeltaT;
    }

    delete Propagate;
    return 0;
}

