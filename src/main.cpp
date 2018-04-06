#include <iostream>
#include <fstream>
#include "FGPropagate.h"

#include "ForceMoments.h"

using namespace std;

//initial state
double longitude = 90;
double latitude = 0;
double alt = 0;

//euler angles in degree
double phi = 0;
double theta = 45;
double psi = 45;

//uvw velocity in body frame
FGColumnVector3 vUVW_body(200, 0, 0);
//pqr Note: derived value!
FGColumnVector3 vPQR_body;

const double     Mass = 1.0;
// The body inertia matrix expressed in the body frame
const FGMatrix33 J(20, 0 , 0,
                   0 ,10, 0,
                   0, 0 , 10);


FGColumnVector3 calcPQR_body(const FGQuaternion& orientation, const FGLocation& position,
                             const FGColumnVector3& uvwBody)
{
    // Refer to Stevens and Lewis, 1.5-14a, pg. 49.
    // This is the rotation rate of the "Local" frame, expressed in the local frame.
    const FGMatrix33& Tb2l = orientation.GetTInv();
    const FGMatrix33& Tl2b = orientation.GetT();

    FGColumnVector3 vUVW_NED = Tb2l*uvwBody;

    double radInv = 1.0 / position.GetRadius();
    FGColumnVector3 vOmegaLocal = FGColumnVector3(
                                                  radInv*vUVW_NED(eEast),
                                                  -radInv*vUVW_NED(eNorth),
                                                  -radInv*vUVW_NED(eEast)*position.GetTanLatitude() );

    return Tl2b * vOmegaLocal;
}


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

    FGLocation position;
    position.SetEllipse(GetSemimajor(), GetSemiminor());
    position.SetLongitude(longitude * degtorad);
    position.SetAltitudeASL(alt);            //above sea level
    position.SetLatitude(latitude * degtorad);

    FGQuaternion orientation(phi*degtorad, theta*degtorad, psi*degtorad);

    vPQR_body = calcPQR_body(orientation, position, vUVW_body);

    Propagate->SetInitialState(position, orientation, vUVW_body, vPQR_body);

    Propagate->InitializeDerivatives();

    //todo any other initialize??

	cout << "Hello World!" << endl;

    std::ofstream out("out.txt");
    std::streambuf *coutbuf = std::cout.rdbuf(); //save old buf
    std::cout.rdbuf(out.rdbuf()); //redirect std::cout to out.txt!


    ForceMoments f_m;

    double duration = 30.0;
    double thisTime = 0;
    while(thisTime < duration)
    {
        //caitodo Calculate forces and moments in body frame;
        //Using formulas from Small unmmaned aircraft.

        //todo  create a new thread to recv PX4 msg
        //f_m.in.delta_a; delta_r, delta_e, delta_t from PX4 firmware actuator output
        f_m.in.vPQR = Propagate->GetPQR();
        //alpha and beta coulde be get from velocity vector;

        FGColumnVector3 Forces = f_m.GetForces();
        FGColumnVector3 Moments = f_m.GetMoments();

        Propagate->Run(Mass, J, Forces, Moments);
        thisTime += Propagate->in.DeltaT;



        cout<<"time = " << thisTime << ",  ";
        cout<< "ASL = " << Propagate->GetAltitudeASL()<< ","<<endl;


        if(Propagate->GetAltitudeASL()<=0)
            break;
    }

    std::cout.rdbuf(coutbuf); //reset to standard output again

    delete Propagate;
    return 0;
}

