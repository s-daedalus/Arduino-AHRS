#include "INS.h"


using namespace BLA;


INS::INS(float _freq):dt(9.81 / _freq ), v_ias(0){
    x << 0, 0, 0;
    g << 0, 0, 1;
    ahrs.begin(_freq);
}

void INS::step(const float& ax, const float& ay, const float& az){
    Matrix<3, 1> al;
    al << ax, ay, az;
    x += (al + g)*dt;
    stepcount++;
}

void INS::predict(const float& wx, const float& wy, const float& wz,
                  const float& ax, const float& ay, const float& az,
                  const float& mx, const float& my, const float& mz)
{
    // todo: move centripedal compensation over
    float gx = -0.101936799f * ax;
    float gy = -0.101936799f * ay - v_ias * wz * 0.0174533f;
    float gz = -0.101936799f * az + v_ias * wy *  0.0174533f;
    
    ahrs.update(wx, wy, wz, gx, gy, gz, mx, my, mz);
    float nax, nay, naz;
    ahrs.RotateVector(ax, ay, az, nax, nay, naz);
    step(nax, nay, naz);

        // correct would be:
    // maybe trans
    te = sqrtf(x(0,0) * x(0,0) + x(1,0) * x(1,0)) * ax - x(2,0) ;
}

void INS::provideAirspeed(const float& v_ias){
    this->v_ias = v_ias;
}

float INS::getTE(){
    return te;
}
