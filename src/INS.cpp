#include "INS.h"
#include <BasicLinearAlgebra.h>
using namespace BLA;


INS::INS(float _dt):dt(9.81 / _dt ){
    x << 0, 0, 0;
    g << 0, 0, 1;
}

void INS::step(const float& ax, const float& ay, const float& az){
    Matrix<3, 1> al;
    al << ax, ay, az;
    x += (al + g)*dt;
    stepcount++;
}