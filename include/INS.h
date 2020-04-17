#include <BasicLinearAlgebra.h>

class INS{
    private:
    // state transition
    float dt;
    // state
    BLA::Matrix<3, 1> g;

    public:
    BLA::Matrix<3, 1> x;
    
    int stepcount;

    INS(float _dt);
    void step(const float& ax, const float& ay, const float& az);

};