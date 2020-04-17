#include <BasicLinearAlgebra.h>
#include "MadgwickAHRS.h"

/**
 * An AHRS/INS Class.
 * This system uses a right-handed north(x)-east(y)-down(z) coordinate system as a base.
*/
class INS{
    private:
    /**
     * delta time between measurements in seconds.
    */
    float dt;

    float v_ias;
    float te;

    BLA::Matrix<3, 1> g;

    
    void step(const float& ax, const float& ay, const float& az);


    public:

    /**
     * the velocity state in NED-coordinates. [m/s]
    */
    BLA::Matrix<3, 1> x;

    

    /**
     * AHRS instance in use.
    */
    Madgwick ahrs;

    /**
     * the number of iterations performed.
    */
    int stepcount;

    /**
     * Constructor for ins-object.
     * @param _freq : state update frequency [hz].
    */
    INS(float _freq);


    /** 
     * Perform an AHRS / INS prediction step.
     * @param wx : angular velocity around x-axis [rad/sec].
     * @param wy : angular velocity around y-axis [rad/sec].
     * @param wz : angular velocity around z-axis [rad/sec].
     * @param ax : accelerometer measurement in x-axis.
     * @param ay : accelerometer measurement in y-axis.
     * @param az : accelerometer measurement in z-axis.
     * @param mx : magnetic field measurement.
     * @param my : magnetic field measurement.
     * @param mz : magnetic field measurement.
    */
    void predict(const float& wx, const float& wy, const float& wz,
                 const float& ax, const float& ay, const float& az,
                 const float& mx, const float& my, const float& mz);

    void provideAirspeed(const float& v_tas);

    float getTE();

    const Madgwick& getAHRS(){return ahrs;};
    
    

};