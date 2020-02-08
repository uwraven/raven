#ifndef ATT_EKF_H
#define ATT_EKF_H

#include <BasicLinearAlgebra.h>
#include <BasicLinearAlgebraOptimized.h>
#include <Quaternion.h>
#include <Vec3.h>
#include <Utility.h>

class AttitudeKalmanFilter {
public:

    AttitudeKalmanFilter();     // Constructor
    bool init();                // Initialization method to be called once

    // Gain initialization methods
    void setQGains();
    void setRGains();
    void setInitialErrorCovariance(BLA::Matrix<7> covariance);
    
    // Set initial state estimate
    void setInitialState(BLA::Matrix<7> state);

    // Set attitude reference vector
    void setReferenceAcceleration(Vec3 a_reference);

    // Lifecycle methods
    void main(Vec3 w_m, Vec3 a_m, float yaw);

    BLA::Matrix<7> getState();

    bool isConverged();

    uint64_t getCycleCount();

    float getAverageCycleDuration();

private:
    BLA::Matrix<7>                          _Xk;     // Attitude state [q bias]
    BLA::Matrix<7,7,Diagonal<7, float>>     _Qk;    // Q gains
    BLA::Matrix<4,4,Diagonal<4, float>>     _Rk;    // R gains
    BLA::Matrix<7,4>                        _K;     // Kalman gains
    BLA::Matrix<7,7>                        _Pk;    // Error covariance
    BLA::Matrix<7,7>                        _Ja;    // Process jacobian
    BLA::Matrix<4,7>                        _Jh;    // Measurement jacobian
    BLA::Matrix<7>                          _Inn;   // Innovation

    // Timing control
    uint64_t cycleCount;
    uint32_t cycleElapsedTime;
    uint64_t lastCycleTime;
    float averageCycleDuration;
    float nominalDeltaT;
    void updateInternalClock();

    // Measurement cache
    Vec3 am;
    Vec3 wm;
    float yaw_m;

    // Reference acceleration vector
    Vec3 a_ref = Vec3(0.0, 0.0, 1.0);

    // Filter status
    bool converged;

    // Methods
    void updateEstimate();
    void updateMeasurementJacobian();
    void updateProcessJacobian();
    void predictState();
    void correctState();

};

#endif