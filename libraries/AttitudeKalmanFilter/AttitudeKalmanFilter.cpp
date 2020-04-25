#include <AttitudeKalmanFilter.h>
#include <BasicLinearAlgebra.h>
#include <BasicLinearAlgebraOptimized.h>
#include <Quaternion.h>
#include <Utility.h>
#include <Vec3.h>

AttitudeKalmanFilter::AttitudeKalmanFilter() {
    // Constructor
}

// Primary initialization method
// - Setup the kalman filter with default gains
bool AttitudeKalmanFilter::init() {
    nominalDeltaT = 500e-6;

    _Qk.delegate = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    _Rk.delegate = {1.0, 1.0, 1.0, 1.0};
    _Qk *= nominalDeltaT;
    _Rk *= nominalDeltaT;

    for (int i = 0; i < 7; i++) {
        _Pk(i, i) = 100.0 * nominalDeltaT;
    }

    _Xk.delegate = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
}

// Main loop method
// - Cache updated measurement values
// - Update attitude estimate
// - Update filter status
void AttitudeKalmanFilter::main(Vec3 w_m, Vec3 a_m, float yaw) {
    // Get delta t since last cycle and update internal cycle averages
    updateInternalClock();

    // Save new measurement values
    wm = w_m;
    am = a_m;
    yaw_m = yaw;

    // Update the kalman filter attitude estimate
    updateEstimate();

    // Check if filter is converged
    // TODO :: Filter convergence algorithm
    // Use filter innovation and covariance derivatives to determine convergence
}

void AttitudeKalmanFilter::updateEstimate() {
    // Kalman Filter steps:
    // 1) Update the process jacobian (Ja) that models vehicle dynamics with new acceleration and gyro rates
    // 2) Use this jacobian to predict the next state (X) over a small dt
    // 3) Calculate updated error covariances (P) from process jacobian
    // 4) Update the measurement jacobian (Jh) with new acceleration and gyro rates
    // 5) Calculate the Kalman gains (K) from updated jacobians and error covariance matrices
    // 6) Use the Kalman gains to compute an innovation matrix and correct the state prediction made in (2)
    // 7) Use the current error covariances, Kalman gains, and measurement jacobian to calculate updated error covariances

    updateProcessJacobian();                                       // (1) Update Ja
    predictState();                                                // (2) Predict _Xk
    _Pk = _Ja * (_Pk * (~_Ja)) + _Qk;                              // (3) Intermediate _Pk
    updateMeasurementJacobian();                                   // (4) Update Jh
    _K = (_Pk * (~_Jh)) * (_Jh * (_Pk * (~_Jh)) + _Rk).Inverse();  // (5) Compute _K
    correctState();                                                // (6) Correct _Xk
    _Pk = _Pk - _K * (_Jh * _Pk);                                  // (7) Compute new _Pk
}

void AttitudeKalmanFilter::updateInternalClock() {
    uint64_t currentCycleTime = micros();

    // Get the change in time since beginning of last loop
    cycleElapsedTime = currentCycleTime - lastCycleTime;
    if (lastCycleTime == 0) {
        cycleElapsedTime = 0;
    };

    // Add cycle to total cycle count
    cycleCount++;

    // Compute the new cycle duration average
    averageCycleDuration = averageCycleDuration + ((float)cycleElapsedTime - averageCycleDuration) / (float)cycleCount;

    // Compute the running nominal delta t
    // TODO:: update nominal delta t algorithm to short period running average
    nominalDeltaT = averageCycleDuration / 1000000.0;

    // Set the last cycle timestamp to current micros()
    lastCycleTime = currentCycleTime;
}

void AttitudeKalmanFilter::updateProcessJacobian() {
    // Extract quaternion and bias from state
    Quaternion q(_Xk(0), _Xk(1), _Xk(2), _Xk(3));
    Vec3 b(_Xk(4), _Xk(5), _Xk(6));

    float t0 = (wm.y - b.y) * (1.0 / 2.0);
    float t1 = (wm.z - b.z) * (1.0 / 2.0);
    float t2 = (wm.x - b.x) * (1.0 / 2.0);
    float t3 = (b.x - wm.x) * (1.0 / 2.0);
    float t4 = (b.z - wm.z) * (1.0 / 2.0);
    float t5 = (b.y - wm.y) * (1.0 / 2.0);
    float t6 = q.x * (1.0 / 2.0);
    float t7 = q.y * (1.0 / 2.0);
    float t8 = q.z * (1.0 / 2.0);
    _Ja(0, 1) = t3;
    _Ja(0, 2) = t5;
    _Ja(0, 3) = t4;
    _Ja(0, 4) = t6;
    _Ja(0, 5) = t7;
    _Ja(0, 6) = t8;
    _Ja(1, 0) = t2;
    _Ja(1, 2) = t1;
    _Ja(1, 3) = t5;
    _Ja(1, 4) = q.w * (-1.0 / 2.0);
    _Ja(1, 5) = t8;
    _Ja(1, 6) = -t7;
    _Ja(2, 0) = t0;
    _Ja(2, 1) = t4;
    _Ja(2, 3) = t2;
    _Ja(2, 4) = -t8;
    _Ja(2, 5) = q.w * (-1.0 / 2.0);
    _Ja(2, 6) = t6;
    _Ja(3, 0) = t1;
    _Ja(3, 1) = t0;
    _Ja(3, 2) = t3;
    _Ja(3, 4) = t7;
    _Ja(3, 5) = -t6;
    _Ja(3, 6) = q.w * (-1.0 / 2.0);
    _Ja(4, 4) = 1.0;
    _Ja(5, 5) = 1.0;
    _Ja(6, 6) = 1.0;
}

void AttitudeKalmanFilter::updateMeasurementJacobian() {
    // Get updated quaternion estimate
    Quaternion q(_Xk(0), _Xk(1), _Xk(2), _Xk(3));

    float t0 = (q.y * q.y) * 2.0 + (q.z * q.z) * 2.0 - 1.0;
    float t1 = 1.0 / ((q.w * q.w) * (q.z * q.z) * 4.0 + (q.x * q.x) * (q.y * q.y) * 4.0 + t0 * t0 + q.w * q.x * q.y * q.z * 8.0);
    float t2 = q.w * q.z * 2.0 + q.x * q.y * 2.0;
    float t3 = a_ref.z * q.z * 2.0;
    float t4 = a_ref.z * q.x * 2.0;
    float t5 = 1.0 / (t0 * t0);
    float t6 = 1.0 / t0;
    float t7 = 1.0 / t5;
    float t8 = a_ref.z * q.w * 2.0;
    float t9 = a_ref.z * q.y * 2.0;
    _Jh(0, 0) = -t9;
    _Jh(0, 1) = t3;
    _Jh(0, 2) = -t8;
    _Jh(0, 3) = t4;
    _Jh(1, 0) = t4;
    _Jh(1, 1) = t8;
    _Jh(1, 2) = t3;
    _Jh(1, 3) = t9;
    _Jh(2, 0) = t8;
    _Jh(2, 1) = -t4;
    _Jh(2, 2) = -t9;
    _Jh(2, 3) = t3;
    _Jh(3, 0) = t0 * t1 * q.z * -2.0;
    _Jh(3, 1) = t0 * t1 * q.y * -2.0;
    _Jh(3, 2) = -t1 * t7 * (t6 * q.x * 2.0 - t2 * t5 * q.y * 4.0);
    _Jh(3, 3) = -t1 * t7 * (t6 * q.w * 2.0 - t2 * t5 * q.z * 4.0);
}

void AttitudeKalmanFilter::predictState() {
    // Get current attitude quaternion and bias vector estimates
    Quaternion q(_Xk(0), _Xk(1), _Xk(2), _Xk(3));
    Vec3 b(_Xk(4), _Xk(5), _Xk(6));

    // Integrate the attitude quaternion
    Quaternion q_next;
    q_next = quaternionIntegral(q, wm, b, nominalDeltaT);
    _Xk(0) = q_next.w;
    _Xk(1) = q_next.x;
    _Xk(2) = q_next.y;
    _Xk(3) = q_next.z;
    // The angular rate bias does not evolve according to any dynamics
    // therefore the change in _Xk(4-6) is 0
}

void AttitudeKalmanFilter::correctState() {
    // Get most recent attitude estimate
    Quaternion q(_Xk(0), _Xk(1), _Xk(2), _Xk(3));

    // Rotate the global reference vector to the local frame
    Vec3 a_ref_local = a_ref.reverseRotateBy(q);

    // Extract yaw angle from attitude quaternion
    float yaw = q.yawDeg() * DEG2RAD;

    // Normalize measured acceleration, copy to new Vec3 first
    Vec3 amu = am;
    amu.normalize();

    // Compare measured acceleration to transformed reference acceleration vector
    // and compare measured yaw with estimated yaw to produce error vector
    BLA::Matrix<4> Err = {
        amu.x - a_ref_local.x,
        amu.y - a_ref_local.y,
        amu.z - a_ref_local.z,
        angleErr(yaw_m * DEG2RAD, yaw)};

    // Update the innovation matrix from error between measurements and estimations
    _Inn = _K * Err;

    // Sum the current state and innovation
    _Xk += _Inn;

    // Force normalization of quaternion component of state
    float norm = sqrt(_Xk(0) * _Xk(0) + _Xk(1) * _Xk(1) + _Xk(2) * _Xk(2) + _Xk(3) * _Xk(3));
    _Xk(0) /= norm;
    _Xk(1) /= norm;
    _Xk(2) /= norm;
    _Xk(3) /= norm;
}

void AttitudeKalmanFilter::setQGains() {
    // _Qk.delegate = gains;
    // _Qk *= nominalDeltaT;
}

void AttitudeKalmanFilter::setRGains() {
    // _Rk.delegate = gains;
    // _Rk *= nominalDeltaT;
}

void AttitudeKalmanFilter::setInitialErrorCovariance(BLA::Matrix<7> covariance) {
    for (int i = 0; i < 7; i++) {
        _Pk(i, i) = covariance(i) * nominalDeltaT;
    }
}

void AttitudeKalmanFilter::setInitialState(BLA::Matrix<7> state) {
    _Xk = state;
}

void AttitudeKalmanFilter::setReferenceAcceleration(Vec3 a_reference) {
    a_ref = a_reference;
}

BLA::Matrix<7> AttitudeKalmanFilter::getState() {
    return _Xk;
}

bool AttitudeKalmanFilter::isConverged() {
    return converged;
}

uint64_t AttitudeKalmanFilter::getCycleCount() {
    return cycleCount;
}

float AttitudeKalmanFilter::getAverageCycleDuration() {
    return averageCycleDuration;
}
