/*
    Quaternion arithmetic and utility functions
    07/23/19 Matt Vredevoogd
*/

#include <Arduino.h>
#include <Quaternion.h>

Quaternion::Quaternion() {
    w = 1; x = 0; y = 0; z = 0;
}

Quaternion::Quaternion(float quat[4]) {
    w = quat[0];
    x = quat[1];
    y = quat[2];
    z = quat[3];
}

Quaternion::Quaternion(float _w, float _x, float _y, float _z) {
    w = _w; x = _x; y = _y; z = _z;
    normalize();
}

Quaternion::Quaternion(float _x, float _y, float _z) {
    w = sqrtf(1 - _x * _x - _y * _y - _z * _z);
    x = _x;
    y = _y;
    z = _z;
}

float Quaternion::norm() {
    float sq = w * w + x * x + y * y + z * z;
    return sqrt(sq);
}

void Quaternion::normalize() {
    float length = norm();
    w = w / length;
    x = x / length;
    y = y / length;
    z = z / length;
}

Quaternion Quaternion::conjugate() {
    Quaternion result(w, -x, -y, -z);
    return result;
}

Quaternion Quaternion::inverse() {
    Quaternion conj = conjugate();
    Quaternion result;
    float length = norm();
    result = conj / (length * length);
    return result;
}

Quaternion Quaternion::operator + (Quaternion const & quat) {
    Quaternion result;
    result.x = w + quat.w;
    result.x = x + quat.x;
    result.y = y + quat.y;
    result.z = z + quat.z;
    return result;
}

Quaternion Quaternion::operator - (Quaternion const & quat) {
    Quaternion result;
    result.x = w - quat.w;
    result.x = x - quat.x;
    result.y = y - quat.y;
    result.z = z - quat.z;
    return result;
}

Quaternion Quaternion::operator * (float const & scalar) {
    Quaternion result;
    result.w = w * scalar;
    result.x = x * scalar;
    result.y = y * scalar;
    result.z = z * scalar;
    return result;
}

Quaternion Quaternion::operator / (float const & scalar) {
    Quaternion result;
    result.w = w / scalar;
    result.x = x / scalar;
    result.y = y / scalar;
    result.z = z / scalar;
    return result;
}

Quaternion Quaternion::rotateBy(Quaternion qr) {
    Quaternion result;
    float t0 = w * qr.y - x * qr.z - y * qr.w + z * qr.x;
    float t1 = w * qr.z + x * qr.y - y * qr.x - z * qr.w;
    float t2 = w * qr.x - x * qr.w + y * qr.z - z * qr.y;
    float t3 = w * qr.w + x * qr.x + y * qr.y + z * qr.z;
    result.w = t0 * qr.y + t2 * qr.x + t3 * qr.w + t1 * qr.z;
    result.x = -t2 * qr.w + t0 * qr.z - t1 * qr.y + t3 * qr.x;
    result.y = -t0 * qr.w + t1 * qr.x - t2 * qr.z + t3 * qr.y;
    result.z = -t0 * qr.x - t1 * qr.w + t2 * qr.y + t3 * qr.z;
}

float Quaternion::yawDeg() {
    float yaw = 57.29577951308232 * atan2f(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    yaw = (yaw > 360.0) ? yaw - 360.0 : (yaw < 0) ? yaw + 360 : yaw;
    return yaw;
}


// static Quaternion Quaternion::quaternionError(Quaternion qr, Quaternion q) {
//     // rotation from q to target quaternion qt
//     Quaternion result;
//     q.w = q.w * q.w-q.x * q.x-y * y-q.z * q.z;
//     q.x = q.w * q.x * 2.0;
//     y = q.w * y * 2.0;
//     q.z = q.w * q.z * 2.0; 
//     return result;
// }

