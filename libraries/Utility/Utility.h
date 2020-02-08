#ifndef RO_MATH_H
#define RO_MATH_H

#include <Quaternion.h>
#include <Vec3.h>

#define PI 3.141592653589793f
#define PI2 1.570796326794897f
#define DEG2RAD 0.017453292519943f
#define RAD2DEG 57.29577951308232f

#define M_EPSILON 10e-12f
#define MM2M 0.001;
#define M2MM 1000;

static bool isEqual(const float a, const float b) {
    return (fabsf(a - b) < M_EPSILON);
}

static bool isZero(const float a) {
    return (fabsf(a) < M_EPSILON);
}

static float norm2(const float a, const float b) { 
    return sqrt(a * a + b * b);
}

static float norm3(const float a, const float b, const float c) { 
    return sqrt(a * a + b * b + c * c);
};

template<typename T>
static T clamp(const T& a, const T& min, const T& max) {
    return (a < min) ? min : (a > max) ? max : a;
}

static float angleErr(const float a, const float b) {
    float dist = a - b;
    dist = (dist > PI) ? dist - 2 * PI : (dist < -PI) ? dist + 2 * PI : dist;
    return dist;
}

static Quaternion quaternionError(Quaternion qr, Quaternion q) {
    // rotation from q to target quaternion qt
    Quaternion result;
    q.w = q.w * q.w-q.x * q.x-q.y * q.y-q.z * q.z;
    q.x = q.w * q.x * 2.0;
    q.y = q.w * q.y * 2.0;
    q.z = q.w * q.z * 2.0; 
    return result;
}

static Quaternion quaternionIntegral(Quaternion q, Vec3 & w, Vec3 & b, float const & dt) {

    float qm[4][3] = {
        -q.x,  -q.y,  -q.z,
        q.w,   -q.z,  q.y,
        q.z,   q.w,   -q.x,
        -q.y,  q.x,   q.w
    };

    Quaternion _q;

    _q.w = q.w + dt * (-(b.x * qm[0][0] + b.y * qm[0][1] + b.z * qm[0][2]) + qm[0][0] * w.x + qm[0][1] * w.y + qm[0][2] * w.z) * (1.0 / 2.0);
    _q.x = q.x + dt * (-(b.x * qm[1][0] + b.y * qm[1][1] + b.z * qm[1][2]) + qm[1][0] * w.x + qm[1][1] * w.y + qm[1][2] * w.z) * (1.0 / 2.0);
    _q.y = q.y + dt * (-(b.x * qm[2][0] + b.y * qm[2][1] + b.z * qm[2][2]) + qm[2][0] * w.x + qm[2][1] * w.y + qm[2][2] * w.z) * (1.0 / 2.0);
    _q.z = q.z + dt * (-(b.x * qm[3][0] + b.y * qm[3][1] + b.z * qm[3][2]) + qm[3][0] * w.x + qm[3][1] * w.y + qm[3][2] * w.z) * (1.0 / 2.0);

    _q.normalize();
    
    return _q;
}

// These are already defined as macros in arduino
// template<typename T>
// static T& max(const T& a, const T& b) {
//     return (a > b) ? a : b;
// }

// template<typename T>
// static T& min(const T& a, const T& b) {
//     return (b > a) ? a : b;
// }

// float radians(float deg) {
//     return deg * DEG2RAD;
// }

// float degrees(float rad) {
//     return rad * RAD2DEG;
// }

#endif