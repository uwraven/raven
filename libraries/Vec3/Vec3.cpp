/*
    R3 Vector arithmetic and utility functions
    07/22/19 Matt Vredevoogd
*/

#include <Arduino.h>
#include <Vec3.h>
#include <Quaternion.h>

Vec3::Vec3(float _x, float _y, float _z) {
    x = _x;
    y = _y;
    z = _z;
}

Vec3::Vec3() {
    x = 0;
    y = 0;
    z = 0;
}

float Vec3::norm() {
    float sq = x * x + y * y + z * z;
    return sqrt(sq);
}

void Vec3::normalize() {
    float length = norm();
    x = x / length;
    y = y / length;
    z = z / length;
}

Vec3 Vec3::operator + (Vec3 const & vec) {
    Vec3 result;
    result.x = x + vec.x;
    result.y = y + vec.y;
    result.z = z + vec.z;
    return result;
}

Vec3 Vec3::operator - (Vec3 const & vec) {
    Vec3 result;
    result.x = x - vec.x;
    result.y = y - vec.y;
    result.z = z - vec.z;
    return result;
}

Vec3 Vec3::operator * (float const & prod) {
    Vec3 result;
    result.x = x * prod;
    result.y = y * prod;
    result.z = z * prod;
    return result;
}

Vec3 Vec3::operator / (float const & denom) {
    Vec3 result;
    result.x = x / denom;
    result.y = y / denom;
    result.z = z / denom;
    return result;
}

Vec3& Vec3::operator += (Vec3 const & vec) {
    this->x += vec.x;
    this->y += vec.y;
    this->z += vec.z;
}

Vec3& Vec3::operator -= (Vec3 const & vec) {
    this->x -= vec.x;
    this->y -= vec.y;
    this->z -= vec.z;
}

Vec3& Vec3::operator *= (float const & prod) {
    this->x *= prod;
    this->y *= prod;
    this->z *= prod;
}

Vec3& Vec3::operator /= (float const & denom) {
    this->x /= denom;
    this->y /= denom;
    this->z /= denom;
}

Vec3 Vec3::rotateBy(Quaternion q) {
    // transforms self by rotation described by quaternion q
    // converts a vector from body frame to global frame
    Vec3 result;
    float t0 = q.w * y - q.x * z + q.z * x;
    float t1 = q.w * z + q.x * y - q.y * x;
    float t2 = q.w * x + q.y * z - q.z * y;
    float t3 = q.x * x + q.y * y + q.z * z;
    // result.w = t3 * q.w - t2 * q.x - t0 * q.y - t1 * q.z;
    result.x = t2 * q.w + t3 * q.x + t1 * q.y - t0 * q.z;
    result.y = t0 * q.w - t1 * q.x + t3 * q.y + t2 * q.z;
    result.z = t1 * q.w + t0 * q.x - t2 * q.y + t3 * q.z;
    return result;
}

Vec3 Vec3::reverseRotateBy(Quaternion q) {
    // transforms self by the opposite rotation described by q
    Vec3 result;
    float t0 = q.w * y + q.x * z - q.z * x;
    float t1 = q.w * z - q.x * y + q.y * x;
    float t2 = q.w * x - q.y * z + q.z * y;
    float t3 = q.x * x + q.y * y + q.z * z;
    // result.w = -t3 * q.w + t2 * q.x + t0 * q.y + t1 * q.z;
    result.x = t2 * q.w + t3 * q.x - t1 * q.y + t0 * q.z;
    result.y = t0 * q.w + t1 * q.x + t3 * q.y - t2 * q.z;
    result.z = t1 * q.w - t0 * q.x + t2 * q.y + t3 * q.z;
    return result;
}

static Quaternion Vec3::quaternionRotation(Vec3 vf, Vec3 vt) {
    Vec3 cp = Vec3::cross(vf, vt);
    float d = Vec3::dot(vf, vt) + sqrt(abs(vf.norm()) * abs(vt.norm()));
    Quaternion result(d, cp.x, cp.y, cp.z);
    result.normalize();
    return result;
}

static float Vec3::dot(Vec3 vec1, Vec3 vec2) {
    return vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z;
}

static Vec3 Vec3::cross(Vec3 vec1, Vec3 vec2) {
    Vec3 result;
    result.x = vec1.y * vec2.z - vec1.z * vec2.y;
    result.y = vec1.z * vec2.x - vec1.x * vec2.z;
    result.z = vec1.x * vec2.y - vec1.y * vec2.x;
    return result;
}

