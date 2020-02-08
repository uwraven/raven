/*
    R3 Vector arithmetic and utility functions
    07/22/19 Matt Vredevoogd
*/

#ifndef Vec3_H
#define Vec3_H

#include <Arduino.h>
#include <Quaternion.h>


class Vec3 {
    public:

        Vec3(float _x, float _y, float _z);
        Vec3(float vec[3]);
        Vec3();

        float x;
        float y;
        float z;

        float norm();
        void normalize();

        Vec3 operator+ (Vec3 const & vec2);
        Vec3 operator- (Vec3 const & vec2);
        Vec3 operator* (float const & prod);
        Vec3 operator/ (float const & denom);

        Vec3& operator+= (Vec3 const & vec2);
        Vec3& operator-= (Vec3 const & vec2);
        Vec3& operator*= (float const & prod);
        Vec3& operator/= (float const & denom);

        Vec3 rotateBy(Quaternion q);
        Vec3 reverseRotateBy(Quaternion q);

        static Quaternion quaternionRotation(Vec3 vf, Vec3 vt);
        static Vec3 cross(Vec3 vec1, Vec3 vec2);
        static float dot(Vec3 vec1, Vec3 vec2);
        
};

#endif
