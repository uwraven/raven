/*
    Quaternion arithmetic and utility functions
    07/23/19 Matt Vredevoogd
*/

#ifndef Quaternion_H
#define Quaternion_H

#include <Arduino.h>

class Quaternion {
    public:
        Quaternion();
        Quaternion(float quat[4]);
        Quaternion(float _w, float _x, float _y, float _z);
        Quaternion(float _x, float _y, float _z);
        
        float w, x, y, z;
        float norm();
        void normalize();

        Quaternion conjugate();
        Quaternion inverse();

        Quaternion operator + (Quaternion const & quat);
        Quaternion operator - (Quaternion const & quat);
        Quaternion operator * (float const & scalar);
        Quaternion operator / (float const & scalar);

        Quaternion rotateBy( Quaternion q);

        float yawDeg();

        // static Quaternion Quaternion::quaternionError(Quaternion q, Quaternion qt);

        // static Quaternion Quaternion::integral(Quaternion q, Vec3 & w, Vec3 & b, float const & dt);

        void print() {
            Serial.print(w, 5); Serial.print(" ");
            Serial.print(x, 5); Serial.print(" ");
            Serial.print(y, 5); Serial.print(" ");
            Serial.print(z, 5);
        }
};

#endif