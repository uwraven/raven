#include <AttitudeKalmanFilter.h>
#include <BasicLinearAlgebra.h>
#include <BasicLinearAlgebraOptimized.h>
#include <LSM303.h>
#include <L3G.h>
#include <Vec3.h>
#include <Wire.h>

using namespace BLA;

AttitudeKalmanFilter ekf;
LSM303 compacc;
L3G gyro;

#define TWA_FAST_MODE 400000

void setup() {
      delay(1000);


    Serial.begin(9600);
    
    Wire.begin();
    Wire.setSDA(18);
    Wire.setSCL(19);
    Wire.setClock(TWA_FAST_MODE);

    ekf.init();
    compacc.init();
    gyro.init();

    compacc.enableDefault();
    gyro.enableDefault();

    gyro.read();

    gyro.writeReg(L3G::CTRL_REG1, 0xDF);
    
}

void loop() {
    // updateFilter();
    // BLA::Matrix<7> X = ekf.getState();

    float loopDuration;

    for (int i = 0; i < 5000; i++) {
        uint64_t t_start = micros();

        byte ctrReg = gyro.readReg(L3G::STATUS);

        int x_ready = (ctrReg & 0x01) >> 0;
        int y_ready = (ctrReg & 0x02) >> 1;
        int z_ready = (ctrReg & 0x04) >> 2;

        if (x_ready & y_ready & z_ready) {
            Serial.print(1); Serial.print(" ");
            gyro.read();

            Serial.print((float)gyro.g.x / 114.0); Serial.print(" ");
            Serial.print((float)gyro.g.y / 114.0); Serial.print(" ");
            Serial.print((float)gyro.g.z / 114.0); Serial.print(" ");

        } else {
            Serial.print(0); Serial.print(" ");
        }

        // Serial.print(ctrReg); Serial.print(" ");

        // printBits(ctrReg); Serial.print(" ");
        // printBits(ctrReg2); Serial.print(" ");

        // Serial.print(x_ready); Serial.print(" ");
        // Serial.print(y_ready); Serial.print(" ");
        // Serial.print(z_ready); Serial.print(" ");

        uint64_t t_end = micros();
        float dt = (float)t_end - (float)t_start;
        loopDuration += dt;


        Serial.println(dt);
        // Serial.println();
    }

    loopDuration /= 5000;

    Serial.println(loopDuration);

    while(1);

    // if (ekf.getCycleCount() % 100 == 0) {
    //     Serial << X;
    //     Serial.println();
    // }

    // if (ekf.getCycleCount() > 20000) {
    //     Serial.print(ekf.getAverageCycleDuration());
    //     while(1);
    // }

}

void updateFilter() {

    Vec3 w_m(gyro.g.x, gyro.g.y, gyro.g.z);
    Vec3 a_m(compacc.a.x, compacc.a.y, compacc.a.z);
    float yaw_m = compacc.heading();

    w_m /= 114.0;
    a_m /= 16384.0;

    ekf.main(w_m, a_m, yaw_m);
}

void printBits(byte data) {
    for (byte mask = 0x80; mask; mask >>= 1) {
        if (mask & data) {
            Serial.print('1');
        } else {
            Serial.print('0');
        }
    }
}
