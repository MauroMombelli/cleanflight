#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "common/axis.h"
#include "common/maths.h"

#include "flight/imu.h"

#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"
#include "sensors/compass.h"

/*
 * IMU DIRTY WORK
 * TODO: make it configurable
 * */

float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;

float loopTimeSec = 0;

float twoKp = (2.0f * 0.5f);
float twoKi = (2.0f * 0.1625f);


void mahonyAHRSreset(){
    q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;
}

float invSqrt(float x) {
    return 1/sqrtf(x); /* Here we can optimize, maybe */
}

void mahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex = 0, halfey = 0, halfez = 0;
    float qa, qb, qc;

    // Auxiliary variables to avoid repeated arithmetic
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    if(!((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))) {
        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrtf(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        halfex += (my * halfwz - mz * halfwy);
        halfey += (mz * halfwx - mx * halfwz);
        halfez += (mx * halfwy - my * halfwx);
    }

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity and magnetic field
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        // Error is sum of cross product between estimated direction and measured direction of field vectors
        halfex += (ay * halfvz - az * halfvy);
        halfey += (az * halfvx - ax * halfvz);
        halfez += (ax * halfvy - ay * halfvx);

    }
    
    // Compute and apply integral feedback if enabled
    if(twoKi > 0.0f) {
        integralFBx += twoKi * halfex * loopTimeSec;    // integral error scaled by Ki
        integralFBy += twoKi * halfey * loopTimeSec;
        integralFBz += twoKi * halfez * loopTimeSec;
        gx += integralFBx;  // apply integral feedback
        gy += integralFBy;
        gz += integralFBz;
    } else {
        integralFBx = 0.0f; // prevent integral windup
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;

    // Integrate rate of change of quaternion
    gx *= (0.5f * loopTimeSec);     // pre-multiply common factors
    gy *= (0.5f * loopTimeSec);
    gz *= (0.5f * loopTimeSec);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

/*END OF IMU DIRTY WORK*/

float throttleAngleScale;
float gyroScaleRad;

float calculateThrottleAngleScale(uint16_t throttle_correction_angle)
{
    return (1800.0f / M_PIf) * (900.0f / throttle_correction_angle);
}

void imuInit(void){
    smallAngle = 0;
    uint8_t i;
    for (i = 0; i < XYZ_AXIS_COUNT; i++){
        EstG[i] = 0;
        inclinationDecigrees[i] = 0;
    }

    gyroScaleRad = gyro.scale * (M_PIf / 180.0f) * 0.000001f;

    mahonyAHRSreset();
}

void imuConfigure(uint16_t throttle_correction_angle){
    throttleAngleScale = calculateThrottleAngleScale(throttle_correction_angle);
}

void imuResetAccelerationSum(void){

}

void imuUpdate(){
    static uint32_t previousT;
    uint32_t currentT = micros();
    uint32_t deltaT;
    deltaT = currentT - previousT;

    loopTimeSec = deltaT/1000000.0f;

    mahonyAHRSupdate(gyroADC[X] * gyroScaleRad, gyroADC[Y] * gyroScaleRad, gyroADC[Z] * gyroScaleRad, accADC[X], accADC[Y], accADC[Z], magADC[X], magADC[Y], magADC[Z]);
    
    /* TODO: assing results to proper shared variable */
}



int16_t calculateThrottleAngleCorrection(uint8_t throttle_correction_value)
{
    float cosZ = EstG[Z] / sqrtf(EstG[X] * EstG[X] + EstG[Y] * EstG[Y] + EstG[Z] * EstG[Z]);

    /*
    * Use 0 as the throttle angle correction if we are inverted, vertical or with a
    * small angle < 0.86 deg
    * TODO: Define this small angle in config.
    */
    if (cosZ <= 0.015f) {
        return 0;
    }
    int angle = lrintf(acosf(cosZ) * throttleAngleScale);
    if (angle > 900)
        angle = 900;
    return lrintf(throttle_correction_value * sin_approx(angle / (900.0f * M_PIf / 2.0f)));
}

