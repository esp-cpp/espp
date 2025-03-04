#pragma once

#include <cmath>

#include "fast_math.hpp"

namespace espp {
class MadgwickFilter {
public:
  MadgwickFilter(float beta = 0.1f, float sampleFreq = 100.0f)
      : beta(beta)
      , sampleFreq(sampleFreq)
      , q0(1.0f)
      , q1(0.0f)
      , q2(0.0f)
      , q3(0.0f) {}

  void update(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;

    // Convert gyroscope from degrees/s to radians/s
    gx *= M_PI / 180.0f;
    gy *= M_PI / 180.0f;
    gz *= M_PI / 180.0f;

    // Compute quaternion derivatives
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Normalize accelerometer measurement
    recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Compute feedback correction terms
    s0 = 2.0f * (q2 * q3 - q0 * q1) - ax;
    s1 = 2.0f * (q0 * q2 + q1 * q3) - ay;
    s2 = 1.0f - 2.0f * (q1 * q1 + q2 * q2) - az;
    s3 = 2.0f * (q1 * q2 + q0 * q3) - az; // Corrected: Missing s3 term

    // Normalize correction
    recipNorm = 1.0f / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Compute feedback
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3; // Corrected: s3 was missing before

    // Integrate quaternion
    q0 += qDot1 * (1.0f / sampleFreq);
    q1 += qDot2 * (1.0f / sampleFreq);
    q2 += qDot3 * (1.0f / sampleFreq);
    q3 += qDot4 * (1.0f / sampleFreq);

    // Normalize quaternion
    recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
  }

  void get_euler(float &pitch, float &roll, float &yaw) {
    pitch = atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / M_PI;
    roll = asin(2.0f * (q0 * q2 - q3 * q1)) * 180.0f / M_PI;
    yaw = atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / M_PI;
  }

protected:
  float beta, sampleFreq;
  float q0, q1, q2, q3; // Quaternion values
};
} // namespace espp
