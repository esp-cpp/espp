#pragma once

#include <cmath>

#include "fast_math.hpp"

namespace espp {
/// Madgwick filter for IMU data Based on:
/// https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/ (now
/// https://github.com/xioTechnologies/Fusion)
class MadgwickFilter {
public:
  /// @brief Constructor
  /// @param beta Filter gain
  /// @param sampleFreq Sample frequency
  explicit MadgwickFilter(float beta = 0.1f, float sampleFreq = 100.0f)
      : beta(beta)
      , sampleFreq(sampleFreq)
      , q0(1.0f)
      , q1(0.0f)
      , q2(0.0f)
      , q3(0.0f) {}

  /// @brief Update the filter with new data
  /// @param ax Accelerometer x value in g
  /// @param ay Accelerometer y value in g
  /// @param az Accelerometer z value in g
  /// @param gx Gyroscope x value in degrees/s
  /// @param gy Gyroscope y value in degrees/s
  /// @param gz Gyroscope z value in degrees/s
  /// @note Accelerometer values should be normalized
  /// @note Gyroscope values should be in degrees/s
  void update(float ax, float ay, float az, float gx, float gy, float gz) {
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

  /// @brief Get the current quaternion values as euler angles
  /// @param[out] pitch Pitch angle in degrees
  /// @param[out] roll Roll angle in degrees
  /// @param[out] yaw Yaw angle in degrees
  /// @note Euler angles are in degrees
  void get_euler(float &pitch, float &roll, float &yaw) const {
    pitch = atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / M_PI;
    roll = asin(2.0f * (q0 * q2 - q3 * q1)) * 180.0f / M_PI;
    yaw = atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / M_PI;
  }

protected:
  float beta, sampleFreq;
  float q0, q1, q2, q3; // Quaternion values
};
} // namespace espp
