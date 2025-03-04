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
  explicit MadgwickFilter(float beta = 0.1f)
      : beta(beta)
      , q0(1.0f)
      , q1(0.0f)
      , q2(0.0f)
      , q3(0.0f) {}

  /// @brief Update the filter with new data
  /// @param dt Time step in seconds
  /// @param ax Accelerometer x value in g
  /// @param ay Accelerometer y value in g
  /// @param az Accelerometer z value in g
  /// @param gx Gyroscope x value in rad/s
  /// @param gy Gyroscope y value in rad/s
  /// @param gz Gyroscope z value in rad/s
  /// @param mx Magnetometer x value in uT
  /// @param my Magnetometer y value in uT
  /// @param mz Magnetometer z value in uT
  /// @note Accelerometer values should be normalized
  /// @note Gyroscope values should be in rad/s
  /// @note Magnetometer values should be normalized
  /// @note If magnetometer values are not available (all 0), this function will
  ///       fallback to using only accelerometer and gyroscope values
  void update(float dt, float ax, float ay, float az, float gx, float gy, float gz, float mx,
              float my, float mz) {
    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer
    // normalisation)
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
      update(dt, ax, ay, az, gx, gy, gz);
      return;
    }

    float recipNorm;
    float qDot1, qDot2, qDot3, qDot4;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer
    // normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
      float s0, s1, s2, s3;
      float hx, hy;
      float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2,
          _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

      // Normalise accelerometer measurement
      recipNorm = fast_inv_sqrt(ax * ax + ay * ay + az * az);
      ax *= recipNorm;
      ay *= recipNorm;
      az *= recipNorm;

      // Normalise magnetometer measurement
      recipNorm = fast_inv_sqrt(mx * mx + my * my + mz * mz);
      mx *= recipNorm;
      my *= recipNorm;
      mz *= recipNorm;

      // Auxiliary variables to avoid repeated arithmetic
      _2q0mx = 2.0f * q0 * mx;
      _2q0my = 2.0f * q0 * my;
      _2q0mz = 2.0f * q0 * mz;
      _2q1mx = 2.0f * q1 * mx;
      _2q0 = 2.0f * q0;
      _2q1 = 2.0f * q1;
      _2q2 = 2.0f * q2;
      _2q3 = 2.0f * q3;
      _2q0q2 = 2.0f * q0 * q2;
      _2q2q3 = 2.0f * q2 * q3;
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

      // Reference direction of Earth's magnetic field
      hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 -
           mx * q2q2 - mx * q3q3;
      hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 +
           _2q2 * mz * q3 - my * q3q3;
      _2bx = sqrt(hx * hx + hy * hy);
      _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 -
             mz * q2q2 + mz * q3q3;
      _4bx = 2.0f * _2bx;
      _4bz = 2.0f * _2bz;

      // Gradient decent algorithm corrective step
      s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) -
           _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
           (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
           _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
      s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) -
           4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) +
           _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
           (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
           (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
      s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) -
           4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) +
           (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
           (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
           (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
      s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) +
           (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
           (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
           _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
      recipNorm = fast_inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
      s0 *= recipNorm;
      s1 *= recipNorm;
      s2 *= recipNorm;
      s3 *= recipNorm;

      // Apply feedback step
      qDot1 -= beta * s0;
      qDot2 -= beta * s1;
      qDot3 -= beta * s2;
      qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalise quaternion
    recipNorm = fast_inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
  }

  /// @brief Update the filter with new data
  /// @param dt Time step in seconds
  /// @param ax Accelerometer x value in g
  /// @param ay Accelerometer y value in g
  /// @param az Accelerometer z value in g
  /// @param gx Gyroscope x value in rad/s
  /// @param gy Gyroscope y value in rad/s
  /// @param gz Gyroscope z value in rad/s
  /// @note Accelerometer values should be normalized
  /// @note Gyroscope values should be in rad/s
  void update(float dt, float ax, float ay, float az, float gx, float gy, float gz) {
    float recipNorm;
    float qDot1, qDot2, qDot3, qDot4;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer
    // normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
      float s0, s1, s2, s3;
      float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

      // Normalise accelerometer measurement
      recipNorm = fast_inv_sqrt(ax * ax + ay * ay + az * az);
      ax *= recipNorm;
      ay *= recipNorm;
      az *= recipNorm;

      // Auxiliary variables to avoid repeated arithmetic
      _2q0 = 2.0f * q0;
      _2q1 = 2.0f * q1;
      _2q2 = 2.0f * q2;
      _2q3 = 2.0f * q3;
      _4q0 = 4.0f * q0;
      _4q1 = 4.0f * q1;
      _4q2 = 4.0f * q2;
      _8q1 = 8.0f * q1;
      _8q2 = 8.0f * q2;
      q0q0 = q0 * q0;
      q1q1 = q1 * q1;
      q2q2 = q2 * q2;
      q3q3 = q3 * q3;

      // Gradient decent algorithm corrective step
      s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
      s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 +
           _8q1 * q2q2 + _4q1 * az;
      s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 +
           _8q2 * q2q2 + _4q2 * az;
      s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
      recipNorm = fast_inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
      s0 *= recipNorm;
      s1 *= recipNorm;
      s2 *= recipNorm;
      s3 *= recipNorm;

      // Apply feedback step
      qDot1 -= beta * s0;
      qDot2 -= beta * s1;
      qDot3 -= beta * s2;
      qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalise quaternion
    recipNorm = fast_inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
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
    pitch =
        atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * 180.0f / M_PI;
    roll = asin(-2.0f * (q1 * q3 - q0 * q2)) * 180.0f / M_PI;
    yaw = atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 180.0f / M_PI;
  }

protected:
  float beta;           // Filter gain
  float q0, q1, q2, q3; // Quaternion values
};
} // namespace espp
