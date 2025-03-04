#pragma once

#include <cmath>

#include "fast_math.hpp"

namespace espp {
/// Complementary filter for estimating pitch and roll angles from
/// accelerometer and gyroscope readings.
/// The filter is defined by the following equation:
///  angle = alpha * (angle + gyro * dt) + (1 - alpha) * accel
///  where:
///  - angle is the estimated angle,
///  - alpha is the filter coefficient (0 < alpha < 1),
///  - gyro is the gyroscope reading,
///  - accel is the accelerometer reading,
///  - dt is the time step.
class ComplementaryFilter {
public:
  /// Constructor
  /// \param alpha Filter coefficient (0 < alpha < 1)
  explicit ComplementaryFilter(float alpha = 0.98)
      : alpha(alpha)
      , pitch(0.0f)
      , roll(0.0f) {}

  /// Update the filter with accelerometer and gyroscope readings.
  /// \param accel Accelerometer readings (m/s^2)
  /// \param gyro Gyroscope readings (degrees/s)
  /// \param dt Time step (s)
  /// \note The accelerometer and gyroscope readings must be in the
  /// same coordinate system.
  void update(const std::span<const float, 3> &accel, const std::span<const float, 3> &gyro,
              float dt) {
    update(accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], dt);
  }

  /// Update the filter with accelerometer and gyroscope readings.
  /// \param ax Accelerometer x-axis reading (m/s^2)
  /// \param ay Accelerometer y-axis reading (m/s^2)
  /// \param az Accelerometer z-axis reading (m/s^2)
  /// \param gx Gyroscope x-axis reading (degrees/s)
  /// \param gy Gyroscope y-axis reading (degrees/s)
  /// \param gz Gyroscope z-axis reading (degrees/s)
  /// \param dt Time step (s)
  /// \note The accelerometer and gyroscope readings must be in the
  /// same coordinate system.
  void update(float ax, float ay, float az, float gx, float gy, float gz, float dt) {
    // Convert gyroscope readings from degrees/s to radians/s
    gx = gx * M_PI / 180.0f;
    gy = gy * M_PI / 180.0f;

    // Compute pitch and roll from accelerometer (degrees)
    float accelPitch = atan2(ay, sqrt(ax * ax + az * az)) * 180.0f / M_PI;
    float accelRoll = atan2(-ax, az) * 180.0f / M_PI;

    // Integrate gyroscope readings
    float gyroPitch = pitch + gy * dt;
    float gyroRoll = roll + gx * dt;

    // Apply complementary filter
    pitch = alpha * gyroPitch + (1 - alpha) * accelPitch;
    roll = alpha * gyroRoll + (1 - alpha) * accelRoll;
  }

  /// Get the estimated pitch angle (degrees)
  /// \return The estimated pitch angle
  float get_pitch() const { return pitch; }

  /// Get the estimated roll angle (degrees)
  /// \return The estimated roll angle
  float get_roll() const { return roll; }

protected:
  float alpha;
  float pitch, roll;
}; // class ComplimentaryFilter
} // namespace espp
