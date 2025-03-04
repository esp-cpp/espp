#pragma once

#include <cmath>

#include "fast_math.hpp"

namespace espp {
/// Kalman Filter for angle estimation
///
/// This class implements a simple Kalman filter for estimating the angle of a
/// system based on accelerometer and gyroscope measurements. The filter is
/// based on the following state-space model:
///  x(k+1) = A * x(k) + B * u(k) + w(k)
///  z(k) = H * x(k) + v(k)
///  where:
///  - x(k) is the state vector at time k
///  - u(k) is the control input at time k
///  - z(k) is the measurement at time k
///  - w(k) is the process noise at time k
///  - v(k) is the measurement noise at time k
///  - A, B, and H are matrices
///  - A is the state transition matrix
///  - B is the control input matrix
///  - H is the measurement matrix
///  The Kalman filter estimates the state x(k) based on the measurements z(k).
///  The filter is implemented in two steps: prediction and correction.
///  The prediction step estimates the state at the next time step based on the
///  current state and the control input. The correction step updates the state
///  estimate based on the measurement.
class KalmanFilter {
public:
  /// Constructor
  KalmanFilter() {
    angle = 0.0f;
    bias = 0.0f;
    P[0][0] = 1.0f;
    P[0][1] = 0.0f;
    P[1][0] = 0.0f;
    P[1][1] = 1.0f;

    Q_angle = 0.001f;  // Process noise variance for accelerometer
    Q_bias = 0.003f;   // Process noise variance for gyro bias
    R_measure = 0.03f; // Measurement noise variance
  }

  /// Update the filter with a new measurement
  /// \param newAngle The new angle measurement
  /// \param newRate The new rate measurement
  /// \param dt The time step
  /// \return The updated angle estimate
  float update(float newAngle, float newRate, float dt) {
    // Prediction step
    rate = newRate - bias;
    angle += dt * rate;

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Correction step
    float S = P[0][0] + R_measure;
    float K[2] = {P[0][0] / S, P[1][0] / S};

    float y = newAngle - angle;
    angle += K[0] * y;
    bias += K[1] * y;

    // Update error covariance matrix
    float P00_temp = P[0][0], P01_temp = P[0][1];
    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
  }

protected:
  float angle, bias, rate;
  float P[2][2]; // Error covariance matrix
  float Q_angle, Q_bias, R_measure;
}; // class KalmanFilter
} // namespace espp
