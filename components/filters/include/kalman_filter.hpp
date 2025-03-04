#pragma once

#include <array>
#include <cmath>

#include "fast_math.hpp"

namespace espp {
/// Kalman Filter for single variable state estimation
/// @tparam N Number of states
///
/// This class implements a simple Kalman filter for estimating the state of a
/// variable, such as the orientation of a system. The filter is based on the
/// following state-space model:
/// \f[
/// X' = X + U \cdot dt
/// \f]
/// \f[
/// P' = P + Q
/// \f]
/// \f[
/// Y = Z - X
/// \f]
/// \f[
/// K = P / (P + R)
/// \f]
/// \f[
/// X = X + K \cdot Y
/// \f]
/// \f[
/// P = (1 - K) \cdot P
/// \f]
/// where:
/// - \f$X\f$ is the state vector
/// - \f$P\f$ is the covariance matrix
/// - \f$Q\f$ is the process noise
/// - \f$R\f$ is the measurement noise
/// - \f$U\f$ is the control input
/// - \f$dt\f$ is the time step
/// - \f$Z\f$ is the measurement
template <size_t N> class KalmanFilter {
public:
  /// Constructor
  KalmanFilter() {
    X.fill(0.0f); // Initialize state vector
    P.fill({});   // Initialize covariance matrix

    for (size_t i = 0; i < N; i++) {
      P[i][i] = 1.0f;   // Initial covariance
      Q[i][i] = 0.001f; // Process noise
      R[i][i] = 0.01f;  // Measurement noise
    }
  }

  /// Set process noise
  /// @param q Process noise
  /// @note The process noise is the uncertainty in the model. It is used to
  ///       compute the error covariance, which determines how much the
  ///       prediction is trusted. A higher process noise means that the
  ///       prediction is less trusted, and the filter relies more on the
  ///       measurement. A lower process noise means that the prediction is
  ///       more trusted, and the filter relies less on the measurement.
  void set_process_noise(float q) {
    for (size_t i = 0; i < N; i++)
      Q[i][i] = q;
  }

  /// Set process noise
  /// @param q Process noise vector
  void set_process_noise(const std::array<float, N> &q) {
    for (size_t i = 0; i < N; i++)
      Q[i][i] = q[i];
  }

  /// Set measurement noise
  /// @param r Measurement noise
  /// @note The measurement noise is the uncertainty in the measurement. It is
  ///       used to compute the Kalman Gain, which determines how much the
  ///       measurement should affect the state estimate.
  void set_measurement_noise(float r) {
    for (size_t i = 0; i < N; i++)
      R[i][i] = r;
  }

  /// Set measurement noise
  /// @param r Measurement noise vector
  void set_measurement_noise(const std::array<float, N> &r) {
    for (size_t i = 0; i < N; i++)
      R[i][i] = r[i];
  }

  /// Predict next state
  /// @param U Control input
  /// @param dt Time step
  /// @note The prediction step estimates the state at the next time step
  ///       based on the current state and the control input. The control
  ///       input is used to model the effect of the control on the state. The
  ///       time step is the time between measurements.
  void predict(const std::array<float, N> &U, float dt) {
    // Predict next state: X' = X + U * dt
    for (size_t i = 0; i < N; i++) {
      X[i] += U[i] * dt;
    }

    // Update covariance: P = P + Q
    for (size_t i = 0; i < N; i++) {
      for (size_t j = 0; j < N; j++) {
        P[i][j] += Q[i][j];
      }
    }
  }

  /// Update state estimate
  /// @param Z Measurement
  /// @note The correction step updates the state estimate based on the
  ///       measurement. The measurement is used to compute the Kalman Gain,
  ///       which determines how much the measurement should affect the state
  ///       estimate. The Kalman Gain is used to update the state estimate and
  ///       the error covariance.
  void update(const std::array<float, N> &Z) {
    std::array<float, N> Y; // Measurement residual

    // Compute residual: Y = Z - X
    for (size_t i = 0; i < N; i++) {
      Y[i] = Z[i] - X[i];
    }

    // Compute Kalman Gain: K = P / (P + R)
    for (size_t i = 0; i < N; i++) {
      for (size_t j = 0; j < N; j++) {
        K[i][j] = P[i][j] / (P[i][j] + R[i][j]);
      }
    }

    // Update state: X = X + K * Y
    for (size_t i = 0; i < N; i++) {
      X[i] += K[i][i] * Y[i];
    }

    // Update error covariance: P = (1 - K) * P
    for (size_t i = 0; i < N; i++) {
      for (size_t j = 0; j < N; j++) {
        P[i][j] = (1 - K[i][i]) * P[i][j];
      }
    }
  }

  /// Get state estimate
  /// @return State estimate
  /// @note The state estimate is the current estimate of the state based on
  ///       the measurements.
  const std::array<float, N> &get_state() const { return X; }

protected:
  std::array<float, N> X;                // State vector
  std::array<std::array<float, N>, N> P; // Covariance matrix
  std::array<std::array<float, N>, N> Q; // Process noise
  std::array<std::array<float, N>, N> R; // Measurement noise
  std::array<std::array<float, N>, N> K; // Kalman Gain
};                                       // class KalmanFilter
} // namespace espp
