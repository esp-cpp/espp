#pragma once

#include <cmath>

namespace espp {
/**
 * @brief Implements a gaussian function
 *        \f$y(t)=\alpha\exp(-\frac{(t-\beta)^2}{2\gamma^2})\f$.
 * @details Alows you to store the alpha, beta, and gamma coefficients as well
 *          as update them dynamically.
 *
 * \section gaussian_ex1 Example
 * \snippet math_example.cpp gaussian example
 */
class Gaussian {
public:
  /**
   * @brief Configuration structure for initializing the gaussian.
   */
  struct Config {
    float gamma; ///< Slope of the gaussian, range [0, 1]. 0 is more of a thin spike from 0 up to
                 ///< max output (alpha), 1 is more of a small wave around the max output (alpha).
    float alpha = 1.0f; ///< Max amplitude of the gaussian output, defautls to 1.0.
    float beta =
        0.5f; ///< Beta value for the gaussian, default to be symmetric at 0.5 in range [0,1].

    bool operator==(const Config &rhs) const = default;
  };

  /**
   * @brief Construct the gaussian object, configuring its parameters.
   * @param config Config structure for the gaussian.
   */
  explicit Gaussian(const Config &config)
      : gamma_(config.gamma)
      , alpha_(config.alpha)
      , beta_(config.beta) {}

  /**
   * @brief Get the currently configured gamma (shape).
   * @return The current gamma (shape) value [0, 1].
   */
  float gamma() const { return gamma_; }

  /**
   * @brief Set / Update the gamma (shape) value.
   * @param g New gamma (shape) to use.
   */
  void gamma(float g) { gamma_ = g; }

  /**
   * @brief Get the currently configured alpha (scaling) value.
   * @return The current alpha (scaler) value.
   */
  float alpha() const { return alpha_; }

  /**
   * @brief Set / Update the alpha (scaling) value.
   * @param a New alpha (scaler) to use.
   */
  void alpha(float a) { alpha_ = a; }

  /**
   * @brief Get the currently configured beta (shifting) value.
   * @return The current beta (shifter) value [0, 1].
   */
  float beta() const { return beta_; }

  /**
   * @brief Set / Update the beta (shifting) value.
   * @param b New beta (shifter) to use.
   */
  void beta(float b) { beta_ = b; }

  /**
   * @brief Evaluate the gaussian at \p t.
   * @param t The evaluation parameter, [0, 1].
   * @return The gaussian evaluated at \p t.
   */
  float at(float t) const {
    float tmb_y = (t - beta_) / gamma_;  // (t - B) / y
    float power = -0.5f * tmb_y * tmb_y; // -(t - B)^2 / 2y^2
    return alpha_ * exp(power);
  }

  /**
   * @brief Evaluate the gaussian at \p t.
   * @note Convienience wrapper around the at() method.
   * @param t The evaluation parameter, [0, 1].
   * @return The gaussian evaluated at \p t.
   */
  float operator()(float t) const { return at(t); }

protected:
  float gamma_;
  float alpha_;
  float beta_;
};
} // namespace espp
