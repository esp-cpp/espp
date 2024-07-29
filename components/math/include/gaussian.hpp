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
    float alpha{1.0f}; ///< Max amplitude of the gaussian output, defautls to 1.0.
    float beta{
        0.5f}; ///< Beta value for the gaussian, default to be symmetric at 0.5 in range [0,1].
  };

  /**
   * @brief Construct the gaussian object, configuring its parameters.
   * @param config Config structure for the gaussian.
   */
  explicit Gaussian(const Config &config)
      : gamma(config.gamma)
      , alpha(config.alpha)
      , beta(config.beta) {}

  /**
   * @brief Evaluate the gaussian at \p t.
   * @param t The evaluation parameter, [0, 1].
   * @return The gaussian evaluated at \p t.
   */
  float at(float t) const {
    float tmb_y = (t - beta) / gamma;    // (t - B) / y
    float power = -0.5f * tmb_y * tmb_y; // -(t - B)^2 / 2y^2
    return alpha * exp(power);
  }

  /**
   * @brief Evaluate the gaussian at \p t.
   * @note Convienience wrapper around the at() method.
   * @param t The evaluation parameter, [0, 1].
   * @return The gaussian evaluated at \p t.
   */
  float operator()(float t) const { return at(t); }

  float gamma; ///<! Slope of the gaussian, range [0, 1]. 0 is more of a thin spike from 0 up to
               ///   max output (alpha), 1 is more of a small wave around the max output (alpha).
  float alpha; ///<! Max amplitude of the gaussian output, defautls to 1.0.
  float beta;  ///<! Shifting / Beta value for the gaussian, default to be
               ///   symmetric at 0.5 in range [0,1].
};
} // namespace espp
