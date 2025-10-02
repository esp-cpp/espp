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
 * \section gaussian_ex2 Fade-In/Fade-Out Example
 * \snippet math_example.cpp gaussian fade in fade out example
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
   * @brief Evaluate the gaussian at \p t.
   * @param t The evaluation parameter, [0, 1].
   * @return The gaussian evaluated at \p t.
   */
  float at(float t) const {
    float tmb_y = (t - beta_) / gamma_;  // (t - B) / y
    float power = -0.5f * tmb_y * tmb_y; // -(t - B)^2 / 2y^2
    return alpha_ * expf(power);
  }

  /**
   * @brief Evaluate the gaussian at \p t.
   * @note Convienience wrapper around the at() method.
   * @param t The evaluation parameter, [0, 1].
   * @return The gaussian evaluated at \p t.
   */
  float operator()(float t) const { return at(t); }

  /**
   * @brief Update the gaussian configuration.
   * @param config The new configuration.
   */
  void update(const Config &config) {
    gamma_ = config.gamma;
    alpha_ = config.alpha;
    beta_ = config.beta;
  }

  /**
   * @brief Set the configuration of the gaussian.
   * @param config The new configuration.
   */
  void set_config(const Config &config) { update(config); }

  /**
   * @brief Get the current configuration of the gaussian.
   * @return The current configuration.
   */
  Config get_config() const { return {.gamma = gamma_, .alpha = alpha_, .beta = beta_}; }

  /**
   * @brief Get the gamma value.
   * @return The gamma value.
   */
  float get_gamma() const { return gamma_; }

  /**
   * @brief Get the alpha value.
   * @return The alpha value.
   */
  float get_alpha() const { return alpha_; }

  /**
   * @brief Get the beta value.
   * @return The beta value.
   */
  float get_beta() const { return beta_; }

  /**
   * @brief Set the gamma value.
   * @param gamma The new gamma value.
   */
  void set_gamma(float gamma) { gamma_ = gamma; }

  /**
   * @brief Set the alpha value.
   * @param alpha The new alpha value.
   */
  void set_alpha(float alpha) { alpha_ = alpha; }

  /**
   * @brief Set the beta value.
   * @param beta The new beta value.
   */
  void set_beta(float beta) { beta_ = beta; }

protected:
  float gamma_; ///<! Slope of the gaussian, range [0, 1]. 0 is more of a thin spike from 0 up to
                ///   max output (alpha), 1 is more of a small wave around the max output (alpha).
  float alpha_; ///<! Max amplitude of the gaussian output, defautls to 1.0.
  float beta_;  ///<! Shifting / Beta value for the gaussian, default to be
                ///   symmetric at 0.5 in range [0,1].
};
} // namespace espp
