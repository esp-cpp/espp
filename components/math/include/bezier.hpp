#pragma once

#include <array>

namespace espp {
/**
 * @brief Implements rational / weighted and unweighted cubic bezier curves
 *        between control points.
 * @note See https://pomax.github.io/bezierinfo/ for information on bezier
 *       curves.
 * @note Template class which can be used individually on floating point
 *       values directly or on containers such as Vector2d<float>.
 */
template <typename T> class Bezier {
public:
  /**
   * @brief Unweighted cubic bezier configuration for 4 control points.
   */
  struct Config {
    std::array<T, 4> control_points; ///< Array of 4 control points
  };

  /**
   * @brief Weighted cubic bezier configuration for 4 control points with
   *        individual weights.
   */
  struct WeightedConfig {
    std::array<T, 4> control_points; ///< Array of 4 control points
    std::array<float, 4> weights = {1.0f, 1.0f, 1.0f,
                                    1.0f}; ///< Array of 4 weights, default is array of 1.0f
  };

  /**
   * @brief Construct an unweighted cubic bezier curve for evaluation.
   * @param config Unweighted Config structure containing the control points.
   */
  explicit Bezier(const Config &config)
      : weighted_(false), control_points_(config.control_points) {}

  /**
   * @brief Construct a rational / weighted cubic bezier curve for evaluation.
   * @param config Rational / weighted WeightedConfig structure containing the
   *        control points and their weights.
   */
  explicit Bezier(const WeightedConfig &config)
      : weighted_(true), control_points_(config.control_points), weights_(config.weights) {}

  /**
   * @brief Evaluate the bezier at \p t.
   * @param t The evaluation parameter, [0, 1].
   * @return The bezier evaluated at \p t.
   */
  T at(float t) const {
    if (weighted_) {
      return weighted_eval(t);
    } else {
      return eval(t);
    }
  }

  /**
   * @brief Evaluate the bezier at \p t.
   * @note Convienience wrapper around the at() method.
   * @param t The evaluation parameter, [0, 1].
   * @return The bezier evaluated at \p t.
   */
  T operator()(float t) const { return at(t); }

protected:
  T eval(float t) const {
    auto t2 = t * t;
    auto t3 = t2 * t;
    auto mt = 1.0f - t;
    auto mt2 = mt * mt;
    auto mt3 = mt2 * mt;
    return control_points_[0] * mt3 + control_points_[1] * 3.0f * mt2 * t +
           control_points_[2] * 3.0f * mt * t2 + control_points_[3] * t3;
  }

  T weighted_eval(float t) const {
    auto t2 = t * t;
    auto t3 = t2 * t;
    auto mt = 1.0f - t;
    auto mt2 = mt * mt;
    auto mt3 = mt2 * mt;
    const float f[] = {weights_[0] * mt3, weights_[1] * 3.0f * mt2 * t,
                       weights_[2] * 3.0f * mt * t2, weights_[3] * t3};
    float basis = f[0] + f[1] + f[2] + f[3];
    return (f[0] * control_points_[0] + f[1] * control_points_[1] + f[2] * control_points_[2] +
            f[3] * control_points_[3]) /
           basis;
  }

  bool weighted_{false};
  std::array<T, 4> control_points_;
  std::array<float, 4> weights_ = {{1.0f, 1.0f, 1.0f, 1.0f}};
};
} // namespace espp
