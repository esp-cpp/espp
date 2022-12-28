#pragma once

#include "fast_math.hpp"

namespace espp {
  /**
   * @brief Implements cubic bezier curves between control points.
   * @note See https://pomax.github.io/bezierinfo/ for information on bezier
   *       curves.
   * @param t Curve position parameter in the range [0, 1]
   * @param p0 First control point
   * @param p1 Second control point
   * @param p2 Third control point
   * @param p3 Fourth control point
   * @return Interpolated point along the curve defined by the control points at
   *         t.
   */
  template <typename T>
  T bezier(float t, T p0, T p1, T p2, T p3) {
    auto t2 = t*t;
    auto t3 = t2*t;
    auto mt = 1.0f - t;
    auto mt2 = mt*mt;
    auto mt3 = mt2*mt;
    return
      p0 * mt3 +
      p1 * 3.0f * mt2 * t +
      p2 * 3.0f * mt * t2 +
      p3 * t3;
  }

  /**
   * @brief Implements rational / weighted cubic bezier curves between control
   *        points.
   * @note See https://pomax.github.io/bezierinfo/ for information on bezier
   *       curves.
   * @param t Curve position parameter in the range [0, 1]
   * @param p0 First control point
   * @param p1 Second control point
   * @param p2 Third control point
   * @param p3 Fourth control point
   * @param w0 First weight
   * @param w1 Second weight
   * @param w2 Third weight
   * @param w3 Fourth weight
   * @return Interpolated point along the curve defined by the control points at
   *         t with the associated control point weights.
   */
  template <typename T>
  T bezier(float t, T p0, T p1, T p2, T p3, float w0, float w1, float w2, float w3) {
    auto t2 = t*t;
    auto t3 = t2*t;
    auto mt = 1.0f - t;
    auto mt2 = mt*mt;
    auto mt3 = mt2*mt;
    float f[] = {
      w0 * mt3,
      w1 * 3.0f * mt2 * t,
      w2 * 3.0f * mt * t2,
      w3 * t3
    };
    auto basis = f[0] + f[1] + f[2] + f[3];
    return (f[0] * p0 + f[1] * p1 + f[2] * p2 + f[3] * p3) / basis;
  }
}
