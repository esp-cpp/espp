#pragma once

#include <array>
#include <math.h>

#include "format.hpp"

namespace espp {
  /**
   * @brief Container representing a 2 dimensional vector.
   *
   * Provides getters/setters, index operator, and vector / scalar math
   * utilities.
   */
  template <typename T> class Vector2d {
  public:
    /**
     * @brief Constructor for the vector, defaults to 0,0.
     * @param x The starting X value.
     * @param y The starting Y value.
     */
    Vector2d(T x = T(0), T y = T(0)) : x_(x), y_(y) {}

    /**
     * @brief Vector copy constructor.
     * @param other Vector to copy.
     */
    Vector2d(const Vector2d& other) : x_(other.x_), y_(other.y_) {}

    /**
     * @brief Returns vector magnitude: ||v||.
     * @return The magnitude.
     */
    T magnitude() const {
      return sqrt(magnitude_squared());
    }

    /**
     * @brief Returns vector magnitude squared: ||v||^2.
     * @return The magnitude squared.
     */
    T magnitude_squared() const {
      return x_ * x_ + y_ * y_;
    }

    /**
     * @brief Getter for the x value.
     * @return The current x value.
     */
    T x() const {
      return x_;
    }

    /**
     * @brief Setter for the x value.
     * @param v New value for \c x.
     */
    void x(T v) {
      x_ = v;
    }

    /**
     * @brief Getter for the y value.
     * @return The current y value.
     */
    T y() const {
      return y_;
    }

    /**
     * @brief Setter for the y value.
     * @param v New value for \c y.
     */
    void y(T v) {
      y_ = v;
    }

    /**
     * @brief Index operator for vector elements. NOTE: Returns a mutable
     *        reference to the element.
     * @param index The index to return.
     * @return Mutable reference to the element at \p index.
     */
    T& operator[] (int index) {
        return values_[index];
    }

    /**
     * @brief Negate the vector.
     * @return The new vector which is the negative.
     */
    Vector2d operator- () const {
      return Vector2d(-x_, -y_);
    }

    /**
     * @brief Return a new vector which is the provided vector subtracted from
     *        this vector.
     * @param rhs The vector to subtract from this vector.
     * @return Resultant vector subtraction.
     */
    Vector2d operator- (const Vector2d& rhs) const {
      return Vector2d(x_ - rhs.x_, y_ - rhs.y_);
    }

    /**
     * @brief Return the provided vector subtracted from this vector.
     * @param rhs The vector to subtract from this vector.
     * @return Resultant vector subtraction.
     */
    Vector2d& operator-= (const Vector2d& rhs) {
      x_ -= rhs.x_;
      y_ -= rhs.y_;
      return *this;
    }

    /**
     * @brief Return a new vector, which is the addition of this vector and the
     *        provided vector.
     * @param rhs The vector to add to this vector.
     * @return Resultant vector addition.
     */
    Vector2d operator+ (const Vector2d& rhs) const {
      return Vector2d(x_ + rhs.x_, y_ + rhs.y_);
    }

    /**
     * @brief Return the vector added with the provided vector.
     * @param rhs The vector to add to this vector.
     * @return Resultant vector addition.
     */
    Vector2d& operator+= (const Vector2d& rhs) {
      x_ += rhs.x_;
      y_ += rhs.y_;
      return *this;
    }

    /**
     * @brief Return a scaled version of the vector, multiplied by the provided
     *        value.
     * @param v Value the vector should be multiplied by.
     * @return Resultant scaled vector.
     */
    Vector2d operator* (const T& v) const {
      return Vector2d(x_ * v, y_ * v);
    }

    /**
     * @brief Return the vector multiplied by the provided value.
     * @param v Value the vector should be scaled by.
     * @return Resultant scaled vector.
     */
    Vector2d& operator*= (const T& v) {
      x_ *= v;
      y_ *= v;
      return *this;
    }

    /**
     * @brief Return a scaled version of the vector, divided by the provided
     *        value.
     * @param v Value the vector should be divided by.
     * @return Resultant scaled vector.
     */
    Vector2d operator/ (const T& v) const {
      if (v != T(0)) {
        return Vector2d(x_ / v, y_ / v);
      } else {
        return *this;
      }
    }

    /**
     * @brief Return the vector divided by the provided value.
     * @param v Value the vector should be divided by.
     * @return Resultant scaled vector.
     */
    Vector2d& operator/= (const T& v) {
      if (v != T(0)) {
        x_ /= v;
        y_ /= v;
      }
      return *this;
    }

    /**
     * @brief Dot product of this vector with another vector.
     * @param other The second vector
     * @return The dot product (x1*x2 + y1*y2)
     */
    T dot (const Vector2d& other) const {
      return (x_ * other.x_) + (y_ * other.y_);
    }

    /**
     * @brief Return normalized (unit length) version of the vector.
     * @return The normalized vector.
     */
    Vector2d normalized() const {
      T m = magnitude();
      if (m > T(0)) {
        return *this / m;
      }
      return *this;
    }

    /**
     * @brief Rotate the vector by \p radians.
     *        NOTE: this function is only available if T is a floating point
     *        value.
     *
     * @param radians Amount of rotation (in radians) to rotate the vector by.
     * @return Rotated vector.
     */
    template<class U = T,
             typename std::enable_if<std::is_floating_point<U>::value>::type* = nullptr>
    Vector2d rotated(T radians) const {
      if (radians != T(0)) {
        T x2 = x_ * cos(radians) - y_ * sin(radians);
        T y2 = x_ * sin(radians) + y_ * cos(radians);
        return Vector2d(x2, y2);
      }
      return *this;
    }

    /**
     * @brief Print the vector to a string, formatted "({},{})".
     * @return std::string holding the formatted vector.
     */
    std::string to_string() const {
        return fmt::format("({},{})", x_, y_);
    }

  protected:
    union {
      struct {
        T x_;
        T y_;
      };
      std::array<T,2> values_;
    };
  };

  typedef Vector2d<float> Vector2f;
  typedef Vector2d<double> Vector2f64;
  typedef Vector2d<int> Vector2i;
  typedef Vector2d<uint8_t> Vector2u8;
}
