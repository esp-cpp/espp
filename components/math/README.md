# Math Component

[![Badge](https://components.espressif.com/components/espp/math/badge.svg)](https://components.espressif.com/components/espp/math)

The `math` component provides various classes and static functions for commonly
used math operations.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [Math Component](#math-component)
  - [Bezier](#bezier)
  - [Fast Math](#fast-math)
  - [Gaussian](#gaussian)
  - [Range Mapper](#range-mapper)
  - [Vector2d (2 Dimensional Vector)](#vector2d-2-dimensional-vector)
  - [Example](#example)

<!-- markdown-toc end -->

## Bezier

The `bezier` header provides a templated implementation of cubic bezier curves
and rational cubic bezier curves. Intended use for these templated functions is
on raw floating point values or on the associated Vector2d class.

## Fast Math

The `Fast Math` header provides a set of static functions which implement
optimized approximations of the following functions:

* fast_sqrt
* fast_ln
* fast_sin
* fast_cos

along with some utility functions:

* square (x^2)
* cube (x^3)
* sgn (sign of a number)
* round (round floating point value to nearest integer value)
* lerp (linear interpolate between two points)
* inv_lerp (inverse linear interpolate between two points)
* piecewise_linear (compute the piecewise linear interpolation between a set of points)

## Gaussian

The `gaussian` class provides an implementation of the gaussian function:

```math
   y(t)= \alpha e^{ -\frac{ (t-\beta)^2 }{ 2\gamma^2 } }
```

The class allows for dynamically changing the $\alpha$, $\beta$, and $\gamma$
parameters.

## Range Mapper

The `RangeMapper` provides a class which allows you to map from a configurable
input range to a standardized output range of [-1,1].

## Vector2d (2 Dimensional Vector)

The `Vector2d` provides a container for a 2-dimensional vector with associated
math operations implemented.

## Example

The [example](./example) showcases some of the math functionality provided by
the `math` component.

