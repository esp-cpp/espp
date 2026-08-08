Fast Math
*********

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

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/fast_math.inc
