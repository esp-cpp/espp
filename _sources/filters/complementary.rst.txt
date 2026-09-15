Complementary Filter
********************

The `ComplementaryFilter` provides a simple implementation of a filter
specifically designed to combine the outputs of a gyroscope and an accelerometer
to produce a more accurate estimate of the orientation of an object. This filter
is provided for completeness and simplicity, but the `KalmanFilter` or
`MadgwickFilter` are generally preferred for this purpose.

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/complementary_filter.inc
