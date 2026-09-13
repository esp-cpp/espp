Kalman Filter
*************

The `KalmanFilter` class implements a Kalman filter for linear systems. The
filter can be used to estimate the state of a linear system given noisy
measurements. The filter can be configured for an arbitrary number of states
and measurements. You can also specify the process noise and measurement noise
covariances.

To use the filter, you must first create an instance of the `KalmanFilter`
class, then call the `predict` and `update` methods to estimate the state of
the system. To get the current state estimate, call the `get_state` method.

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/kalman_filter.inc
