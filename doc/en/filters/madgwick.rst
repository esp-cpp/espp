Madgwick Filter
***************

The `MadgwickFilter` implements the Madgwick algorithm for orientation
estimation using an IMU. The algorithm is based on the paper `An efficient
orientation filter for inertial and inertial/magnetic sensor arrays <https://courses.cs.washington.edu/courses/cse466/14au/labs/l4/madgwick_internal_report.pdf>`_ by
Sebastian Madgwick. It supports state / orientation estimation for both 6-axis
IMUs as well as for 9-axis IMUs.

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/madgwick_filter.inc
