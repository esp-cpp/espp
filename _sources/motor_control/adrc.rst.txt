ADRC APIs
*********

ADRC
----

The `adrc` component provides active disturbance rejection controllers for
first-order and second-order plants, including both linear ADRC and Han-style
nonlinear ADRC variants.

At a high level, ADRC treats plant uncertainty and external disturbances as an
extra state that can be estimated online by an extended state observer (ESO).
Instead of relying on a very accurate plant model, you provide an approximate
input gain ``b0`` and let the observer estimate the remaining dynamics. That
usually makes ADRC attractive for embedded robotics systems where friction,
battery sag, payload changes, cable drag, and contact disturbances are present
but hard to model precisely.

This component includes:

- ``HanTrackingDifferentiator`` for setpoint shaping and reference-rate
  estimation
- ``LinearAdrcFirstOrder`` for approximately first-order plants
- ``LinearAdrcSecondOrder`` for approximately second-order plants
- ``HanAdrcFirstOrder`` for nonlinear first-order ADRC
- ``HanAdrcSecondOrder`` for nonlinear second-order ADRC

Choosing a controller
---------------------

.. list-table::
   :header-rows: 1
   :widths: 24 38 38

   * - Controller
     - Best fit
     - Typical robotics example
   * - Linear first-order ADRC
     - Output behaves roughly like a one-pole loop
     - Motor velocity loop above an inner current or torque loop
   * - Linear second-order ADRC
     - Input primarily affects acceleration
     - Servo position loop, gimbal angle loop, heading loop
   * - Han first-order ADRC
     - First-order loop needing nonlinear shaping or better large-error behavior
     - Velocity loop with stiction, load steps, or wide operating range
   * - Han second-order ADRC
     - Second-order loop with large disturbances and aggressive setpoint changes
     - Motor-driven joint position control or heading control on variable terrain

For many motor-control applications, a good starting point is:

1. use a faster inner current or torque loop if one already exists,
2. place first-order ADRC around motor speed, or second-order ADRC around
   position,
3. move to the Han nonlinear variants only if the linear controllers do not
   give the disturbance rejection or large-error behavior you want.

Robotics motor-control usage
----------------------------

In a practical robotics stack, ADRC is usually most effective in the outer
mechanical loops rather than as a replacement for every low-level regulator.
For example:

- a BLDC or DC motor drive might keep its inner current loop and use ADRC for
  wheel speed,
- a servo joint might use second-order ADRC on position with the motor driver
  handling the faster electrical dynamics,
- a mobile robot heading controller can use second-order ADRC to compensate for
  uneven traction, slope changes, or payload variation.

The observer states are often useful for debugging and tuning:

- ``z2`` in the first-order controllers is the estimated lumped disturbance,
- ``z3`` in the second-order controllers is the estimated lumped disturbance,
- large persistent disturbance estimates usually indicate load torque, friction,
  bias, or a poor ``b0`` estimate.

Those estimated disturbance states are especially valuable in robotics, because
they can explain why a loop feels "mysteriously" different under battery sag,
ground contact, or changing payload.

Tuning for motors and actuators
-------------------------------

ADRC is often tuned by bandwidth rather than by directly shaping multiple
independent gains. A practical workflow for motor-control applications is:

1. **Pick the structure first.** Use first-order ADRC for speed-like loops and
   second-order ADRC for position-like loops.
2. **Estimate ``b0``.** The sign must be correct. The magnitude only needs to
   be approximate, but if it is very wrong the controller will feel badly
   scaled and may saturate too early.
3. **Start with conservative bandwidth.** Increase the controller bandwidth
   until the loop is responsive but not noisy or oscillatory.
4. **Make the observer faster than the controller.** A faster ESO helps reject
   disturbances early, but too much observer bandwidth will amplify encoder or
   velocity-estimate noise.
5. **Use the tracking differentiator to soften steps.** This is especially
   helpful for position commands that would otherwise demand unrealistic
   acceleration from the motor.
6. **Watch saturation.** If the command rails against ``output_min`` /
   ``output_max``, back off the bandwidth or revisit ``b0`` and trajectory
   shaping before increasing gains further.

Some practical heuristics:

- for the linear variants, choose ``observer_bandwidth`` several times larger
  than ``controller_bandwidth`` and increase gradually;
- if encoder noise or quantization dominates, reduce observer bandwidth before
  blaming the control law;
- if large setpoint steps cause overshoot or chatter, enable the tracking
  differentiator or lower its aggressiveness;
- for the Han nonlinear variants, increase ``fal_delta`` if the loop is too
  sharp around zero error, and adjust the ``alpha`` parameters only after the
  main gain and bandwidth choices are in the right range.

.. note::

   The most important ADRC tuning parameter is usually not a fancy nonlinear
   exponent, but the combination of loop rate, actuator saturation, sensor
   quality, and a reasonable ``b0`` estimate. If the sample period is too slow
   or the actuator is saturating constantly, ADRC will not rescue the loop.

References
----------

Useful starting points for ADRC background and tuning include:

- Jingqing Han, *From PID to Active Disturbance Rejection Control*, IEEE
  Transactions on Industrial Electronics, 2009.
- Zhiqiang Gao, *Active Disturbance Rejection Control: A Paradigm Shift in
  Feedback Control System Design*, Proceedings of the American Control
  Conference, 2006.
- Zhiqiang Gao, *Scaling and Bandwidth-Parameterization Based Controller
  Tuning*, Proceedings of the American Control Conference, 2002.
- Gernot Herbst, *Practical Active Disturbance Rejection Control: Tuning
  Methods and Guidelines for Continuous-Time Systems*, IFAC-PapersOnLine, 2015.

Code examples for the ADRC API are provided in the ``components/adrc/example``
folder.

.. toctree::

   adrc_example

API Reference
-------------

.. include-build-file:: inc/adrc.inc
