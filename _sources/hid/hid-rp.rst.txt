HID-RP
******

The `hid-rp` component provides a wrapper around
https://github.com/intergatedcircuits/hid-rp and also provides an example
implementation of a configurable HID Gamepad using hid-rp.

It also implements Playstation DualSense, Playstation DualShock 4, Nintendo Switch Pro, Xbox gamepad,
and 3Dconnexion SpaceMouse reports.

At runtime, ``ReportMap`` maps a device's report descriptor (its Input and Output
fields), and ``RumbleEncoder`` builds a controller's rumble report from it: any
controller that declares a Physical Interface Device style rumble report (as the
Xbox Wireless controller does over Bluetooth LE) can be driven without knowing its
VID/PID. ``decode_switch_rumble_report()`` reads the motor levels a Nintendo Switch
host sends a Pro Controller.

.. ------------------------------- Example -------------------------------------

.. toctree::

   hid-rp_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/gamepad_hat.inc
.. include-build-file:: inc/gamepad_imu.inc
.. include-build-file:: inc/hid-rp.inc
.. include-build-file:: inc/hid-rp-3dconnexion.inc
.. include-build-file:: inc/hid-rp-gamepad.inc
.. include-build-file:: inc/hid-rp-playstation.inc
.. include-build-file:: inc/hid-rp-ps4.inc
.. include-build-file:: inc/hid-rp-rumble.inc
.. include-build-file:: inc/hid-rp-switch-pro.inc
.. include-build-file:: inc/hid-rp-xbox.inc
