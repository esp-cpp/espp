Gamepad Map APIs
****************

Gamepad Map
-----------

The `gamepad_map` component remaps a gamepad's buttons and axes: button to
button, analog to digital (a stick direction or trigger past a threshold),
digital to analog (a button driving an axis) and analog to analog (swap,
invert, scale), plus radial stick dead zones. `GamepadState` names buttons by
position (south / east / west / north) so a map means the same thing for Xbox,
Nintendo and PlayStation controllers. `GamepadProfiles` keeps a default map and
one map per controller type (vendor / product ID) in a file, stored with
`PersistentData`. `to_gamepad_state()` converts the runtime HID decoder's
`hid_rp::GamepadReport`.

`GamepadMapService` serves a `GamepadProfiles` store over any framed byte
stream (dispatcher module 11, protocol ``espp.gamepad_map`` v1), so the hosted
**Gamepad Map Console** web app can edit the maps over WebUSB / Web Serial and
show the controller's input before and after mapping, live.

Code examples for the Gamepad Map API are provided in the `gamepad_map`
`example` (decode, map, profiles) and `webapp_example` (the service over USB
with the web console) folders.

.. ------------------------------- Example -------------------------------------

.. toctree::

   gamepad_map_example
   gamepad_map_webapp_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/gamepad_state.inc
.. include-build-file:: inc/gamepad_map.inc
.. include-build-file:: inc/gamepad_map_service.inc
.. include-build-file:: inc/gamepad_profiles.inc
.. include-build-file:: inc/gamepad_state_hid_rp.inc
.. include-build-file:: inc/rumble_state.inc
