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

Code examples for the Gamepad Map API are provided in the `gamepad_map` example
folder.

.. ------------------------------- Example -------------------------------------

.. toctree::

   gamepad_map_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/gamepad_state.inc
.. include-build-file:: inc/gamepad_map.inc
.. include-build-file:: inc/gamepad_profiles.inc
.. include-build-file:: inc/gamepad_state_hid_rp.inc
