State Machine APIs
******************

The `state_machine` component is a light wrapper around the `webgme-hfsm
<https://github.com/finger563/webgme-hfsm>`_ static generated code. It is
designed to be used as the component that one or more specific hfsms (manually written or
generated from webgme-hfsm) can depend on.

Code examples for the state_machine API are provided in the `state_machine`
example folder.

The example runs the generated code for the following example hsfm, which is
provided as a model (``components/state_machine/example/main/Complex.json``)
and generated to C++ as part of the example's build:

.. image:: images/complex-hfsm.png
  :alt: "Complex" example HFSM showing the many of the UML formalisms supported.

.. ------------------------------ Playground -----------------------------------

Modeling and generating a state machine
---------------------------------------

State machines are modeled and generated with `webgme-hfsm
<https://github.com/finger563/webgme-hfsm>`__. The **HFSM Playground** runs the
whole toolchain in the browser -- edit the machine, simulate it, and read or
download the generated C++ -- with nothing to install and no server:

`Open the HFSM Playground in a new tab <https://finger563.github.io/webgme-hfsm/>`_,
or go straight to `this example's machine
<https://finger563.github.io/webgme-hfsm/?example=Complex&view=diagram>`_.

The playground below is live, not a picture: drag the states, press
**HFSM-Restart** and send events to watch the machine run, or switch to
**Code** to read the C++ it generates.

.. raw:: html

   <iframe src="https://finger563.github.io/webgme-hfsm/?example=Complex&amp;view=diagram&amp;embed=1"
           title="The Complex example HFSM in the HFSM Playground"
           loading="lazy"
           style="width:100%;height:640px;border:1px solid var(--color-background-border,#ccc);border-radius:8px;margin-top:0.5em"></iframe>

The same generator runs on the command line, which is how the example builds
its C++ from the model:

.. code-block:: sh

   npx -p webgme-hfsm hfsm-gen my_machine.json -o generated --no-support

``--no-support`` leaves out the shared runtime (``state_base.hpp``, the
history states, ``magic_enum.hpp``) because this component already provides
it. See the example's README for how that is wired into CMake.

.. ------------------------------- Example -------------------------------------

.. toctree::

   state_machine_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/state_machine.inc
.. include-build-file:: inc/state_base.inc
.. include-build-file:: inc/shallow_history_state.inc
.. include-build-file:: inc/deep_history_state.inc
