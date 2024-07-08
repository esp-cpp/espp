State Machine APIs
******************

The `state_machine` component is a light wrapper around the `webgme-hfsm
<https://github.com/finger563/webgme-hfsm>`_ static generated code. It is
designed to be used as the component that one or more specific hfsms (manually written or
generated from webgme-hfsm) can depend on.

Code examples for the state_machine API are provided in the `state_machine`
example folder.

The example runs the generated code for the following example hsfm (which is
provided and for which the code was generated from webgme-hfsm):

.. image:: images/complex-hfsm.png
  :alt: "Complex" example HFSM showing the many of the UML formalisms supported.

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
