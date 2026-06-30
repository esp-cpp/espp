Getting Started
***************

ESPP (`espp <https://github.com/esp-cpp/espp>`_) is a collection of C++
components and abstractions for the `ESP-IDF
<https://github.com/espressif/esp-idf>`_ framework. Each component is reusable on
its own, and many also build on a few foundational components such as
``format``, ``logger``, and ``task``.

ESPP currently targets ESP-IDF ``6.0`` while maintaining support for ``5.5.1``.

How this documentation is organized
===================================

The APIs are grouped by capability — :doc:`core/index`, :doc:`sensors
<imu/index>`, :doc:`buses/index`, :doc:`network/index`, and so on — with all of
the supported development boards collected under :doc:`dev_boards/index`. Use the
navigation sidebar to browse a category, or the search box to jump straight to a
component.

Every component has:

- an **API Reference** page (the pages linked from each category), and
- an **example** that shows basic usage. The example code lives in that
  component's ``example`` directory in the repository and is embedded in the
  component's documentation page.

Using a component
=================

There are a few ways to pull ESPP components into your project:

#. Start from the `esp-cpp/template <https://github.com/esp-cpp/template>`_
   repository, which is set up for C++ development with ``espp`` already included
   as a submodule.

#. Add individual components with the IDF component manager. All ``espp``
   components are published to the `ESP Component Registry
   <https://components.espressif.com/components?q=namespace:espp>`_ under the
   ``espp`` namespace, for example:

   .. code-block:: console

      idf.py add-dependency "espp/task^1.0"
      idf.py add-dependency "espp/ble_gatt_server^1.0"

#. Clone ``espp`` as a submodule in your project's ``components`` directory and
   add it to ``EXTRA_COMPONENT_DIRS`` in your ``CMakeLists.txt``:

   .. code-block:: cmake

      set(EXTRA_COMPONENT_DIRS
        "components/espp/components"
      )

   Be sure to clone recursively (or run ``git submodule update --init
   --recursive``) so the required submodules are present.

For questions or to chat, join the `Discord
<https://discord.gg/dvcQw37xAY>`_.
