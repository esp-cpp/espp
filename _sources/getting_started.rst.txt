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

There are a few ways to pull ESPP components into your project. These are
**independent alternatives** — pick the one that best fits your situation. You
only need to do one of them; they are not steps to be run in sequence.

- **Start from the template repository (recommended for new projects).**
  `esp-cpp/template <https://github.com/esp-cpp/template>`_ is a ready-to-go
  starting point geared towards C++ development that is already set up to pull the
  ``espp`` components it needs via the IDF component manager. It is set up as a
  GitHub *template repository*, so you can create your own project from it
  directly using the green **"Use this template"** button on its `GitHub page
  <https://github.com/esp-cpp/template>`_ — no need to fork or clone it by hand.
  After creating your repository from the template, run the included setup script
  to customize and configure the project for your needs.

- **Add individual components with the IDF component manager.** All ``espp``
  components are published to the `ESP Component Registry
  <https://components.espressif.com/components?q=namespace:espp>`_ under the
  ``espp`` namespace, for example:

  .. code-block:: console

     idf.py add-dependency "espp/task^1.0"
     idf.py add-dependency "espp/ble_gatt_server^1.0"

- **Add espp to an existing project as a submodule.**

  .. note::

     This approach is **not recommended** and may not be fully supported going
     forward. espp is a large repository with many submodules and supports many
     targets, so vendoring the whole repository this way is heavyweight. Prefer
     the IDF component manager (above), which pulls in only the components you
     need; use the submodule approach only in exceptional cases where the
     component manager is not an option.

  If you already have a project with a ``components`` directory, add ``espp`` as a
  submodule there:

  .. code-block:: console

     git submodule add https://github.com/esp-cpp/espp components/espp
     git submodule update --init --recursive

  The ``--recursive`` flag is important: it pulls in the nested submodules that
  some components require. Then point ``EXTRA_COMPONENT_DIRS`` in your top-level
  ``CMakeLists.txt`` at the components:

  .. code-block:: cmake

     set(EXTRA_COMPONENT_DIRS
       "components/espp/components"
     )

For questions or to chat, join the `Discord
<https://discord.gg/dvcQw37xAY>`_.
