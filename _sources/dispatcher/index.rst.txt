Dispatcher APIs
***************

.. toctree::
    :maxdepth: 1

    dispatcher
    custom_modules

The `Dispatcher` component multiplexes several independent framed protocols over
a single byte stream. It parses the :doc:`../stream_frame/index` codec once and
routes each frame to a per-module handler by the frame's ``module`` byte, so
OTA, crash-dump inspection, a CAN bridge and an application's own control
channel can share one USB / socket / UART link without interfering.

Building your own protocol on top of this (a device-side module plus a
browser web app that talks to it)? See :doc:`custom_modules` for an in-depth,
worked tutorial.
