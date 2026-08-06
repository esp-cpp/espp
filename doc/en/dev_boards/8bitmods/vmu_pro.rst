8BitMods VMU Pro
****************

VMU Pro
-------

The VMU Pro is an ESP32-S3 based replacement for the Sega Dreamcast Visual
Memory Unit (VMU) which doubles as a standalone handheld gaming device. It
features a 1.5" 240x240 IPS TFT color display, a D-pad and several buttons, a
mono speaker, a micro-SD card slot, and a rechargeable battery with USB-C
charging.

See the `VMU Pro product page
<https://8bitmods.com/vmupro-handheld-visual-memory-card-for-dreamcast-classic-white/>`_
for more information.

The `espp::VmuPro` component provides a singleton hardware abstraction for
initializing the various subsystems.

.. warning::

   The GPIO assignments in this BSP are UNVERIFIED placeholders. The VMU Pro's
   schematic is not publicly available, so the pin numbers in the component
   must be corrected against real hardware or vendor documentation before use.

.. ------------------------------ Example  -------------------------------------

.. toctree::

   vmu_pro_example.md

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/vmu-pro.inc
