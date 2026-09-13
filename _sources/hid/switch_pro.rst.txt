Switch Pro Controller (NS1)
***************************

The `switch_pro` component provides ``espp::SwitchPro``, a **Nintendo Switch Pro
controller (NS1) USB emulation protocol engine**: it implements the Switch Pro
controller's USB HID handshake and input-report protocol so an ESP32-S3 (or other
native-USB ESP) can present itself to a Nintendo Switch as a Pro Controller.

It is transport-light — the class owns the controller state and the
request/response state machine but performs no USB I/O. Drive it from a USB HID
interface (the espp ``usb_device`` HID function is the intended pairing): feed
host OUTPUT reports (the handshake) to ``on_hid_report()`` and send the replies
(and the periodic standard input report) back as HID INPUT reports. The HID report
descriptor and report packing come from the ``hid-rp`` component.

.. warning::

   **Emulation only.** A real Switch only binds a device advertising Nintendo's
   Pro Controller USB VID/PID (``0x057E`` / ``0x2009``) and identity strings
   (exposed as ``espp::SwitchPro`` constants). Use these to emulate / test against
   a Switch you own; do not ship a product impersonating Nintendo hardware.

.. ------------------------------- Example -------------------------------------

.. toctree::

   switch_pro_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/switch_pro.inc
