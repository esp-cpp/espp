Expressive Eyes APIs
********************

The `ExpressiveEyes` class provides an animated expressive eyes system for
displays using simple blob shapes. It supports multiple expressions, smooth
eye movement, blinking, and optional physics-based pupil movement.

The component uses a callback-based drawing system, allowing you to implement
custom renderers for different display types and visual styles. Example drawer
implementations are provided showing both realistic eyes with pupils and
minimalist monochrome designs.

Features include:

- Multiple expressions (happy, sad, angry, surprised, neutral, sleepy, bored, wink_left, wink_right)
- Smooth eye movement with look_at positioning
- Automatic blinking with configurable intervals
- Optional pupils with physics-based movement
- Eyebrows and cheeks for enhanced expressions
- Smooth expression transitions with blending
- Customizable colors and sizes
- Frame-based animation system

The drawing callback receives complete eye state information including
position, size, expression parameters, pupil position, eyebrow configuration,
and more, giving you full control over the rendering.

.. ------------------------------- Example -------------------------------------

.. toctree::

   expressive_eyes_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/expressive_eyes.inc
