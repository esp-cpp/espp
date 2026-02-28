Color APIs
**********

The `color.hpp` header provides two classes for performing color management,
interpolation, and conversion - `espp::Rgb` and `espp::Hsv` which can be
converted between each other. The RGB color space provides support for additive
blending (which includes averaging, as opposed to light-model-based mixing) and
is therefore suited for producing gradients.

Please see `Computer Graphics and Geometric Modeling: Implementation and
Algorithms <https://isidore.co/calibre/browse/book/5588>`_, specifically section
8.6 for more information.

.. ------------------------------- Example -------------------------------------

.. toctree::

   color_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/color.inc
