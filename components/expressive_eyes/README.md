# Expressive Eyes Component

[![Badge](https://components.espressif.com/components/espp/expressive_eyes/badge.svg)](https://components.espressif.com/components/espp/expressive_eyes)

The `ExpressiveEyes` component provides animated expressive eyes for displays
using simple blob shapes. Eyes can blink, look around, change expression, and
display various emotions with smooth animations.

The component uses a callback-based drawing system, allowing you to implement
custom renderers for different display types and visual styles.

## Features

- Multiple expressions (happy, sad, angry, surprised, neutral, sleepy, wink)
- Smooth eye movement with look_at positioning
- Automatic blinking with configurable intervals
- Optional pupils with physics-based movement
- Eyebrows and cheeks for enhanced expressions
- Customizable colors and sizes
- Frame-based animation system

## Example

The [example](./example) demonstrates the expressive eyes component with two
different drawer implementations (full-featured realistic eyes and monochrome
blue eyes) running on various ESP32 display boards.
