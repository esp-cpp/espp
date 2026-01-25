# Expressive Eyes Component

[![Badge](https://components.espressif.com/components/espp/expressive_eyes/badge.svg)](https://components.espressif.com/components/espp/expressive_eyes)

The `ExpressiveEyes` component provides animated expressive eyes for displays
using simple blob shapes. Eyes can blink, look around, change expression, and
display various emotions with smooth animations.

The component uses a callback-based drawing system, allowing you to implement
custom renderers for different display types and visual styles.

## Features

- Multiple expressions (happy, sad, angry, surprised, neutral, sleepy, bored, wink_left, wink_right)
- Smooth eye movement with look_at positioning
- Automatic blinking with configurable intervals
- Optional pupils with physics-based movement
- Eyebrows and cheeks for enhanced expressions
- Smooth expression transitions with blending
- Customizable colors and sizes
- Frame-based animation system

<table>
 <tr>
   <td>happy <img alt="image" src="https://github.com/user-attachments/assets/b0cc5477-e0de-446d-bfd4-7f9e1d6d60e2" /></td>
   <td>sad <img alt="image" src="https://github.com/user-attachments/assets/12c4acc0-94ee-403b-b557-4062a1596681" /></td>
 </tr>
 <tr>
   <td>angry <img alt="image" src="https://github.com/user-attachments/assets/54059c71-3c0d-4d69-b5de-29e3a55df412" /></td>
   <td>surprised <img alt="image" src="https://github.com/user-attachments/assets/78cdbe54-2d7a-41b4-a521-70546af1dec7" /></td>
 </tr>
 <tr>
   <td>looking <img alt="image" src="https://github.com/user-attachments/assets/2fdfebb9-a241-4607-9f96-8b2082d51be1" /></td>
   <td>looking <img alt="image" src="https://github.com/user-attachments/assets/3b12cae4-4c89-4d8b-9038-25e3c09a5833" /></td>
 </tr>
</table>

Videos:

https://github.com/user-attachments/assets/42f5db22-1b2b-4a66-945f-542e57df55bf

## Example

The [example](./example) demonstrates the expressive eyes component with two
different drawer implementations (full-featured realistic eyes and monochrome
blue eyes) running on various ESP32 display boards.
