# Expressive Eyes Component

[![Badge](https://components.espressif.com/components/espp/expressive_eyes/badge.svg)](https://components.espressif.com/components/espp/expressive_eyes)

The `ExpressiveEyes` component provides animated expressive eyes for displays
using simple blob shapes. Eyes can blink, look around, change expression, and
display various emotions with smooth animations.

The component uses a callback-based drawing system, allowing you to implement
custom renderers for different display types and visual styles.

https://github.com/user-attachments/assets/966f5e63-aeb8-4fc3-b915-c4936fee7a1f

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
   <td>happy <img alt="image" src="https://github.com/user-attachments/assets/65f3afd4-fa4e-4100-9269-09b08300d37d" /></td>
   <td>sad <img alt="image" src="https://github.com/user-attachments/assets/d3df6898-dae4-4884-b72a-81a5c5a371ed" /></td>
 </tr>
 <tr>
   <td>angry <img alt="image" src="https://github.com/user-attachments/assets/43e54326-8da3-4d6b-8696-66fb85d2675c" /></td>
   <td>surprised <img alt="image" src="https://github.com/user-attachments/assets/a78b289c-7383-4b92-8651-bdc843b08bf7" /></td>
 </tr>
 <tr>
   <td>sleepy <img alt="image" src="https://github.com/user-attachments/assets/b891e260-97dd-4cce-9d80-2300816b6e79" /></td>
   <td>bored <img alt="image" src="https://github.com/user-attachments/assets/cc955086-bb69-41da-becc-80584b0af5c0" /></td>
 </tr>
 <tr>
   <td>wink left <img alt="image" src="https://github.com/user-attachments/assets/a1dde472-2bfa-480f-8a87-4deccd3b96b9" /></td>
   <td>wink right <img alt="image" src="https://github.com/user-attachments/assets/d92f8602-f287-441b-9612-50210be5831a" /></td>
 </tr>
 <tr>
   <td>looking left <img alt="image" src="https://github.com/user-attachments/assets/504d90a8-2125-45f7-8eeb-44ca6aa05cba" /></td>
   <td>looking right <img alt="image" src="https://github.com/user-attachments/assets/58626990-37cf-43d0-a8a6-e5ed01bd8b7c" /></td>
 </tr>
 <tr>
   <td>looking up <img alt="image" src="https://github.com/user-attachments/assets/1765eb2d-7c29-44c9-bbed-658af2ccabd4" /></td>
   <td>looking down <img alt="image" src="https://github.com/user-attachments/assets/65c19bb7-e8c4-4cee-93ba-a5be96cb2df7" /></td>
 </tr>
</table>

## Example

The [example](./example) demonstrates the expressive eyes component with two
different drawer implementations (full-featured realistic eyes and monochrome
blue eyes) running on various ESP32 display boards.
