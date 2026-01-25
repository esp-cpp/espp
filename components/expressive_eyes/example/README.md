# Expressive Eyes Example

This example demonstrates animated expressive eyes on various ESP32 display
boards. It showcases different expressions, eye movements, and includes a
continuous random demo mode perfect for a desk display.

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

Videos:

https://github.com/user-attachments/assets/966f5e63-aeb8-4fc3-b915-c4936fee7a1f

## How to use example

### Hardware Required

This example can be configured to run on the following dev boards:
- ESP32-S3-BOX / ESP32-S3-BOX-3
- MaTouch Rotary Display
- WS-S3-Touch

### Configure the project

```
idf.py menuconfig
```

Navigate to `Expressive Eyes Example Configuration` to:
- Select your board
- Choose drawing method (Full Featured or Monochrome Blue)

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Breakdown

The example has three phases:

### 1. Expression Showcase

Cycles through all available expressions (3 seconds each):
- Neutral
- Happy
- Sad
- Angry
- Surprised

### 2. Look Direction Demo

Demonstrates the look_at functionality by looking in different directions (1.5 seconds each):
- Left
- Right
- Up
- Down
- Center

### 3. Random Demo Mode

Enters continuous random demo mode for engaging desk display:
- **Natural behavior**: Mostly stays neutral with occasional emotional expressions
- **Random looking**: Eyes look in random directions every 2-6 seconds
- **Expression changes**: Randomly changes expression every 5-15 seconds
  - 50% neutral (most common)
  - 20% happy
  - 10% surprised
  - 10% sad
  - 10% angry
- **Automatic blinking**: Eyes blink naturally at random intervals

The random demo mode runs indefinitely, making it perfect for a continuous desk display.

## Drawer Implementations

The example includes two drawer implementations:

### Full Featured Drawer
- White eye background with black pupils
- Eyebrows drawn as rotated lines with rounded caps
- Cheeks for emotional expressions
- Respects LVGL theme background color

### Monochrome Blue Drawer
- Electric blue (#00BFFF) eyes on black background
- No pupils (solid colored eyes)
- Minimalist aesthetic perfect for sci-fi interfaces
- Black eyebrows and cheeks

See [README_DRAWERS.md](./main/README_DRAWERS.md) for details on creating custom drawer implementations.

## Example Output

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

<img width="800" height="470" alt="image" src="https://github.com/user-attachments/assets/fcdae7f6-8bcc-4d0a-bf05-38573cb492ad" />

```console
I (886) main_task: Calling app_main()
[Expressive Eyes Example/I][0.889]: Starting Expressive Eyes Example
[WsS3Touch/I][0.899]: Initializing LCD...
[WsS3Touch/I][0.900]: Initializing SPI...
[WsS3Touch/I][0.906]: Adding device to SPI bus...
[WsS3Touch/I][0.911]: Initializing the display driver...
[WsS3Touch/I][1.321]: Display driver initialized successfully!
W (1322) ledc: the binded timer can't keep alive in sleep
[WsS3Touch/I][1.324]: Initializing display with pixel buffer size: 12000
[WsS3Touch/I][1.334]: Display initialized successfully!
[Expressive Eyes Example/I][1.335]: Display size: 240x280
[Expressive Eyes Example/I][1.342]: Using Monochrome Blue drawer
[Expressive Eyes Example/I][1.392]: Expressive eyes initialized
[Expressive Eyes Example/I][1.392]: Testing different expressions...
[Expressive Eyes Example/I][1.395]: Expression: Neutral
[Expressive Eyes Example/I][11.928]: Expression: Happy
[Expressive Eyes Example/I][22.548]: Expression: Sad
[Expressive Eyes Example/I][33.168]: Expression: Angry
[Expressive Eyes Example/I][43.788]: Expression: Surprised
[Expressive Eyes Example/I][54.407]: Testing look_at functionality
[Expressive Eyes Example/I][54.408]: Looking left
[Expressive Eyes Example/I][59.717]: Looking right
[Expressive Eyes Example/I][65.028]: Looking up
[Expressive Eyes Example/I][70.337]: Looking down
[Expressive Eyes Example/I][75.648]: Looking center
[Expressive Eyes Example/I][80.957]: Starting random demo mode - will run continuously
```
