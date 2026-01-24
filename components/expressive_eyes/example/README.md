# Expressive Eyes Example

This example demonstrates animated expressive eyes on various ESP32 display
boards. It showcases different expressions, eye movements, and includes a
continuous random demo mode perfect for a desk display.

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
