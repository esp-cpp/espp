# Expressive Eyes Example

This example demonstrates animated expressive eyes on various ESP32 display
boards. It showcases different expressions, eye movements, and includes a
continuous random demo mode perfect for a desk display.

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

```
I (380) Expressive Eyes Example: Starting Expressive Eyes Example
I (425) Expressive Eyes Example: Display size: 320x240
I (430) Expressive Eyes Example: Expressive eyes initialized
I (435) Expressive Eyes Example: Testing different expressions...
I (440) Expressive Eyes Example: Expression: NEUTRAL
I (3445) Expressive Eyes Example: Expression: HAPPY
I (6450) Expressive Eyes Example: Expression: SAD
I (9455) Expressive Eyes Example: Expression: ANGRY
I (12460) Expressive Eyes Example: Expression: SURPRISED
I (15465) Expressive Eyes Example: Testing look_at functionality
I (15470) Expressive Eyes Example: Looking left
I (16975) Expressive Eyes Example: Looking right
I (18480) Expressive Eyes Example: Looking up
I (19985) Expressive Eyes Example: Looking down
I (21490) Expressive Eyes Example: Looking center
I (22995) Expressive Eyes Example: Starting random demo mode - will run continuously
```
