# Expressive Eyes Example

Demonstrates animated expressive eyes on various display boards.

## Hardware Required

Select one of the supported boards in menuconfig:
- ESP32-S3-BOX / ESP32-S3-BOX-3
- MaTouch Rotary Display
- ESP-WROVER-KIT
- M5Stack Tab5

## How to Use

### Hardware Setup

Connect your chosen display board. No additional wiring is needed.

### Configure the Project

```bash
idf.py menuconfig
```

Navigate to `Expressive Eyes Example Configuration`:
- Select your board
- Configure eye appearance (size, spacing, color)
- Enable/disable auto demo mode
- Configure blink settings
- Enable/disable pupils

### Build and Flash

```bash
idf.py build flash monitor
```

## Features

### Auto Demo Mode (Enabled by default)

Automatically cycles through different expressions every 3 seconds (configurable):
- Neutral
- Happy
- Sad
- Angry
- Surprised
- Sleepy
- Wink Left
- Wink Right

Eyes also randomly look around and blink automatically.

### Manual Mode

Disable auto demo to show a single expression. Modify the code to control expressions programmatically.

### Customization

All parameters can be adjusted via menuconfig without code changes:
- Eye dimensions and spacing
- Pupil size (or disable pupils entirely)
- Blink frequency
- Demo interval
- Eye color (RGB565)

## Example Output

```
I (380) Expressive Eyes Example: Starting Expressive Eyes Example
I (425) Expressive Eyes Example: Display size: 320x240
I (430) Expressive Eyes Example: Expressive eyes initialized
I (435) Expressive Eyes Example: Starting auto demo mode (interval: 3000ms)
I (3440) Expressive Eyes Example: Changed to expression 1, looking at (0.42, -0.67)
I (6445) Expressive Eyes Example: Changed to expression 2, looking at (-0.23, 0.81)
```

## Troubleshooting

- **Display not working**: Check board selection in menuconfig
- **Eyes too small/large**: Adjust eye width/height in menuconfig
- **Performance issues**: Try reducing eye size or disabling pupils
