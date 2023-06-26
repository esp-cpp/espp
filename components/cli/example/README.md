# Cli Example

This example shows how to use the `Cli` component to setup and run a command
line interface (CLI) on the ESP.

The `Cli` and associated `LineInput` classes support:
  *   ctrl+a (move to beginning of line)
  *   ctrl+e (move to end of line)
  *   ctrl+n (move up a line / previous input history)
  *   ctrl+p (move down a line / next input history)
  *   ctrl+k (delete from the cursor to the end of the line)
  *   ctrl+b (move the cursor back one character)
  *   ctrl+f (move the cursor forward one character)
  *   ctrl+l (clear the screen)

## How to use example

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

![output](https://user-images.githubusercontent.com/213467/231225408-90881f56-901a-4d7f-9649-1ddd4d3ea560.png)

https://user-images.githubusercontent.com/213467/231188170-47f45641-fc44-4cc6-b673-506e643a5e02.mp4

Showing terminal resize and support for `ctrl+l`:

https://github.com/esp-cpp/espp/assets/213467/54c362a7-99f2-4d8a-aa26-4498646f86e6

