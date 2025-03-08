# RunQueue Example

This example shows how you can use the `espp::RunQueue` to schedule functions to
run asynchronously with priority ordering. You can configure the RunQueue task
so that you have different RunQueue tasks for different groups of function
priorities as well as so that you can have a RunQueue on each core if you like.

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

![CleanShot 2025-03-07 at 21 55 15@2x](https://github.com/user-attachments/assets/d339b4ab-0cf6-4b90-85c3-4c228e913a92)

