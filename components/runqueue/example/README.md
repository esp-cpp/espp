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

![CleanShot 2025-03-07 at 22 13 10@2x](https://github.com/user-attachments/assets/dedff19e-c40f-4451-a0e1-94e2b4a8835f)

