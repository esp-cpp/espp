# PID (Proportional-Integral-Derivative) Feedback Control Component

The `PID` component provides a simple, thread-safe class representing a PID
controller. It tracks how frequently its `update()` method is called and can
have its gains change dynamically.

## Example

The [example](./example) shows how to use the `espp::Pid` class to perform closed-loop
feedback control using proportional-integral-derivative (PID) control.
