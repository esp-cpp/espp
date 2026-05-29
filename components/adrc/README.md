# ADRC (Active Disturbance Rejection Control) Component

[![Badge](https://components.espressif.com/components/espp/adrc/badge.svg)](https://components.espressif.com/components/espp/adrc)

The `adrc` component provides reusable active disturbance rejection control
implementations for ESPP applications.

## Features

- Linear first-order ADRC
- Linear second-order ADRC
- Han-style nonlinear first-order ADRC
- Han-style nonlinear second-order ADRC
- Han tracking differentiator utility for smoothing references and estimating
  reference rate
- Thread-safe configuration and state updates

## Example

The [example](./example) shows how to use the ADRC classes against simulated
first-order and second-order plants with injected disturbances.
