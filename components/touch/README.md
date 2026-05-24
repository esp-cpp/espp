# Touch Interface Component

[![Badge](https://components.espressif.com/components/espp/touch/badge.svg)](https://components.espressif.com/components/espp/touch)

The `touch` component defines shared touch data types and the runtime interface
used by ESPP touch controller drivers. It standardizes cached touch state,
single-point compatibility helpers, and runtime-polymorphic access for BSPs that
need to select between multiple touch controllers.

## Example

The [example](./example) shows how to implement and consume the shared
`ITouchDevice` interface.
