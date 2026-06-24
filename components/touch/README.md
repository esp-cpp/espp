# Touch Interface Component

[![Badge](https://components.espressif.com/components/espp/touch/badge.svg)](https://components.espressif.com/components/espp/touch)

The `touch` component defines shared touch data types and the runtime interface
used by ESPP touch controller drivers. It standardizes cached touch state,
single-point compatibility helpers, and runtime-polymorphic access for BSPs that
need to select between multiple touch controllers.

In addition to the `ITouchDevice` interface, the component provides the
type-erasure helpers `TouchDriverConcept`, `ITouchDriver`, `TouchDriverAdapter`,
and the `make_touch_driver()` factory. These let a BSP wrap any concrete driver
(e.g. `Gt911` or `St7123Touch`) in a single `std::shared_ptr<ITouchDriver>`
regardless of which controller is present at runtime.

## Example

The [example](./example) shows how to implement and consume the shared
`ITouchDevice` interface.
