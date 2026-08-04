# PCA9535 / PCA9555 I/O Expander

[![Badge](https://components.espressif.com/components/espp/pca9535/badge.svg)](https://components.espressif.com/components/espp/pca9535)

The `PCA9535` / `PCA9555` I/O expander component allows the user to configure
inputs, outputs, input polarity inversion, etc. on a 16-bit (two 8-bit port)
GPIO expander via the I2C interface. The PCA9535 and PCA9555 share an identical
register map; the PCA9535 additionally provides an interrupt output, so a single
`espp::Pca9535` driver covers both parts.

## Example

The [example](./example) shows how to communicate with a PCA9535 / PCA9555 I2C
digital I/O expander using the `espp::Pca9535` component.
