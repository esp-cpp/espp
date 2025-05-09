# Serialization Component

[![Badge](https://components.espressif.com/components/espp/serialization/badge.svg)](https://components.espressif.com/components/espp/serialization)

The serialization library is a light wrapper around the third-party `alpaca
<https://github.com/p-ranav/alpaca>`_ serialization library, providing
boilerplate-free binary (de-)serialization of arbitrary objects.

The `Serialization` component provides a simple wrapper with reasonable default
options around `alpaca`. The default serialization/deserialization options are
configured such that messages / types can be distinguished from each other.

## Example

The [example](./example) shows the use of the `serialization` component (and the
`alpaca` library it wraps) to perform robust and reliable binary serialization
and deserialization of arbitrary objects.

