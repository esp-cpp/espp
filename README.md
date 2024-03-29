# Espressif++ (ESPP)

This is the repository for some c++ components developed for the
[ESP-IDF](https://github.com/espressif/esp-idf) framework.

 * [Documentation](https://esp-cpp.github.io/espp/) - github hosted version of
   the documentation found in [./docs](./docs). This documentation is
   automatically built as part of the CI, but can be locally built for
   validation by running [./build_docs.sh](./build_docs.sh). NOTE: to ensure
   proper build environments, the local documentation build relies on docker, so
   you'll need to run `docker build -t esp-docs doc` once before running
   `build_docs.sh`. This is only required if you want to build the documentation
   locally.

Many components in this repo contain example code (referenced in the
documentation above) that shows some basic usage. This example code can be found
in that component's `example` directory. NOTE: many component examples also make
use of other components (esp. some of the foundational components such as
`format`, `logger`, and `task`.

The [esp-cpp github organization](https://github.com/esp-cpp) contains other
repositories that build specific demonstrations with these (and other)
components.

One such example is the [esp-box-emu](http://github.com/esp-cpp/esp-box-emu)
repository, which builds upon these components to create a multiplatform
emulation system using the ESP32-S3-BOX hardware.
