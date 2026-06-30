# Espressif++ (ESPP)

This is the repository for some c++ components developed for the
[ESP-IDF](https://github.com/espressif/esp-idf) framework. Specifically we are
targeting `ESP-IDF 6.0` currently, but we are maintaining support for `ESP-IDF
v5.5.1` as well.

> NOTE: This repo attempts to stay up to date with ESP-IDF. This means that the
> code within may not be supported on older ESP-IDF targets.

Each component has an `example` folder which contains c++ code showing how to
use the component and which has a README including instructions for how to run
the example

If you have questions or would like to chat, feel free to hop over to our discord!

[<img src="https://discord.com/api/guilds/1345508990716743741/widget.png?style=banner2" alt="Discord Banner 2"/>](https://discord.gg/dvcQw37xAY)

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [Espressif++ (ESPP)](#espressif-espp)
    - [Getting Started](#getting-started)
    - [Additional Information and Links](#additional-information-and-links)
    - [Developing](#developing)

<!-- markdown-toc end -->

## Getting Started

The components in this repository are targeted towards ESP-IDF >=5.0, though
they are mainly tested against 5.5.1 right now.

There are a few different ways to use the components in this repository. **These
are independent alternatives — pick the one that best fits your situation. You
only need to do one of them; they are not steps to be run one after another.**

- **Start from the template repository (recommended for new projects).** The
  [esp-cpp/template](https://github.com/esp-cpp/template) repository is a
  ready-to-go starting point that is similar to the esp-idf template but geared
  towards C++ development, with `espp` already added as a submodule and
  configured in the top-level `CMakeLists.txt`. It is set up as a GitHub
  *template repository*, so you can create your own project from it directly
  using the green **"Use this template"** button on its
  [GitHub page](https://github.com/esp-cpp/template) — there's no need to fork or
  clone it by hand. After creating your repository from the template, run the
  included setup script to customize and configure the project (renaming it,
  etc.) for your needs.

- **Add individual components with the IDF component manager.** All `espp`
  components are published to the [ESP Component
  Registry](https://components.espressif.com/components?q=namespace:espp) under
  the namespace `espp`. For example, if you want to use the `task` component and
  the `ble_gatt_server` component, you could run:

     ```console
     idf.py add-dependency "espp/task^1.0"
     idf.py add-dependency "espp/ble_gatt_server^1.0"
     ```

   Alternatively, you could add the following dependencies to your
   `main/idf_component.yml`:

     ```yaml
     dependencies:
        esp-cpp/ble_gatt_server: '>=1.0'
        esp-cpp/task: '>=1.0'
        # other dependencies here...
     ```

- **Add espp to an existing project as a submodule.** If you already have a
  project with a `components` directory, add `espp` as a submodule there:

    ```console
    git submodule add https://github.com/esp-cpp/espp components/espp
    git submodule update --init --recursive
    ```

  The `--recursive` flag is important: it pulls in the nested submodules that
  some components require. Afterwards, update your top-level `CMakeLists.txt` to
  add

    ```cmake
    # add the component directories that we want to use
    set(EXTRA_COMPONENT_DIRS
      "components/espp/components"
    )
    ```

- **Clone espp anywhere and reference its components.** You can also clone espp
  somewhere on your computer and then point your project to its `components`
  directory to use any of the components it contains, similar to the option
  above.

   Note: you should ensure that you clone recursively or run `git submodule
   update --init --recursive` to ensure that you have the latest versions of all
   the submodules which are required to build the components in this repository.

## Additional Information and Links

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

## Developing

If you plan to develop espp components, then it is recommended to configure your
development environment:

### Code style

1. Ensure `clang-format` is installed
2. Ensure [pre-commit](https://pre-commit.com) is installed
3. Set up `pre-commit` for this repository:

  ``` console
  pre-commit install
  ```

This helps ensure that consistent code formatting is applied.

### Building and viewing documentation locally

If you wish to build the documentation, you should have docker installed, then
you can run

``` console
docker build -t esp-docs doc
```

To build the docker container which you will use to build the documentation
locally (for testing).

Thereafter, you can use the included build script to build the documentation
locally.

``` console
./build_docs.sh
```
