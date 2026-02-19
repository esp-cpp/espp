# Copilot Instructions for ESPP

## Repository Overview

ESPP (Espressif++) is a C++ component library for ESP32/ESP-IDF. It targets ESP-IDF v5.5.1+ and provides ~120 reusable components published to the [ESP Component Registry](https://components.espressif.com) under the `espp` namespace.

## Repository Structure

```
components/          # All library components (one directory per component)
doc/                 # Documentation (Doxygen + Sphinx/RST)
.github/workflows/   # CI: build.yml and upload_components.yml
lib/                 # External library submodules
pc/                  # PC-side utilities
python/              # Python utilities
```

## Component Structure

Every component follows this layout (use `components/task` as a reference):

```
components/<name>/
├── CMakeLists.txt          # idf_component_register(...)
├── README.md               # Component overview
├── idf_component.yml       # IDF Component Manager manifest
├── include/
│   └── <name>.hpp          # Public header(s)
├── src/
│   └── <name>.cpp          # Implementation (if any)
└── example/
    ├── CMakeLists.txt      # ESP-IDF project cmake
    ├── README.md           # Example documentation
    ├── sdkconfig.defaults  # Default config (add more for multiple targets)
    └── main/
        ├── CMakeLists.txt  # idf_component_register(...)
        └── <name>_example.cpp
```

Some examples also include `partitions.csv` or multiple `sdkconfig.defaults.*` files for different hardware targets.

## When Adding a New Component

Complete **all** of the following steps (in alphabetical order where required):

### 1. Component Files
- Create the full directory structure above under `components/<name>/`
- `CMakeLists.txt`: use `idf_component_register(INCLUDE_DIRS "include" SRC_DIRS "src" REQUIRES ...)`
- `idf_component.yml`: include license, description, url, repository, maintainers, documentation, examples, tags, and dependencies
- `example/CMakeLists.txt`: set `EXTRA_COMPONENT_DIRS` to `"../../../components/"` and list required components

### 2. Documentation — `doc/Doxyfile`
Add entries in **alphabetical order** to both sections:

```
INPUT = \
  ...
  $(PROJECT_PATH)/components/<name>/include/<name>.hpp \
  ...

EXAMPLE_PATH = \
  ...
  $(PROJECT_PATH)/components/<name>/example/main/<name>_example.cpp \
  ...
```

### 3. Documentation — `doc/en/`
Add two files (use existing files as a template):

- `doc/en/<name>.rst` — RST page describing the component API. Include a `toctree` pointing to the example and an `include-build-file` directive for the API reference.
- `doc/en/<name>_example.md` — Markdown file that simply includes the component's example README:
  ````
  ```{include} ../../components/<name>/example/README.md
  ```
  ````

If the component belongs to a logical group (e.g., `display`, `imu`, `network`), place the files in the appropriate subdirectory under `doc/en/`.

### 4. CI — `.github/workflows/build.yml`
Add a matrix entry in **alphabetical order** inside the `strategy.matrix.test` list. Choose the most appropriate target (`esp32`, `esp32s3`, or `esp32p4`):

```yaml
- path: 'components/<name>/example'
  target: esp32s3
```

### 5. CI — `.github/workflows/upload_components.yml`
Add the component path in **alphabetical order** inside the `components:` block:

```yaml
components/<name>
```

## Code Style

- C++20 standard
- Follow `.clang-format` for formatting
- Use `espp` namespace
- Follow the patterns of existing components for logging (`espp::Logger`), configuration structs (`Config`), and task usage

## Building & Testing

- Build examples using ESP-IDF v5.5.1: `idf.py build` from the example directory
- CI uses `espressif/esp-idf-ci-action@v1` with `esp_idf_version: v5.5.1`
- Pre-commit hooks are configured in `.pre-commit-config.yaml`

## Keeping Things Up to Date

When modifying an **existing** component:
- Update `README.md` and example `README.md` if behavior changes
- Update `idf_component.yml` version as appropriate
- Update the RST file in `doc/en/` if the API description changes
- Keep all entries in `Doxyfile`, `build.yml`, and `upload_components.yml` in alphabetical order
