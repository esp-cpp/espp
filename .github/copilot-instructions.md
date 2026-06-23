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

### 6. Pull Request Text
When adding a new component, also generate PR text using the template below.
Fill in the sections with the actual component name, files touched, test/build
results, and relevant motivation. Mark the applicable checkboxes, and remove the
Hardware section if it is not relevant to the PR.

```md
<!--- Provide a general summary of your changes in the Title above -->

## Description
<!--- Describe your changes in detail -->

## Motivation and Context
<!--- Why is this change required? What problem does it solve? -->
<!--- If it fixes an open issue, please link to the issue here. -->

## How has this been tested?
<!--- Please describe in detail how you tested your changes. -->
<!--- Include details of your testing environment, tests ran to see how -->
<!--- your change affects other areas of the code, etc. -->

## Screenshots (if appropriate, e.g. schematic, board, console logs, lab pictures):

## Types of changes
<!--- What types of changes does your code introduce? Put an `x` in all the boxes that apply: -->
- [ ] Bug fix (non-breaking change which fixes an issue)
- [ ] New feature (non-breaking change which adds functionality)
- [ ] Breaking change (fix or feature that would cause existing functionality to not work as expected)
- [ ] Documentation Update
- [ ] Hardware (schematic, board, system design) change
- [ ] Software change

## Checklist:
<!--- Go over all the following points, and put an `x` in all the boxes that apply. -->
<!--- If you're unsure about any of these, don't hesitate to ask. We're here to help! -->
- [ ] My change requires a change to the documentation.
- [ ] I have added / updated the documentation related to this change via either README or WIKI

### Software
<!-- Delete this section if not relevant to your PR  -->
- [ ] I have added tests to cover my changes.
- [ ] I have updated the `.github/workflows/build.yml` file to add my new test to the automated cloud build github action.
- [ ] All new and existing tests passed.
- [ ] My code follows the code style of this project.

### Hardware
<!-- Delete this section if not relevant to your PR  -->
- [ ] I have updated the design files (schematic, board, libraries).
- [ ] I have attached the PDFs of the SCH / BRD to this PR
- [ ] I have updated the design output (GERBER, BOM) files.
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
