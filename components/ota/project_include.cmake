# espp `ota` component — build-system integration for OTA-over-USB.
#
# Included automatically by ESP-IDF (in project scope) for any project that uses
# the `ota` component. It registers an `ota-usb` build target so you can build
# and OTA-flash your app over USB in one step, the same way `idf.py flash` works
# for the serial bootloader:
#
#     idf.py ota-usb          # builds the app, then OTAs it over USB
#     idf.py build ota-usb    # equivalent explicit form (also works pre-CMake 3.19)
#
# Device/port overrides are read from the environment by the tool, e.g.:
#     ESPP_OTA_PID=0x1234 idf.py ota-usb
#
# The work is done by the pure-Python `espp_ota` tool shipped alongside this file
# (components/ota/python/). It needs `pyusb` at flash time (not at build time):
#     pip install pyusb
#
# For full control (a specific serial, chunk size, discovery probe, ...) run the
# tool directly:  python -m espp_ota flash build/<app>.bin --help

if(NOT TARGET ota-usb)
    idf_build_get_property(python PYTHON)
    set(__espp_ota_pkg_dir "${CMAKE_CURRENT_LIST_DIR}/python")
    # CMAKE_PROJECT_NAME is already set here (the real project() runs before
    # idf_build_process includes this file); the app .bin lands in the build dir.
    set(__espp_ota_bin "${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin")

    add_custom_target(ota-usb
        COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${__espp_ota_pkg_dir}"
                ${python} -m espp_ota flash "${__espp_ota_bin}"
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
        VERBATIM
        USES_TERMINAL
        COMMENT "OTA-flashing ${__espp_ota_bin} over USB (espp_ota)")

    # `gen_project_binary` (the target that produces the app .bin) is defined
    # later in project.cmake, so add the build dependency once this directory
    # scope has finished processing. On CMake < 3.19 (no cmake_language(DEFER))
    # the target still works via the explicit `idf.py build ota-usb` form.
    function(__espp_ota_link_build_dependency)
        if(TARGET gen_project_binary)
            add_dependencies(ota-usb gen_project_binary)
        endif()
    endfunction()
    if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.19")
        cmake_language(DEFER DIRECTORY "${CMAKE_SOURCE_DIR}"
                       CALL __espp_ota_link_build_dependency)
    endif()
endif()
