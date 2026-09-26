# espp `coredump` component — build-system integration for core dumps over USB.
#
# ESP-IDF includes a component's project_include.cmake (in project scope) only
# when that component is part of the build, so requiring the `coredump` component
# gives a project `coredump-usb` build targets: they pull the stored core dump
# off the device over its USB vendor (WebUSB) interface and decode it against
# the app ELF you just built, the way `idf.py coredump-info` does over the serial
# bootloader. That is the HOST half only -- the firmware must store core dumps to
# flash (CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH + a `coredump` partition) and serve
# an espp::CoreDumpService on a USB vendor interface, as the component's example
# (example/main/coredump_example.cpp) does; this file adds no device-side code.
#
#     idf.py coredump-usb          # builds the app, then downloads + decodes the dump
#     idf.py coredump-usb-debug    # same, but opens GDB on the core file instead
#     idf.py build coredump-usb    # equivalent explicit form (also works pre-CMake 3.19)
#
# (Mirrors ESP-IDF's own `coredump-info` / `coredump-debug` pair.) These are the
# FALLBACK: the component's idf_ext.py registers a real `idf.py coredump-usb`
# action with options (--gdb, --summary, --out, --vid/--pid/--serial), which
# idf.py prefers over a CMake target of the same name whenever it loads that
# extension (trusted component sources, or the espp wheel's entry point). idf.py
# cannot pass options to a custom target, so these take none; device overrides
# are read from the environment by the tool, e.g.:
#     ESPP_COREDUMP_PID=0x1234 idf.py coredump-usb
#
# The work is done by the pure-Python `espp_coredump` tool shipped alongside this
# file (components/coredump/python/). It needs `pyusb` at run time (not at build
# time) and the `esp-coredump` decoder (both ship in the ESP-IDF Python env):
#     pip install pyusb esp-coredump
#
# For full control (a specific serial, saving the core file elsewhere, or just
# the crash summary) run the tool directly:
#     python -m espp_coredump debug build/<app>.elf --help
#     python -m espp_coredump summary

if(NOT TARGET coredump-usb)
    idf_build_get_property(python PYTHON)
    set(__espp_coredump_pkg_dir "${CMAKE_CURRENT_LIST_DIR}/python")
    # CMAKE_PROJECT_NAME is already set here (the real project() runs before
    # idf_build_process includes this file); the app .elf lands in the build dir
    # under that name (project.cmake: `set(project_elf ${CMAKE_PROJECT_NAME}.elf)`).
    set(__espp_coredump_elf "${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.elf")

    # Prepend our package dir to PYTHONPATH rather than replacing it, so a
    # PYTHONPATH the environment already relies on is preserved. Use the host's
    # path separator. ($ENV{PYTHONPATH} is the value at configure time, which is
    # the same environment `idf.py coredump-usb` runs in.)
    if(WIN32)
        set(__espp_coredump_pathsep ";")
    else()
        set(__espp_coredump_pathsep ":")
    endif()
    set(__espp_coredump_pythonpath "${__espp_coredump_pkg_dir}")
    if(DEFINED ENV{PYTHONPATH} AND NOT "$ENV{PYTHONPATH}" STREQUAL "")
        set(__espp_coredump_pythonpath
            "${__espp_coredump_pkg_dir}${__espp_coredump_pathsep}$ENV{PYTHONPATH}")
    endif()

    # `debug` decodes with esp-coredump info_corefile; `debug --gdb` opens GDB on
    # the core file instead (esp-coredump dbg_corefile).
    add_custom_target(coredump-usb
        COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${__espp_coredump_pythonpath}"
                ${python} -m espp_coredump debug "${__espp_coredump_elf}"
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
        VERBATIM
        USES_TERMINAL
        COMMENT "Downloading the core dump over USB and decoding it against ${__espp_coredump_elf} (espp_coredump)")
    add_custom_target(coredump-usb-debug
        COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${__espp_coredump_pythonpath}"
                ${python} -m espp_coredump debug "${__espp_coredump_elf}" --gdb
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
        VERBATIM
        USES_TERMINAL
        COMMENT "Downloading the core dump over USB and opening GDB on it against ${__espp_coredump_elf} (espp_coredump)")

    # The app ELF is produced by the project executable target, which is defined
    # later in project.cmake, so add the build dependency once this directory
    # scope has finished processing. On CMake < 3.19 (no cmake_language(DEFER))
    # the targets still work via the explicit `idf.py build coredump-usb` form.
    function(__espp_coredump_link_build_dependency)
        idf_build_get_property(__espp_coredump_exe EXECUTABLE)
        foreach(__espp_coredump_target coredump-usb coredump-usb-debug)
            if(__espp_coredump_exe AND TARGET ${__espp_coredump_exe})
                add_dependencies(${__espp_coredump_target} ${__espp_coredump_exe})
            elseif(TARGET gen_project_binary)
                add_dependencies(${__espp_coredump_target} gen_project_binary)
            endif()
        endforeach()
    endfunction()
    if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.19")
        cmake_language(DEFER DIRECTORY "${CMAKE_SOURCE_DIR}"
                       CALL __espp_coredump_link_build_dependency)
    endif()
endif()
