idf_component_register(SRCS "usb_host_tab5_example.cpp" "gui.cpp" "spacemouse_decoder.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES usb_host hid-rp m5stack-tab5 display lvgl logger task format)
