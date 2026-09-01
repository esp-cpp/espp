idf_component_register(SRC_DIRS "."
                       INCLUDE_DIRS "."
                       REQUIRES bldc_driver bldc_haptics bldc_motor i2c motorgo-axis motorgo-mini
                                mt6701 ota stream_frame task usb_device esp_tinyusb esp_timer espcoredump)
