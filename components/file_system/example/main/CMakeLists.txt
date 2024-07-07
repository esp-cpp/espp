idf_component_register(SRC_DIRS "."
                       INCLUDE_DIRS ".")

# Note: you must have a partition named the first argument (here it's "littlefs")
# in your partition table csv file.
littlefs_create_partition_image(littlefs ../files FLASH_IN_PROJECT)
