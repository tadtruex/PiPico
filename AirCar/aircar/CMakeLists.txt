add_executable(aircar
        aircar.c
        "${CMAKE_CURRENT_LIST_DIR}/usb/src/usb_descriptors.c"
        "${CMAKE_CURRENT_LIST_DIR}/usb/src/msc_disk_dual.c"
        "${CMAKE_CURRENT_LIST_DIR}/fatfs/source/ff.c"
        "${CMAKE_CURRENT_LIST_DIR}/fatfs/source/ffsystem.c"
        "${CMAKE_CURRENT_LIST_DIR}/fatfs/source/ffunicode.c"
        "${CMAKE_CURRENT_LIST_DIR}/fatfs/source/test_diskio.c"
        "${CMAKE_CURRENT_LIST_DIR}/disk_driver/diskio.c"
        "${CMAKE_CURRENT_LIST_DIR}/disk_driver/ramdisk.c"
	)

# Make sure TinyUSB can find tusb_config.h
target_include_directories(aircar PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/usb/src"
        "${CMAKE_CURRENT_LIST_DIR}/fatfs/source"
        "${CMAKE_CURRENT_LIST_DIR}/disk_driver/"
)

# pull in common dependencies
target_link_libraries(aircar pico_stdlib hardware_pwm tinyusb_device tinyusb_board)

# create map/bin/hex file etc.
pico_add_extra_outputs(aircar)

