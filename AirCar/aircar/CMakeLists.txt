add_executable(aircar
        aircar.c
        "${CMAKE_CURRENT_LIST_DIR}/usb/src/usb_descriptors.c"
        "${CMAKE_CURRENT_LIST_DIR}/usb/src/msc_disk_dual.c"
)

# Make sure TinyUSB can find tusb_config.h
target_include_directories(aircar PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/usb/src"
)

# pull in common dependencies
target_link_libraries(aircar pico_stdlib hardware_pwm tinyusb_device tinyusb_board)

# create map/bin/hex file etc.
pico_add_extra_outputs(aircar)

