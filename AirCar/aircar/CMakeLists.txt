add_executable(aircar
        aircar.c
        )

# Make sure TinyUSB can find tusb_config.h
target_include_directories(aircar PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/usb/src")

# pull in common dependencies
target_link_libraries(aircar pico_stdlib hardware_pwm tinyusb_device)

# create map/bin/hex file etc.
pico_add_extra_outputs(aircar)

