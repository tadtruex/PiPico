add_executable(aircar
        aircar.c
        )

# pull in common dependencies
target_link_libraries(aircar pico_stdlib)

# create map/bin/hex file etc.
pico_add_extra_outputs(aircar)

