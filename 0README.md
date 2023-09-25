
# Status

Nothing here.

# Programming

with openocd,

`openocd -f interface/cmsis-dap.cfg -c "adapter speed 5000" -f target/rp2040.cfg -c "program blink.elf reset exit"`

Debug with gdb (arm gnu eabi version buried deep in /opt)

`openocd -f interface/cmsis-dap.cfg -c "adapter speed 5000" -f target/rp2040.cfg

In another terminal

gdb blink.elf
$ target extended-remote :3333

Debug as usual.  sleep_ms does not seem to work.

# set up a cut down version of pico sdk.
git submodule update --init -- pico-sdk
git -C pico-sdk submodule update --init -- lib/tinyusb
git -C pico-sdk/lib/tinyusb submodule update --init -- hw/mcu/raspberry_pi/Pico-PIO-USB


