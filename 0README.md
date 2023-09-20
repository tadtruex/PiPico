
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




