# Embedded course design

make

openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program build/embedded.hex verify reset exit"
