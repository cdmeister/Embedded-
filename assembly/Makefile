all: hello.bin

hello.o: hello_world.s
	arm-none-eabi-as -mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard  -Og -o hello.o hello_world.s

sys_start.o: ./startup/system_stm32f4xx.c
	arm-none-eabi-gcc -c -mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard  -Og  -DSTM32F407xx -Istartup   -o sys_start.o ./startup/system_stm32f4xx.c

startup.o: ./startup/startup_stm32f407xx.s
	arm-none-eabi-as -mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard  -Og -o startup.o ./startup/startup_stm32f407xx.s

hello.elf: hello.o startup.o sys_start.o
	arm-none-eabi-gcc  -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard  -Wl,--gc-sections -specs=nano.specs -lc -lm -lnosys -T./linker/STM32F407VGTx_FLASH.ld -o hello.elf hello.o startup.o sys_start.o

hello.bin: hello.elf
	arm-none-eabi-objcopy -O binary hello.elf hello.bin
