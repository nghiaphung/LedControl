CROSS_COMPILE = arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld
AR = $(CROSS_COMPILE)ar
AS = $(CROSS_COMPILE)as
OC = $(CROSS_COMPILE)objcopy
OD = $(CROSS_COMPILE)objdump
SZ = $(CROSS_COMPILE)size

CFLAGS = -c -fno-common \
		-ffunction-sections \
		-fdata-sections \
		-Os\
		-mcpu=cortex-m0 -Wall \
		-mthumb

LDSCRIPT = ldscripts/stm32_flash.ld
LDFLAGS	 = --gc-sections,-T$(LDSCRIPT),-no-startup,-nostdlib,-lnosys
OCFLAGS  = -Obinary
ODFLAGS  = -S
OUTPUT_DIR = bin
TARGET = $(OUTPUT_DIR)/main

INCLUDE = -I./kernel/stm32f0_hal/CMSIS/Include \
		-I./kernel/stm32f0_hal \
		-I./kernel/stm32f0_hal/config \
		-I./kernel/stm32f0_hal/STM32F0xx_StdPeriph_Driver/inc \
		-I./kernel/stm32f0_hal/vectors \
		-I./kernel/common \
		-I./kernel/init \
		-I./OS/FreeRTOS/include \
		-I./OS/FreeRTOS/portable/GCC/ARM_CM0 \
		-I./OS/FreeRTOS/config \
		-I./drivers/led \
		-I./drivers/serial \
		-I./drivers/timer \
		-I./drivers/adc


SRCS = ./kernel/stm32f0_hal/system_stm32f0xx.c \
		./kernel/stm32f0_hal/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_adc.c \
		./kernel/stm32f0_hal/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_exti.c \
		./kernel/stm32f0_hal/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_flash.c \
		./kernel/stm32f0_hal/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_gpio.c \
		./kernel/stm32f0_hal/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_i2c.c \
		./kernel/stm32f0_hal/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_misc.c \
		./kernel/stm32f0_hal/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_rcc.c \
		./kernel/stm32f0_hal/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_spi.c \
		./kernel/stm32f0_hal/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_syscfg.c \
		./kernel/stm32f0_hal/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_tim.c \
		./kernel/stm32f0_hal/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_usart.c \
		./OS/FreeRTOS/portable/MemMang/heap_4.c \
		./OS/FreeRTOS/tasks.c \
		./OS/FreeRTOS/list.c \
		./OS/FreeRTOS/queue.c \
		./OS/FreeRTOS/timers.c \
		./OS/FreeRTOS/portable/GCC/ARM_CM0/port.c \
		./libs/newlib/_syscalls.c \
		./libs/newlib/assert.c \
		./libs/newlib/_sbrk.c \
		./libs/newlib/_exit.c \
		./kernel/init/clock.c \
		./drivers/led/led.c \
		./drivers/serial/serial.c \
		./drivers/timer/timer.c \
		./main/main.c

OBJS = $(SRCS:.c=.o)
.PHONY : clean all

all: $(TARGET).bin  $(TARGET).list
	$(SZ) $(TARGET).elf

clean:
	-find . -name '*.o'   -exec rm {} \;
	-find . -name '*.elf' -exec rm {} \;
	-find . -name '*.lst' -exec rm {} \;
	-find . -name '*.out' -exec rm {} \;
	-find . -name '*.bin' -exec rm {} \;
	-find . -name '*.map' -exec rm {} \;

$(TARGET).list: $(TARGET).elf
	$(OD) $(ODFLAGS) $< > $(TARGET).lst

$(TARGET).bin: $(TARGET).elf
	$(OC) $(OCFLAGS) $(TARGET).elf $(TARGET).bin

$(TARGET).elf: $(OBJS) ./kernel/init/startup_stm32f0xx.o
	@$(CC) -mcpu=cortex-m0 -mthumb -Wl,$(LDFLAGS),-o$(TARGET).elf,-Map,$(TARGET)\
	.map ./kernel/init/startup_stm32f0xx.o $(OBJS)

%.o: %.c
	@echo "  CC $<"
	@$(CC) $(INCLUDE) $(CFLAGS)  $< -o $*.o

%.o: %.S
	@echo "  CC $<"
	@$(CC) $(INCLUDE) $(CFLAGS)  $< -o $*.o
