################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/src/stm32f401re_gpio_driver.c \
../drivers/src/stm32f401re_spi_driver.c 

OBJS += \
./drivers/src/stm32f401re_gpio_driver.o \
./drivers/src/stm32f401re_spi_driver.o 

C_DEPS += \
./drivers/src/stm32f401re_gpio_driver.d \
./drivers/src/stm32f401re_spi_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/src/%.o: ../drivers/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -DSTM32 -DSTM32F4 -DSTM32F401RETx -DNUCLEO_F401RE -DDEBUG -I"/Users/zhanibeksmac/Documents/OpenSTM Workspace/stm32f4xx_drivers/drivers/inc" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


