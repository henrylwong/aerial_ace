################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/adc_setup.c \
../Core/Src/calculate_gestures.c \
../Core/Src/calculate_orientation.c \
../Core/Src/dac_setup.c \
../Core/Src/imu_setup.c \
../Core/Src/lcd.c \
../Core/Src/lcd_setup.c \
../Core/Src/lsm9ds1_reg.c \
../Core/Src/main.c \
../Core/Src/stm32f3xx_hal_msp.c \
../Core/Src/stm32f3xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f3xx.c \
../Core/Src/utils.c 

OBJS += \
./Core/Src/adc_setup.o \
./Core/Src/calculate_gestures.o \
./Core/Src/calculate_orientation.o \
./Core/Src/dac_setup.o \
./Core/Src/imu_setup.o \
./Core/Src/lcd.o \
./Core/Src/lcd_setup.o \
./Core/Src/lsm9ds1_reg.o \
./Core/Src/main.o \
./Core/Src/stm32f3xx_hal_msp.o \
./Core/Src/stm32f3xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f3xx.o \
./Core/Src/utils.o 

C_DEPS += \
./Core/Src/adc_setup.d \
./Core/Src/calculate_gestures.d \
./Core/Src/calculate_orientation.d \
./Core/Src/dac_setup.d \
./Core/Src/imu_setup.d \
./Core/Src/lcd.d \
./Core/Src/lcd_setup.d \
./Core/Src/lsm9ds1_reg.d \
./Core/Src/main.d \
./Core/Src/stm32f3xx_hal_msp.d \
./Core/Src/stm32f3xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f3xx.d \
./Core/Src/utils.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xC -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/adc_setup.cyclo ./Core/Src/adc_setup.d ./Core/Src/adc_setup.o ./Core/Src/adc_setup.su ./Core/Src/calculate_gestures.cyclo ./Core/Src/calculate_gestures.d ./Core/Src/calculate_gestures.o ./Core/Src/calculate_gestures.su ./Core/Src/calculate_orientation.cyclo ./Core/Src/calculate_orientation.d ./Core/Src/calculate_orientation.o ./Core/Src/calculate_orientation.su ./Core/Src/dac_setup.cyclo ./Core/Src/dac_setup.d ./Core/Src/dac_setup.o ./Core/Src/dac_setup.su ./Core/Src/imu_setup.cyclo ./Core/Src/imu_setup.d ./Core/Src/imu_setup.o ./Core/Src/imu_setup.su ./Core/Src/lcd.cyclo ./Core/Src/lcd.d ./Core/Src/lcd.o ./Core/Src/lcd.su ./Core/Src/lcd_setup.cyclo ./Core/Src/lcd_setup.d ./Core/Src/lcd_setup.o ./Core/Src/lcd_setup.su ./Core/Src/lsm9ds1_reg.cyclo ./Core/Src/lsm9ds1_reg.d ./Core/Src/lsm9ds1_reg.o ./Core/Src/lsm9ds1_reg.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32f3xx_hal_msp.cyclo ./Core/Src/stm32f3xx_hal_msp.d ./Core/Src/stm32f3xx_hal_msp.o ./Core/Src/stm32f3xx_hal_msp.su ./Core/Src/stm32f3xx_it.cyclo ./Core/Src/stm32f3xx_it.d ./Core/Src/stm32f3xx_it.o ./Core/Src/stm32f3xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f3xx.cyclo ./Core/Src/system_stm32f3xx.d ./Core/Src/system_stm32f3xx.o ./Core/Src/system_stm32f3xx.su ./Core/Src/utils.cyclo ./Core/Src/utils.d ./Core/Src/utils.o ./Core/Src/utils.su

.PHONY: clean-Core-2f-Src

