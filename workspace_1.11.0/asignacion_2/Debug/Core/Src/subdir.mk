################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Servo.c \
../Core/Src/StepMotor.c \
../Core/Src/i2c-lcd.c \
../Core/Src/main.c \
../Core/Src/max_matrix_stm32.c \
../Core/Src/stm32f3xx_hal_msp.c \
../Core/Src/stm32f3xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f3xx.c \
../Core/Src/ultrasonico.c 

OBJS += \
./Core/Src/Servo.o \
./Core/Src/StepMotor.o \
./Core/Src/i2c-lcd.o \
./Core/Src/main.o \
./Core/Src/max_matrix_stm32.o \
./Core/Src/stm32f3xx_hal_msp.o \
./Core/Src/stm32f3xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f3xx.o \
./Core/Src/ultrasonico.o 

C_DEPS += \
./Core/Src/Servo.d \
./Core/Src/StepMotor.d \
./Core/Src/i2c-lcd.d \
./Core/Src/main.d \
./Core/Src/max_matrix_stm32.d \
./Core/Src/stm32f3xx_hal_msp.d \
./Core/Src/stm32f3xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f3xx.d \
./Core/Src/ultrasonico.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F302x8 -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/Servo.cyclo ./Core/Src/Servo.d ./Core/Src/Servo.o ./Core/Src/Servo.su ./Core/Src/StepMotor.cyclo ./Core/Src/StepMotor.d ./Core/Src/StepMotor.o ./Core/Src/StepMotor.su ./Core/Src/i2c-lcd.cyclo ./Core/Src/i2c-lcd.d ./Core/Src/i2c-lcd.o ./Core/Src/i2c-lcd.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/max_matrix_stm32.cyclo ./Core/Src/max_matrix_stm32.d ./Core/Src/max_matrix_stm32.o ./Core/Src/max_matrix_stm32.su ./Core/Src/stm32f3xx_hal_msp.cyclo ./Core/Src/stm32f3xx_hal_msp.d ./Core/Src/stm32f3xx_hal_msp.o ./Core/Src/stm32f3xx_hal_msp.su ./Core/Src/stm32f3xx_it.cyclo ./Core/Src/stm32f3xx_it.d ./Core/Src/stm32f3xx_it.o ./Core/Src/stm32f3xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f3xx.cyclo ./Core/Src/system_stm32f3xx.d ./Core/Src/system_stm32f3xx.o ./Core/Src/system_stm32f3xx.su ./Core/Src/ultrasonico.cyclo ./Core/Src/ultrasonico.d ./Core/Src/ultrasonico.o ./Core/Src/ultrasonico.su

.PHONY: clean-Core-2f-Src

