################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Stanislav\ NOVAK/Downloads/swap/MEZ-ACIM-FOC/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/G4xx/Src/r3_2_g4xx_pwm_curr_fdbk.c 

OBJS += \
./MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/G4xx/Src/r3_2_g4xx_pwm_curr_fdbk.o 

C_DEPS += \
./MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/G4xx/Src/r3_2_g4xx_pwm_curr_fdbk.d 


# Each subdirectory must supply rules for building sources it contributes
MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/G4xx/Src/r3_2_g4xx_pwm_curr_fdbk.o: C:/Users/Stanislav\ NOVAK/Downloads/swap/MEZ-ACIM-FOC/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/G4xx/Src/r3_2_g4xx_pwm_curr_fdbk.c MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/G4xx/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLibACIM/Any/Inc -I../../MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-MCSDK_v6-2e-4-2e-1-2d-Full-2f-MotorControl-2f-MCSDK-2f-MCLib-2f-G4xx-2f-Src

clean-MCSDK_v6-2e-4-2e-1-2d-Full-2f-MotorControl-2f-MCSDK-2f-MCLib-2f-G4xx-2f-Src:
	-$(RM) ./MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/G4xx/Src/r3_2_g4xx_pwm_curr_fdbk.cyclo ./MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/G4xx/Src/r3_2_g4xx_pwm_curr_fdbk.d ./MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/G4xx/Src/r3_2_g4xx_pwm_curr_fdbk.o ./MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/G4xx/Src/r3_2_g4xx_pwm_curr_fdbk.su

.PHONY: clean-MCSDK_v6-2e-4-2e-1-2d-Full-2f-MotorControl-2f-MCSDK-2f-MCLib-2f-G4xx-2f-Src

