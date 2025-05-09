cmake_minimum_required(VERSION 3.15.3)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/arm-none-eabi-gcc.cmake)

project(binary)

enable_language(C CXX ASM)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD              20)
set(CMAKE_CXX_STANDARD_REQUIRED     ON)
set(CMAKE_CXX_EXTENSIONS            ON)

#cross compiling
set(CMAKE_SYSTEM_NAME Generic)  # muss vor project auf windows (bug)
set(CMAKE_SYSTEM_PROCESSOR arm)
SET(CMAKE_SYSTEM_VERSION 1)

#avoid compiler test failure
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

message("Build type: "              ${CMAKE_BUILD_TYPE})

#include_directories(${CMAKE_SOURCE_DIR}/libs/freechart-1.6/include)
# file(GLOB_RECURSE HALDRIVER_FILES RELATIVE ${CMAKE_SOURCE_DIR} "Drivers/STM32L4xx_HAL_Driver/Src/*.c")

set(HALDRIVER_FILES
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dfsdm.c 
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c 
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.c 
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.c 
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.c 
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.c 
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.c 
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.c 
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.c 
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.c 
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.c 
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.c 
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.c 
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.c 
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.c 
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.c 
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_qspi.c 
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi.c 
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi_ex.c 
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.c 
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.c 
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.c 
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart_ex.c 
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pcd.c 
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pcd_ex.c 
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_usb.c 
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_iwdg.c 

        # Rangesensor Lidar VL5310X
        Drivers/BSP/Components/vl53l0x/vl53l0x_api_calibration.c
        Drivers/BSP/Components/vl53l0x/vl53l0x_api_core.c
        Drivers/BSP/Components/vl53l0x/vl53l0x_api_ranging.c
        Drivers/BSP/Components/vl53l0x/vl53l0x_api_strings.c
        Drivers/BSP/Components/vl53l0x/vl53l0x_api.c
        Drivers/BSP/Components/vl53l0x/vl53l0x_platform_log.c
        Drivers/BSP/Components/vl53l0x/vl53l0x_tof.c

        #Wifi
        Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01.c
        Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.c
        Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_psensor.c
        Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_gyro.c
        Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_hsensor.c
        Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.c
        Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_magneto.c
        Drivers/BSP/Components/hts221/hts221.c
        Drivers/BSP/Components/lsm6dsl/lsm6dsl.c

        Common/Src/es_wifi.c
        Common/Src/es_wifi_io.c
        Common/Src/wifi.c

        )

set(SOURCE_FILES
        Core/Cpp/SomethingInCPP.cpp
        Core/Cpp/Interface.cpp

        Core/Src/main.c 
        Core/Src/stm32l4xx_it.c 
        #Core/Src/stm32l4xx_hal_msp.c 
        Core/Src/system_stm32l4xx.c 
        Core/Src/syscalls.c
        Core/Src/sysmem.c
        startup_stm32l475xx.s
        )

set(EXECUTABLE ${PROJECT_NAME}.elf)

add_executable(${EXECUTABLE} ${HALDRIVER_FILES} ${SOURCE_FILES})

#Global defines for the project
target_compile_definitions(${EXECUTABLE} PRIVATE
        -DUSE_HAL_DRIVER
        -DSTM32L475xx
        )

#All warnings, other than those generated by #warning (see Diagnostics), are suppressed while GCC is processing a system header.
target_include_directories(${EXECUTABLE} SYSTEM PRIVATE
        Drivers/BSP/B-L475E-IOT01
        Common/Inc
        Drivers/STM32L4xx_HAL_Driver/Inc
        Drivers/STM32L4xx_HAL_Driver/Inc/Legacy
        Drivers/CMSIS/Device/ST/STM32L4xx/Include
        Drivers/CMSIS/Include
        #Drivers/Sensors
        )

target_include_directories(${EXECUTABLE} PRIVATE
        Core/Inc
        )

#unterdrueckt warnungen fuer die der studi nix kann, kommt von stmcube und ist nicht schlimm
set_source_files_properties(
        PROPERTIES
        COMPILE_FLAGS -Wno-unused-parameter)

# Core MCU flags, CPU, instruction set and FPU setup
# Needs to be set properly for your MCU
set(CPU_PARAMETERS
        -mthumb
        # This needs attention to properly set for used MCU
        -mcpu=cortex-m4
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard
        )

target_compile_options(${EXECUTABLE} PRIVATE
     
        ${CPU_PARAMETERS}
        -fdata-sections
        -ffunction-sections

        -Wall
        -Wextra
        -Wl,--gc-sections
        -Wno-unused-parameter
        -pedantic

        # this is clearly a C++ specific option
        $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
        # highly recommended option for embedded C++
        $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
        $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>

        $<$<COMPILE_LANGUAGE:CXX>:-Wold-style-cast>
        $<$<COMPILE_LANGUAGE:CXX>:-Wuseless-cast>
        $<$<COMPILE_LANGUAGE:CXX>:-Wsuggest-override>

        # Full debug configuration
        $<$<CONFIG:Debug>:-Og>
        $<$<CONFIG:Debug>:-g>
        $<$<CONFIG:Debug>:-ggdb>
        $<$<CONFIG:Debug>:-gdwarf-2>

        # Generate dependency information
        # https://programmer.group/gcc-m-mm-mmd-mf-mt.html
        # -MMD 
        # -MP 
        # -MF"$(@:%.o=%.d)" // noch nicht cmake kompatibel
        )

#swv/swo debugging: https://www.codeinsideout.com/blog/stm32/swv/#st-linkv2-schematic

target_link_options(${EXECUTABLE} PRIVATE
        -T${CMAKE_SOURCE_DIR}/stm32l475vgtx_flash_manual.ld
        ${CPU_PARAMETERS}
        #GNU ARM libraries use newlib to provide standard implementation of C libraries. To reduce the code size and make it independent to hardware, there is a lightweight version newlib-nano used in MCUs.
        #However, newlib-nano does not provide an implementation of low-level system calls which are used by C standard libraries, such as print() or scan(). To make the application compilable, a new library named nosys should be added. This library just provide a simple implementation of low-level system calls which mostly return a by-pass value.
        -specs=nano.specs
       # -lnosys # muss zuerst kommen
        #die alternative waere https://www.codeinsideout.com/blog/stm32/semihosting/#linker-options

        -Wl,--start-group
        -lc
        -lstdc++
        -lm
         #-u _printf_float braucht dann Core/Src/syscalls.c
        -u _printf_float
        -Wl,--end-group

        # https://www.programmerall.com/article/98742539273/
        # todo: final fix
        -Wl,--no-warn-rwx-segments # avoid ld.exe: warning: xxxx.elf has a LOAD segment with RWX permissions warning for students

        -Wl,-z,max-page-size=8 # Allow good software remapping across address space (with proper GCC section making)
        -Wl,-Map=${PROJECT_NAME}.map,--cref
        -Wl,--gc-sections
        -Wl,--print-memory-usage
        )
# Improve clean target
set_target_properties(${EXECUTABLE} PROPERTIES ADDITIONAL_CLEAN_FILES
        "${PROJECT_NAME}.bin;${PROJECT_NAME}.hex;${PROJECT_NAME}.map;${PROJECT_NAME}.out")

# Print executable size
add_custom_command(TARGET ${EXECUTABLE}
        POST_BUILD
        COMMAND arm-none-eabi-size ${EXECUTABLE})

# Create hex file
add_custom_command(TARGET ${EXECUTABLE}
        POST_BUILD
        COMMAND arm-none-eabi-objcopy -O ihex ${EXECUTABLE} ${PROJECT_NAME}.hex
        COMMAND arm-none-eabi-objcopy -O binary ${EXECUTABLE} ${PROJECT_NAME}.bin)
