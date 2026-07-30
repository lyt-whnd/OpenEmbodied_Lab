cmake_minimum_required(VERSION 3.20)

project(robot_stm32_firmware C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXECUTABLE_SUFFIX ".elf")

set(MCU_FLAGS
    -mcpu=cortex-m3
    -mthumb
)

set(CORE_SOURCES
    Core/Src/main.c
    Core/Src/robot_dispatcher.c
    Core/Src/robot_motion.c
    Core/Src/robot_protocol.c
    Core/Src/robot_reliable.c
    Core/Src/robot_result_cache.c
    Core/Src/robot_sample_block.c
    Core/Src/robot_sensor_catalog.c
    Core/Src/robot_sensor_registry.c
    Core/Src/robot_sensor_service.c
    Core/Src/robot_service_registry.c
    Core/Src/robot_telemetry.c
    Core/Src/robot_time.c
    Core/Src/robot_tx_scheduler.c
    Core/Src/robot_transport_uart.c
    Core/Src/stm32f1xx_hal_msp.c
    Core/Src/stm32f1xx_it.c
    Core/Src/system_stm32f1xx.c
    Core/Src/uart_ring_buffer.c
)

set(HAL_SOURCES
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c
)

add_executable(robot_stm32
    ${CORE_SOURCES}
    ${HAL_SOURCES}
    Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103xb.s
)

target_include_directories(robot_stm32 PRIVATE
    Core/Inc
    Drivers/STM32F1xx_HAL_Driver/Inc
    Drivers/STM32F1xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F1xx/Include
    Drivers/CMSIS/Include
)

target_compile_definitions(robot_stm32 PRIVATE
    USE_HAL_DRIVER
    STM32F103xB
)

target_compile_options(robot_stm32 PRIVATE
    ${MCU_FLAGS}
    -ffunction-sections
    -fdata-sections
    -fno-common
    -Wall
    -Wextra
    -Werror
    $<$<CONFIG:Debug>:-Og>
    $<$<CONFIG:Debug>:-g3>
    $<$<NOT:$<CONFIG:Debug>>:-Os>
)

target_link_options(robot_stm32 PRIVATE
    ${MCU_FLAGS}
    -T${CMAKE_CURRENT_SOURCE_DIR}/STM32F103C8Tx_FLASH.ld
    --specs=nano.specs
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/robot_stm32.map
    -Wl,--print-memory-usage
)

add_custom_command(TARGET robot_stm32 POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:robot_stm32>
            ${CMAKE_CURRENT_BINARY_DIR}/robot_stm32.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:robot_stm32>
            ${CMAKE_CURRENT_BINARY_DIR}/robot_stm32.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:robot_stm32>
    COMMENT "Generating HEX/BIN images and memory summary"
)
