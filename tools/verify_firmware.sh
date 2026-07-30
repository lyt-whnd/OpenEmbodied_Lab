#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
host_build_dir="$(mktemp -d)"
trap 'rm -rf "${host_build_dir}"' EXIT

pio_command="${PIO_COMMAND:-${HOME}/.local/bin/pio}"

cd "${repo_root}"

python3 protocol_test_vectors/generate_v1_vectors.py --check

g++ -std=c++17 -Wall -Wextra -Werror \
    -I ESP_control/esp32cam_gimbal \
    -I protocol_test_vectors/generated \
    ESP_control/tests/protocol_v1_host_test.cpp \
    ESP_control/esp32cam_gimbal/protocol_v1.cpp \
    ESP_control/esp32cam_gimbal/uart_framing.cpp \
    -o "${host_build_dir}/esp32_protocol_test"
"${host_build_dir}/esp32_protocol_test"

g++ -std=c++17 -Wall -Wextra -Werror \
    -I ESP_control/esp32cam_gimbal \
    ESP_control/tests/uart_stream_framer_host_test.cpp \
    ESP_control/esp32cam_gimbal/uart_framing.cpp \
    ESP_control/esp32cam_gimbal/uart_stream_framer.cpp \
    -o "${host_build_dir}/uart_stream_framer_test"
"${host_build_dir}/uart_stream_framer_test"

if rg -q \
    'ProtocolV1::|decodeMessage|src|dst|service|opcode' \
    ESP_control/esp32cam_gimbal/stm32_uart.cpp
then
    echo \
        "STM32 UART transport contains V1 or routing logic" \
        >&2
    exit 1
fi

g++ -std=c++17 -Wall -Wextra -Werror \
    -I ESP_control/esp32cam_gimbal \
    ESP_control/tests/message_router_host_test.cpp \
    ESP_control/esp32cam_gimbal/protocol_v1.cpp \
    ESP_control/esp32cam_gimbal/message_router.cpp \
    ESP_control/esp32cam_gimbal/system_service.cpp \
    -o "${host_build_dir}/esp32_router_test"
"${host_build_dir}/esp32_router_test"

g++ -std=c++17 -Wall -Wextra -Werror \
    -pthread \
    -I ESP_control/esp32cam_gimbal \
    ESP_control/tests/network_tx_queue_host_test.cpp \
    ESP_control/esp32cam_gimbal/network_tx_queue.cpp \
    -o "${host_build_dir}/network_tx_queue_test"
"${host_build_dir}/network_tx_queue_test"

gcc -std=c11 -Wall -Wextra -Werror \
    -I stm32_control/ros2/Core/Inc \
    -I protocol_test_vectors/generated \
    stm32_control/tests/robot_protocol_host_test.c \
    stm32_control/ros2/Core/Src/robot_dispatcher.c \
    stm32_control/ros2/Core/Src/robot_protocol.c \
    stm32_control/ros2/Core/Src/robot_motion.c \
    -o "${host_build_dir}/stm32_protocol_test"
"${host_build_dir}/stm32_protocol_test"

gcc -std=c11 -Wall -Wextra -Werror \
    -I stm32_control/ros2/Core/Inc \
    stm32_control/tests/uart_ring_buffer_host_test.c \
    stm32_control/ros2/Core/Src/robot_motion.c \
    stm32_control/ros2/Core/Src/robot_protocol.c \
    stm32_control/ros2/Core/Src/robot_transport_uart.c \
    stm32_control/ros2/Core/Src/uart_ring_buffer.c \
    -o "${host_build_dir}/stm32_transport_test"
"${host_build_dir}/stm32_transport_test"

PYTHONPATH=src/vision_demo:tools/protocol_sim \
    python3 -m pytest -q \
    src/vision_demo/test/test_protocol_v1.py \
    tools/protocol_sim/test_three_node_sim.py

cmake --fresh -S stm32_control/ros2 \
    -B build/stm32-gcc \
    -G Ninja \
    -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake \
    -DCMAKE_BUILD_TYPE=Release
cmake --build build/stm32-gcc

"${pio_command}" run -d ESP_control/esp32cam_gimbal
