#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
host_build_dir="$(mktemp -d)"
trap 'rm -rf "${host_build_dir}"' EXIT

pio_command="${PIO_COMMAND:-${HOME}/.local/bin/pio}"

cd "${repo_root}"

python3 protocol_test_vectors/generate_v1_vectors.py --check
python3 protocol_schema/generate_services.py --check

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

g++ -std=c++17 -Wall -Wextra -Werror \
    -I ESP_control/esp32cam_gimbal \
    ESP_control/tests/tcp_packet_framer_host_test.cpp \
    ESP_control/esp32cam_gimbal/tcp_packet_framer.cpp \
    -o "${host_build_dir}/tcp_packet_framer_test"
"${host_build_dir}/tcp_packet_framer_test"

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
    ESP_control/esp32cam_gimbal/reliable_endpoint.cpp \
    ESP_control/esp32cam_gimbal/result_cache.cpp \
    ESP_control/esp32cam_gimbal/service_registry.cpp \
    ESP_control/esp32cam_gimbal/system_service.cpp \
    -o "${host_build_dir}/esp32_router_test"
"${host_build_dir}/esp32_router_test"

g++ -std=c++17 -Wall -Wextra -Werror \
    -I ESP_control/esp32cam_gimbal \
    ESP_control/tests/reliable_endpoint_host_test.cpp \
    ESP_control/esp32cam_gimbal/protocol_v1.cpp \
    ESP_control/esp32cam_gimbal/reliable_endpoint.cpp \
    ESP_control/esp32cam_gimbal/result_cache.cpp \
    ESP_control/esp32cam_gimbal/service_registry.cpp \
    -o "${host_build_dir}/esp32_reliable_test"
"${host_build_dir}/esp32_reliable_test"

g++ -std=c++17 -Wall -Wextra -Werror \
    -pthread \
    -I ESP_control/esp32cam_gimbal \
    ESP_control/tests/network_tx_queue_host_test.cpp \
    ESP_control/esp32cam_gimbal/protocol_v1.cpp \
    ESP_control/esp32cam_gimbal/network_tx_queue.cpp \
    ESP_control/esp32cam_gimbal/service_registry.cpp \
    -o "${host_build_dir}/network_tx_queue_test"
"${host_build_dir}/network_tx_queue_test"

gcc -std=c11 -Wall -Wextra -Werror \
    -I stm32_control/ros2/Core/Inc \
    -I protocol_test_vectors/generated \
    stm32_control/tests/robot_protocol_host_test.c \
    stm32_control/ros2/Core/Src/robot_dispatcher.c \
    stm32_control/ros2/Core/Src/robot_protocol.c \
    stm32_control/ros2/Core/Src/robot_reliable.c \
    stm32_control/ros2/Core/Src/robot_result_cache.c \
    stm32_control/ros2/Core/Src/robot_sensor_registry.c \
    stm32_control/ros2/Core/Src/robot_sensor_service.c \
    stm32_control/ros2/Core/Src/robot_service_registry.c \
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

gcc -std=gnu11 -Wall -Wextra -Werror \
    -I stm32_control/ros2/Core/Inc \
    stm32_control/tests/robot_tx_scheduler_host_test.c \
    stm32_control/ros2/Core/Src/robot_tx_scheduler.c \
    -o "${host_build_dir}/stm32_tx_scheduler_test"
"${host_build_dir}/stm32_tx_scheduler_test"

gcc -std=c11 -Wall -Wextra -Werror \
    -I stm32_control/ros2/Core/Inc \
    stm32_control/tests/robot_sensor_host_test.c \
    stm32_control/ros2/Core/Src/robot_protocol.c \
    stm32_control/ros2/Core/Src/robot_sensor_registry.c \
    stm32_control/ros2/Core/Src/robot_sensor_service.c \
    -o "${host_build_dir}/stm32_sensor_test"
"${host_build_dir}/stm32_sensor_test"

gcc -std=c11 -Wall -Wextra -Werror \
    -I stm32_control/ros2/Core/Inc \
    stm32_control/tests/robot_time_host_test.c \
    stm32_control/ros2/Core/Src/robot_time.c \
    -o "${host_build_dir}/stm32_time_test"
"${host_build_dir}/stm32_time_test"

gcc -std=c11 -Wall -Wextra -Werror \
    -I stm32_control/ros2/Core/Inc \
    stm32_control/tests/robot_telemetry_host_test.c \
    stm32_control/ros2/Core/Src/robot_telemetry.c \
    stm32_control/ros2/Core/Src/robot_time.c \
    -o "${host_build_dir}/stm32_telemetry_test"
"${host_build_dir}/stm32_telemetry_test"

gcc -std=c11 -Wall -Wextra -Werror \
    -I stm32_control/ros2/Core/Inc \
    stm32_control/tests/robot_sample_block_host_test.c \
    stm32_control/ros2/Core/Src/robot_sample_block.c \
    stm32_control/ros2/Core/Src/robot_time.c \
    -o "${host_build_dir}/stm32_sample_block_test"
"${host_build_dir}/stm32_sample_block_test"

PYTHONPATH=src/vision_demo:tools/protocol_sim \
    python3 -m pytest -q \
    src/vision_demo/test/test_protocol_v1.py \
    src/vision_demo/test/test_reliable_protocol.py \
    src/vision_demo/test/test_robot_link.py \
    src/vision_demo/test/test_sensor_protocol.py \
    src/vision_demo/test/test_clock_mapper.py \
    src/vision_demo/test/test_telemetry.py \
    src/vision_demo/test/test_sample_block.py \
    src/vision_demo/test/test_tx_scheduler.py \
    src/vision_demo/test/test_tcp_transport.py \
    src/vision_demo/test/test_esp32_discovery.py \
    tools/protocol_sim/test_three_node_sim.py

cmake --fresh -S stm32_control/ros2 \
    -B build/stm32-gcc \
    -G Ninja \
    -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake \
    -DCMAKE_BUILD_TYPE=Release
cmake --build build/stm32-gcc

stm32_size="$(
    arm-none-eabi-size \
        build/stm32-gcc/robot_stm32.elf |
    awk 'NR == 2 {print $2 " " $3}'
)"
read -r stm32_data stm32_bss <<<"${stm32_size}"
stm32_static_ram=$((stm32_data + stm32_bss))
stm32_static_ram_budget="$(
    echo "${STM32_STATIC_RAM_BUDGET_BYTES:-8192}"
)"

if ((stm32_static_ram > stm32_static_ram_budget)); then
    echo \
        "STM32 static RAM ${stm32_static_ram} B exceeds " \
        "Stage-2 budget ${stm32_static_ram_budget} B" \
        >&2
    exit 1
fi

echo \
    "STM32 Stage-2 static RAM gate: " \
    "${stm32_static_ram}/${stm32_static_ram_budget} B"

"${pio_command}" run -d ESP_control/esp32cam_gimbal
