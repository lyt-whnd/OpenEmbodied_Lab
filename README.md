# OpenEmbodied_Lab

OpenEmbodied_Lab is an open-source laboratory for robotics, embedded systems, and embodied AI.

This repository documents the development of ROS 2, Embedded Linux, STM32, RK3568, PCB design, computer vision, and embodied robotic systems.

---

## Current Project

### ROS 2 ESP32-CAM Active Vision Gimbal

This branch develops a Wi-Fi-based active vision gimbal using:

- ESP32-CAM with OV3660 camera
- ROS 2 Jazzy
- OpenCV target detection
- WebSocket control
- STM32 servo control
- Two-axis servo gimbal

Current architecture:

```text
ESP32-CAM / OV3660
        │
        │ HTTP MJPEG stream
        ▼
esp32_camera_node
        │
        │ /image_raw
        ▼
OpenCV Target Detection
        │
        │ /target_position
        ▼
PD Controller
        │
        │ WebSocket V1 binary message
        ▼
ESP32-CAM
        │
        │ UART
        ▼
STM32
        │
        │ PWM
        ▼
Two-axis Servo Gimbal
```

The previous USB-camera and direct-serial implementation is preserved separately.

---

## ESP32-CAM Network Interfaces

The ESP32-CAM firmware currently provides the following interfaces:

```text
http://<ESP32_IP>/
```

Camera test page.

```text
http://<ESP32_IP>/capture
```

Capture one JPEG image.

```text
http://<ESP32_IP>:81/stream
```

Continuous MJPEG video stream.

The control interface is designed as:

```text
ws://<ESP32_IP>/ws
```

Linux sends one complete V1 application message in each binary WebSocket
frame. Video remains on the independent HTTP MJPEG connection.

### V1 application message

The transport-independent header is exactly 10 bytes:

| Offset | Size | Field | Encoding |
|---:|---:|---|---|
| 0 | 1 | `version` | V1 is `0x01` |
| 1 | 1 | `flags` | ACK, response, error, real-time |
| 2 | 1 | `src` | Source node |
| 3 | 1 | `dst` | Destination node |
| 4 | 1 | `service` | System, motion, telemetry, config, event, OTA |
| 5 | 1 | `opcode` | Operation within the service |
| 6 | 2 | `seq` | Unsigned 16-bit, little-endian |
| 8 | 2 | `payload_len` | Unsigned 16-bit, little-endian |
| 10 | N | `payload` | At most 256 bytes |

Node identifiers are:

```text
Linux     0x01
ESP32     0x02
STM32     0x03
Broadcast 0xFF
```

The Linux implementation is:

```text
src/vision_demo/vision_demo/protocol_v1.py
```

The matching ESP32 implementation is:

```text
ESP_control/esp32cam_gimbal/protocol_v1.h
ESP_control/esp32cam_gimbal/protocol_v1.cpp
```

`MOTION/MOVE` currently uses a fixed 12-byte little-endian payload:

| Field | Type | Unit |
|---|---|---|
| `control_epoch` | `uint16` | Control ownership generation |
| `valid_ms` | `uint16` | Command lifetime |
| `linear_mm_s` | `int16` | Tracked-base linear velocity |
| `angular_mrad_s` | `int16` | Tracked-base angular velocity |
| `head_yaw_rate_x10` | `int16` | 0.1 degree/second |
| `head_pitch_rate_x10` | `int16` | 0.1 degree/second |

The current PD node sets both tracked-base velocity fields to zero and fills
the two head-rate fields. `MOVE` uses the real-time flag and only the newest
pending command is retained. `STOP` uses the ACK-required flag.
This stage implements the Linux transmit path; ACK reception, timeout, and
retry handling will be added with the bidirectional WebSocket link manager.

The ESP32 validates the V1 version, known flags, payload limit, exact frame
length, source node, and destination node before routing a message. Messages
addressed to STM32 are forwarded without changing their application header.
Messages addressed to ESP32 currently support:

```text
SYSTEM/PING  opcode 0x01
SYSTEM/PONG  opcode 0x02
```

`PONG` keeps the request sequence number and sets the response flag.

### ESP32-to-STM32 UART framing

WebSocket already preserves message boundaries, while UART is a byte stream.
The ESP32 therefore sends the same V1 application message using:

```text
COBS(application_message + CRC16-CCITT-FALSE) + 0x00
```

CRC parameters:

```text
polynomial: 0x1021
initial:    0xFFFF
refin:      false
refout:     false
xorout:     0x0000
```

The CRC is appended little-endian and covers the complete V1 application
message. The ESP32 UART receiver performs delimiter recovery, COBS decoding,
CRC checking, V1 validation, and source/destination checking. Valid STM32
messages addressed to Linux are returned as binary WebSocket frames. Only
one active Linux WebSocket client is retained by the current firmware.

STM32 must implement the same COBS/CRC framing before the new binary control
path can operate end to end; the former newline text UART format is no longer
used by this ESP32 firmware.

---

## Features

### Implemented

- ESP32-CAM OV3660 initialization
- PSRAM detection
- JPEG image acquisition
- HTTP single-image capture
- MJPEG video streaming
- ROS 2 ESP32-CAM stream receiver
- ROS 2 image topic publication
- OpenCV target detection
- Target center offset calculation
- PD gimbal controller
- Linux V1 application protocol codec
- Binary WebSocket motion sender
- ESP32 V1 binary WebSocket decoder
- ESP32 V1 destination router
- ESP32-to-STM32 COBS and CRC16 framing
- STM32-to-Linux validated binary forwarding
- PWM servo control

### In Progress

- STM32 V1 application protocol decoder
- STM32 COBS and CRC16 UART receiver
- Linux ACK reception, timeout, and retry handling
- YOLO target detection
- RK3568 deployment
- Low-latency frame processing
- Mobile robot integration

---

## Development Environment

### Linux / ROS 2

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3
- OpenCV
- cv_bridge
- websocket-client
- colcon

### ESP32

- Arduino IDE 2.x
- ESP32 Arduino Core by Espressif Systems
- AI Thinker ESP32-CAM
- OV3660 camera sensor
- ESP32-CAM-MB download board
- 2.4 GHz Wi-Fi

### STM32

- STM32F103C8T6
- STM32CubeMX
- STM32 HAL
- UART
- TIM PWM

---

## Repository Structure

```text
OpenEmbodied_Lab/
├── src/
│   └── vision_demo/
│       ├── config/
│       │   └── esp32_wifi.yaml
│       ├── launch/
│       │   └── esp32_wifi.launch.py
│       ├── resource/
│       │   └── vision_demo
│       ├── vision_demo/
│       │   ├── __init__.py
│       │   ├── protocol_v1.py
│       │   ├── esp32_camera_node.py
│       │   ├── color_tracker_node.py
│       │   ├── gimbal_pd_websocket_node.py
│       │   ├── gimbal_pd_serial_node.py
│       │   └── image_viewer_node.py
│       ├── test/
│       │   └── test_protocol_v1.py
│       ├── package.xml
│       └── setup.py
├── ESP_control/
│   ├── esp32cam_gimbal/
│   │   ├── protocol_v1.h
│   │   ├── protocol_v1.cpp
│   │   ├── uart_framing.h
│   │   ├── uart_framing.cpp
│   │   ├── websocket_service.cpp
│   │   └── stm32_uart.cpp
│   └── tests/
│       └── protocol_v1_host_test.cpp
├── README.md
└── .gitignore
```

Generated ROS 2 directories are ignored by Git:

```text
build/
install/
log/
```

---

## Clone the ESP32 Wi-Fi Branch

```bash
git clone https://github.com/lyt-whnd/OpenEmbodied_Lab.git

cd OpenEmbodied_Lab

git switch feature/esp32-wifi
```

Replace `feature/esp32-wifi` if the actual branch uses a different name.

Check the current branch:

```bash
git branch --show-current
```

---

## ROS 2 Installation

Add the ROS 2 environment to the current terminal:

```bash
source /opt/ros/jazzy/setup.bash
```

Install the required ROS 2 and Python packages:

```bash
sudo apt update

sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-cv-bridge \
  python3-opencv \
  python3-numpy \
  python3-websocket \
  python3-serial \
  python3-colcon-common-extensions \
  git
```

Install dependencies declared by ROS 2 packages:

```bash
cd OpenEmbodied_Lab

rosdep update

rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro jazzy \
  -r \
  -y
```

Verify the main Python dependencies:

```bash
python3 -c "import cv2; print('OpenCV:', cv2.__version__)"

python3 -c "import websocket; print('WebSocket:', websocket.__version__)"

python3 -c "import serial; print('PySerial:', serial.__version__)"
```

---

## USB-to-Serial Driver

The ESP32-CAM-MB download board normally uses a USB-to-UART chip such as CH340 or CP210x.

### Ubuntu

The CH340 and CP210x drivers are normally included in the Linux kernel.

Connect the download board and check the USB device:

```bash
lsusb
```

Check the serial device:

```bash
ls /dev/ttyUSB*
```

Check the kernel message:

```bash
sudo dmesg | tail -n 30
```

Check whether the driver is loaded:

```bash
lsmod | grep -E "ch341|cp210x"
```

The serial port normally appears as:

```text
/dev/ttyUSB0
```

Add the current user to the serial-port group:

```bash
sudo usermod -aG dialout "$USER"
```

Log out and log back in after running this command.

Check permissions:

```bash
groups
```

The output should contain:

```text
dialout
```

### Windows

The ESP32-CAM-MB normally appears as a COM port:

```text
COM3
COM9
...
```

If no COM port appears, install the driver corresponding to the USB-to-UART chip on the download board:

```text
CH340 / CH341
or
CP210x
```

---

## ESP32 Arduino Configuration

In Arduino IDE, install:

```text
esp32 by Espressif Systems
```

Recommended board settings:

```text
Board:
AI Thinker ESP32-CAM

Upload Speed:
115200

CPU Frequency:
240 MHz

Flash Frequency:
80 MHz

Flash Mode:
QIO

Partition Scheme:
Huge APP (3MB No OTA / 1MB SPIFFS)
```

The serial monitor baud rate must match:

```cpp
Serial.begin(115200);
```

Therefore, select:

```text
115200 baud
```

The ESP32-CAM requires a 2.4 GHz Wi-Fi network.

### ESP32 protocol host test

The transport-independent V1 codec, COBS, and CRC16 code can be checked on
Linux without Arduino hardware:

```bash
g++ \
  -std=c++17 \
  -Wall \
  -Wextra \
  -Werror \
  -I ESP_control/esp32cam_gimbal \
  ESP_control/tests/protocol_v1_host_test.cpp \
  ESP_control/esp32cam_gimbal/protocol_v1.cpp \
  ESP_control/esp32cam_gimbal/uart_framing.cpp \
  -o /tmp/protocol_v1_host_test

/tmp/protocol_v1_host_test
```

The test covers the shared Linux/ESP32 fixed byte vector, PONG encoding,
CRC16 reference vector, COBS round trips, corrupted frames, and the maximum
V1 payload size.

---

## ROS 2 Configuration

Edit:

```text
src/vision_demo/config/esp32_wifi.yaml
```

Set the actual ESP32-CAM address:

```yaml
esp32_camera_node:
  ros__parameters:
    stream_url: "http://192.168.1.100:81/stream"
    image_topic: "/image_raw"
    frame_id: "esp32_camera"
    reconnect_delay_sec: 2.0
    opencv_buffer_size: 1

color_tracker_node:
  ros__parameters:
    image_topic: "/image_raw"

gimbal_pd_websocket_node:
  ros__parameters:
    target_topic: "/target_position"
    websocket_url: "ws://192.168.1.100/ws"

    control_hz: 10.0

    kp_x: 0.015
    kd_x: 0.006

    kp_y: 0.015
    kd_y: 0.006

    dead_zone_x: 30.0
    dead_zone_y: 30.0

    max_step: 3

    target_timeout_sec: 0.5
    command_refresh_sec: 0.5
    reconnect_delay_sec: 2.0

    motion_valid_ms: 300
    control_epoch: 1
```

Replace:

```text
192.168.1.100
```

with the IP address printed by the ESP32-CAM serial monitor.

The Linux computer and ESP32-CAM must be connected to the same local network.

---

## Build

Enter the ROS 2 workspace:

```bash
cd OpenEmbodied_Lab
```

Load ROS 2 Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
```

Build the complete workspace:

```bash
colcon build --symlink-install
```

Load the workspace environment:

```bash
source install/setup.bash
```

To build only the vision package:

```bash
colcon build \
  --symlink-install \
  --packages-select vision_demo
```

When package files, entry points, or launch files have changed, clean the package before rebuilding:

```bash
rm -rf \
  build/vision_demo \
  install/vision_demo

colcon build \
  --symlink-install \
  --packages-select vision_demo

source install/setup.bash
```

Check the installed executables:

```bash
ros2 pkg executables vision_demo
```

Expected executables include:

```text
vision_demo esp32_camera
vision_demo color_tracker
vision_demo image_viewer
vision_demo gimbal_pd_serial
vision_demo gimbal_pd_websocket
```

---

## Test the ESP32-CAM Connection

Before starting ROS 2, test the ESP32-CAM stream in a browser:

```text
http://<ESP32_IP>:81/stream
```

The browser should display a continuous video stream.

Close the browser stream before starting the ROS 2 camera node to avoid unnecessary simultaneous clients.

Check basic network connectivity:

```bash
ping <ESP32_IP>
```

Example:

```bash
ping 192.168.1.100
```

---

## Run the Camera Node

Load the environment:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Start the ESP32-CAM receiver:

```bash
ros2 run vision_demo esp32_camera \
  --ros-args \
  -p stream_url:="http://192.168.1.100:81/stream"
```

Check the image topic:

```bash
ros2 topic list
```

The topic should include:

```text
/image_raw
```

Check the received frame rate:

```bash
ros2 topic hz /image_raw
```

Check image information:

```bash
ros2 topic info /image_raw
```

---

## Display the ROS 2 Image

In another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run vision_demo image_viewer
```

The image viewer should display the ESP32-CAM video received through ROS 2.

---

## Run Target Detection

Start the target detection node:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run vision_demo color_tracker
```

Check the target-position output:

```bash
ros2 topic echo /target_position
```

The target message contains:

```text
x: horizontal image error
y: vertical image error
z: detected target area
```

---

## Run the WebSocket Controller

The ESP32 firmware must provide:

```text
ws://<ESP32_IP>/ws
```

Start the controller:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run vision_demo gimbal_pd_websocket \
  --ros-args \
  -p websocket_url:="ws://192.168.1.100/ws"
```

The node sends binary messages with the V1 header:

```text
Linux -> STM32
service = MOTION (0x10)
opcode  = MOVE (0x01) or STOP (0x02)
seq     = wrapping uint16
```

The `websocket-client` call uses a binary WebSocket frame; it does not send
the former `#MOVE` strings. The ESP32 decodes the 10-byte application header
and routes STM32 messages through its COBS/CRC16 UART transport. The remaining
end-to-end dependency is the matching STM32 V1 UART decoder. The image
receiver remains independent.

---

## Run the Complete ESP32 Wi-Fi System

After editing `esp32_wifi.yaml`, start all nodes with:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch vision_demo esp32_wifi.launch.py
```

The launch file starts:

```text
esp32_camera_node
color_tracker_node
gimbal_pd_websocket_node
```

Check the ROS 2 graph:

```bash
ros2 node list
ros2 topic list
```

Expected nodes:

```text
/esp32_camera_node
/color_tracker_node
/gimbal_pd_websocket_node
```

Expected topics include:

```text
/image_raw
/target_position
```

---

## Legacy USB Camera Version

The previous implementation used:

```text
USB camera
→ ROS 2
→ OpenCV
→ PD controller
→ Linux serial port
→ STM32
```

The optional USB camera driver can be installed with:

```bash
sudo apt install -y ros-jazzy-usb-cam
```

Start the USB camera:

```bash
ros2 run usb_cam usb_cam_node_exe
```

Start the original serial controller:

```bash
ros2 run vision_demo gimbal_pd_serial
```

The ESP32 Wi-Fi branch does not require `usb_cam` for normal operation.

---

## Troubleshooting

### ESP32 stream cannot be opened

Check that the stream works in a browser:

```text
http://<ESP32_IP>:81/stream
```

Check the network:

```bash
ping <ESP32_IP>
```

Make sure:

```text
ESP32-CAM and Linux are on the same LAN
ESP32-CAM is connected to 2.4 GHz Wi-Fi
The browser is not occupying the only available stream connection
The ESP32 IP address is correct
```

### `/image_raw` has no data

Check the camera node:

```bash
ros2 node info /esp32_camera_node
```

Check the topic:

```bash
ros2 topic hz /image_raw
```

Run the node directly and inspect its log:

```bash
ros2 run vision_demo esp32_camera \
  --ros-args \
  -p stream_url:="http://<ESP32_IP>:81/stream"
```

### WebSocket connection fails

Verify that the ESP32 firmware has implemented:

```text
ws://<ESP32_IP>/ws
```

A working HTTP MJPEG stream does not automatically provide a WebSocket server.

### ROS 2 executable not found

Rebuild and reload the environment:

```bash
cd OpenEmbodied_Lab

rm -rf \
  build/vision_demo \
  install/vision_demo

source /opt/ros/jazzy/setup.bash

colcon build \
  --symlink-install \
  --packages-select vision_demo

source install/setup.bash
```

### Serial permission denied

Add the user to `dialout`:

```bash
sudo usermod -aG dialout "$USER"
```

Then log out and log back in.

---

## Roadmap

### Completed

- [x] ROS 2 workspace
- [x] USB camera prototype
- [x] OpenCV target detection
- [x] Target center offset calculation
- [x] PD tracking controller
- [x] STM32 serial protocol
- [x] STM32 PWM servo control
- [x] ESP32-CAM OV3660 initialization
- [x] HTTP JPEG capture
- [x] MJPEG video streaming
- [x] ROS 2 ESP32-CAM image receiver
- [x] ROS 2 image topic publication
- [x] Linux V1 application protocol codec
- [x] Linux binary WebSocket motion sender
- [x] V1 protocol fixed-vector unit tests
- [x] ESP32 V1 binary WebSocket decoder
- [x] ESP32 V1 message router
- [x] ESP32-to-STM32 COBS and CRC16 framing
- [x] STM32-to-Linux validated V1 forwarding

### In Progress

- [x] ESP32 legacy WebSocket server
- [x] ESP32 legacy UART command forwarding
- [ ] STM32 V1 application decoder
- [ ] STM32 COBS and CRC16 UART receiver
- [ ] Linux ACK receive, timeout, and retry
- [ ] Complete Wi-Fi gimbal control loop
- [ ] YOLO detection node
- [ ] Latest-frame-only inference pipeline
- [ ] RK3568 deployment
- [ ] ROS 2 launch and parameter optimization
- [ ] IMU integration
- [ ] Millimeter-wave radar
- [ ] Mobile robot platform
- [ ] ROS 2 Nav2
- [ ] Embedded AI applications

---

## Future Plans

This repository will continue to grow with projects related to:

- Embedded Linux
- STM32
- ESP32
- RK3568
- ROS 2
- PCB design
- Robotics
- Computer vision
- Edge AI
- Embodied AI
