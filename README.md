# Vision-Guided Line Follower & Sumo Robot (Dual ESP32 + ESP-NOW)

A high-speed, dual-microcontroller autonomous robotic platform combining real-time computer vision and responsive motor drive control over low-latency wireless ESP-NOW.

---

## 📋 Requirements & Bill of Materials (BOM)

### 1. Hardware Requirements
| Item | Recommended Specification | Purpose |
|---|---|---|
| **Vision Processor** | AI-Thinker ESP32-CAM with OV2640 module | Captures frames and computes track offset |
| **Motor Controller** | ESP32 DevKit V1 / NodeMCU-32S (30 or 38 pin) | Executes PID control and drives the motors |
| **Motor Driver** | Toshiba TB6612FNG Dual H-Bridge Module | High-efficiency MOSFET motor control |
| **Motors** | 2x N20 Micro DC Gearmotors (6V, ~500 RPM) | Left and right differential drive |
| **Power Source** | 2S LiPo (7.4V) or 2x 18650 Li-ion cells | Primary system power |
| **Voltage Regulator**| DC-DC Step-Down Buck Converter (e.g., Mini360 / LM2596) | Regulates battery to clean 5V (2A+ peak) for ESP32-CAM |
| **Programmer** | FTDI USB-to-TTL Serial adapter (with 3.3V/5V selector) | Flashing the ESP32-CAM module |
| **Chassis** | 3D-printed or acrylic sumo / line-follower base | Robot body with caster wheel and camera mount |

### 2. Software & Tooling Requirements
- **Development Environment**: 
  - Arduino IDE (v1.8.19 or v2.x) or VS Code with PlatformIO / Arduino Extension.
- **Board Package**:
  - Official **Espressif ESP32 Core** (v2.0.x or v3.x).
  - Board Manager URL:
    ```
    https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
    ```
- **Dependencies & Libraries**:
  - All libraries are native and included with the ESP32 Arduino core:
    - `<esp_camera.h>`
    - `<WiFi.h>`
    - `<esp_now.h>`
    - `<esp_wifi.h>`
  - *No external third-party library installations required.*

---

## ⚠️ Important Engineering Notes & Prerequisites

> [!IMPORTANT]
> **Power Supply & Brownout Prevention**:
> The ESP32-CAM module draws high surge currents when the Wi-Fi radio transmits and the flash LED fires. Powering it via USB-to-Serial 5V pins often leads to `Brownout detector was triggered` reboot loops. Always supply regulated 5V (minimum 2A capable) directly with a decoupling capacitor (e.g., 470µF to 1000µF across 5V and GND).

> [!NOTE]
> **Common Ground**:
> Ensure a single common ground connection (`GND`) links the battery, buck converter, ESP32-CAM, ESP32 motor controller, and the TB6612FNG driver logic ground.

> [!TIP]
> **Flash Mode on ESP32-CAM**:
> To upload code to the ESP32-CAM, bridge `GPIO 0` to `GND` before powering on or pressing the reset button. Disconnect `GPIO 0` from `GND` after flashing and press reset to start standard execution.

---

## 📌 Architecture Overview

This project divides robot autonomy across two dedicated ESP32 modules connected wirelessly via peer-to-peer **ESP-NOW**:

1. **Vision Node (`firmware/ESP32_CAM_V10/ESP32_CAM_V10.ino`)**
   - Captures 160x120 grayscale frames (`FRAMESIZE_QQVGA`).
   - Runs dynamic luma auto-calibration to adapt to ambient lighting changes.
   - Evaluates a Region of Interest (ROI: Y=60 to 120).
   - Applies morphological pseudo-opening to filter floor speckle and noise.
   - Calculates line center of mass error (`cx - FRAME_CENTER`).
   - Detects special track events:
     - `State 0`: Normal line following.
     - `State 1`: Track lost / broken line gap.
     - `State 2`: T-junction or cross intersection.
   - Transmits telemetry packets instantly to the motor controller.

2. **Motor Driver Node (`firmware/ESP32_MotorDriver_V10/ESP32_MotorDriver_V10.ino`)**
   - Receives ESP-NOW command packets with near-zero latency.
   - Executes a closed-loop PID control algorithm (`Kp`, `Ki`, `Kd`).
   - Translates steering adjustments into differential PWM for the TB6612FNG.
   - Implements high-speed reverse-braking when cornering sharply.
   - **Signal Failsafe**: Automatically stops both motors if no packet is received for >500 ms.

```
+--------------------+                     +-----------------------+
|  AI-Thinker CAM    |                     |     ESP32 Node        |
|  - Grayscale QQVGA |   ESP-NOW Packet    |  - PID Control Loop   |
|  - Adaptive Luma   | ------------------> |  - TB6612FNG Driver   |
|  - Centroid Error  |   (Zero Router)     |  - Dual N20 Motors    |
|  - State Detection |                     |  - Failsafe Stop      |
+--------------------+                     +-----------------------+
```

---

## 🗂️ Repository Structure

```
├── firmware/
│   ├── ESP32_CAM_V10/
│   │   └── ESP32_CAM_V10.ino          # Vision sensing & ESP-NOW sender
│   └── ESP32_MotorDriver_V10/
│       └── ESP32_MotorDriver_V10.ino  # ESP-NOW receiver, PID & motor driver
├── tools/
│   └── Get_MAC_Address/
│       └── Get_MAC_Address.ino        # Utility sketch to read receiver MAC address
├── .gitignore
└── README.md
```

---

## 🔌 Hardware Wiring & Pin Configuration

### 1. ESP32 Motor Controller to TB6612FNG

| TB6612FNG Pin | ESP32 GPIO Pin | Function |
|---|---|---|
| **PWMA** | `GPIO 26` | Right Motor Speed (PWM) |
| **AIN1** | `GPIO 27` | Right Motor Direction 1 |
| **AIN2** | `GPIO 14` | Right Motor Direction 2 |
| **PWMB** | `GPIO 12` | Left Motor Speed (PWM) |
| **BIN1** | `GPIO 13` | Left Motor Direction 1 |
| **BIN2** | `GPIO 15` | Left Motor Direction 2 |
| **STBY** | `GPIO 4`  | Standby (High = Enabled) |
| **VM**   | Battery (+) | Motor Power (6V - 12V) |
| **VCC**  | `3V3` or `5V` | Logic Voltage |
| **GND**  | `GND` | Common Ground |

### 2. AI-Thinker ESP32-CAM Pin Allocation
- Sensor: Standard AI-Thinker pinout (pre-configured in sketch).
- Flash LED: `GPIO 4` (illuminates the track).
- Frame configuration: Vertical flip enabled for upright mounting.

---

## ⚙️ Board Flashing Settings

### For Vision Node (`ESP32_CAM_V10`):
- **Board**: `AI Thinker ESP32-CAM` (or `ESP32 Wrover Module`)
- **CPU Frequency**: `240MHz`
- **Flash Frequency**: `80MHz`
- **Partition Scheme**: `Huge APP (3MB No OTA/1MB SPIFFS)` *(Mandatory: sketch requires >1.3MB flash)*
- **PSRAM**: `Enabled`
- **Upload Speed**: `115200` or `921600`

### For Motor Driver Node (`ESP32_MotorDriver_V10`):
- **Board**: `ESP32 Dev Module`
- **CPU Frequency**: `240MHz`
- **Flash Frequency**: `80MHz`
- **Partition Scheme**: `Default 4MB with spiffs`
- **Upload Speed**: `921600`

---

## 🚀 Quick Setup & Deployment

### Step 1: Discover Receiver MAC Address
1. Open `tools/Get_MAC_Address/Get_MAC_Address.ino` and flash to your **Motor Driver ESP32**.
2. Open Serial Monitor at **115200 baud**.
3. Copy the output snippet, e.g.:
   ```cpp
   uint8_t receiverAddress[] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF };
   ```

### Step 2: Configure & Flash Vision Node
1. Open `firmware/ESP32_CAM_V10/ESP32_CAM_V10.ino`.
2. Update line 11 with your receiver's MAC address from Step 1:
   ```cpp
   uint8_t receiverAddress[] = { 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX };
   ```
3. Connect `GPIO 0` to `GND`, tap reset, and flash via FTDI programmer.
4. Disconnect `GPIO 0` from `GND` and reboot.

### Step 3: Flash Motor Driver Node
1. Open `firmware/ESP32_MotorDriver_V10/ESP32_MotorDriver_V10.ino`.
2. Flash to the motor controller ESP32.
3. Once booted, the motor controller will wait for packets from the camera. Placing the camera over a black line will initiate autonomous steering.

---

## 🛠️ Tuning Guide

| Parameter | Location | Description | Default |
|---|---|---|---|
| `BASE_SPEED` | Motor Sketch | Nominal cruising speed (0-255) | `50` |
| `MAX_SPEED` | Motor Sketch | Upper ceiling on motor PWM output | `60` |
| `Kp` | Motor Sketch | Proportional gain: immediate track-centering reaction | `1.2` |
| `Kd` | Motor Sketch | Derivative gain: dampens oscillation and initiates braking | `2.0` |
| `Ki` | Motor Sketch | Integral gain: corrects steady-state offsets on long arcs | `0.002` |
| `ROI_START_Y` / `ROI_END_Y` | Camera Sketch | Vertical pixel range scanned for line detection | `60` / `120` |
| `s->set_hmirror(s, 0)` | Camera Sketch | Mirror horizontally (set to `1` if steering is reversed) | `0` |
