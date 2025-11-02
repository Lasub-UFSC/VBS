# MKS SERVO42C - ESP32 Serial Command Interface

This is an Arduino sketch for the **ESP32 DOIT DEVKIT V1** that provides a full-featured, human-readable command interface for the MKS SERVO42C closed-loop stepper driver.

This code acts as an advanced, **stateful** bridge. It translates intuitive commands (e.g., `move 3200 30`) into the binary packets required by the driver, handling all parsing, validation, and checksum calculation.

More importantly, it adds critical features *on top* of the driver's firmware, such as a **software-based absolute homing system** (`set_home`/`go_home`). This feature uses the driver's `read_pulses` (0x33) command to track the non-circular, 32-bit pulse counter, allowing it to return to a "zero" position across many rotations—a feature not available in the driver's default commands.

This code is based on the **MKS SERVO42C V1.1.2 User Manual**.

## Features

* **Human-Readable Commands:** Control the driver with simple strings (e.g., `set_kp 1500`) instead of hex.
* **Smart Response Parsing:** The code validates and interprets the driver's raw hex responses, printing human-readable output (e.g., `Command OK!` or `Absolute Position (pulses): 4096`).
* **Software-Based Absolute Homing:** Adds `set_home` and `go_home <speed>` commands that use the reliable `read_pulses` counter for true, multi-rotation homing.
* **Full Command Implementation:** Includes all commands from the V1.1.2 manual.
* **Automatic Checksum:** The correct 8-bit additive checksum is automatically calculated for sending and validated on receiving.
* **Firmware Bug Hotfixes:** Includes workarounds for known driver firmware bugs (like extra `0x00` bytes on responses) to ensure stable communication.
* **Data-Driven & Scalable:** The architecture uses `struct` arrays as "dictionaries" to map commands, making the code clean and easy to add to.
* **Robust Error Handling:** The parser returns specific error codes (e.g., `ERR_ARG_OUT_OF_RANGE`) and prints clear, helpful messages to the user.
* **Interactive Help Menu:** A `help` command provides a multi-level menu to guide the user.
* **Non-Blocking:** The code uses non-blocking logic to handle driver responses, ensuring the `loop()` is never stuck.

## Hardware Requirements

* **ESP32 DOIT DEVKIT V1** (or similar ESP32 board)
* **MKS SERVO42C** Closed-Loop Driver (V1.1.x firmware)
* Dupont-style Jumper Wires
* 12V-24V Power Supply for the driver

## Setup & Installation

### 1. Driver Configuration (CRITICAL STEP)

You **must** configure the driver itself using its onboard display and buttons (`Menu`, `Next`, `Enter`). The code will not work otherwise.

1.  **Power on** the MKS SERVO42C.
2.  Press the **`Menu`** button to enter settings.
3.  Use the **`Next`** button to navigate to **`Mode`**.
4.  Press **`Enter`**, use **`Next`** to select **`CR_UART`**, and press **`Enter`** to confirm.
    * *(The factory default is `CR_vFOC`, which will ignore all serial commands).*
5.  Use **`Next`** to navigate to **`UartBaud`**.
6.  Press **`Enter`**, use **`Next`** to select **`38400`**, and press **`Enter`** to confirm.
    * *(This must match the `DRIVER_BAUD_RATE` define in the code).*
7.  Use **`Next`** to navigate to **`UartAddr`**.
8.  Press **`Enter`** and ensure it is **`0xe0`**.
    * *(This must match the `DRIVER_ADDR` define in the code).*
9.  Navigate to **`Exit`** and press **`Enter`** to save and exit the menu.

### 2. Hardware Wiring

Connect the ESP32 to the driver's 4-pin `Usart (TTL)` port.

| ESP32 Pin | Driver Pin | Description |
| :--- | :--- | :--- |
| `GND` | `G` (GND) | Common Ground |
| `GPIO 17` (TX) | `RX` | ESP32 Transmit -> Driver Receive |
| `GPIO 16` (RX) | `TX` | ESP32 Receive <- Driver Transmit |

**Note:** If you get no responses, try swapping the `RX` and `TX` pins.

### 3. Software (Arduino IDE)

1.  Open this `.ino` file in the Arduino IDE.
2.  Select **"DOIT ESP32 DEVKIT V1"** (or your board) from the **Tools > Board** menu.
3.  Upload the code.
4.  Open the **Serial Monitor**.
5.  Set the Serial Monitor's baud rate to **115200** (this is the speed for PC-to-ESP32).
6.  Set the line ending to **"Newline"** (bottom-right of the Serial Monitor).
7.  Type `help` and press Enter to begin.

## How to Use (Command API)

The interface is controlled by sending commands over the Arduino Serial Monitor.

Type `help` to see the main category menu:

--- MKS Controller Help Menu --- Usage: help <category_number>

Motion Commands

Homing (Zero) Commands

Read (Feedback) Commands

Configuration (PID) Commands

Configuration (Driver) Commands

Type `help 2` to see the crucial distinction between the two types of homing:

--- 2. Homing (Zero) Commands --- 
set_home                : (Software) Sets current pulse count as absolute 0. 
go_home <speed>         : (Software) Returns to absolute 0 pulse count. 
--- Driver-Internal Homing (Single-Rotation) --- 
set_zero                : (Driver) Sets the current angle as 0 datum. 
goto_zero               : (Driver) Returns to the 0 angle datum. 
set_zero_mode <0-2>     : Sets driver homing mode (0=Off, 1=Dir, 2=Near).
set_zero_speed <0-4>    : Sets driver homing speed (0=fast, 4=slow).
set_zero_dir <0|1>      : Sets driver homing direction (0=CW, 1=CCW).

## To-Do / Future Improvements

This code provides a solid foundation, but there is always room for improvement.

* **Min/Max Software Endstops:** Now that the ESP32 maintains `currentAbsolutePosition_pulses`, the next logical step is to add `set_min_pos` and `set_max_pos` commands. The `handleMoveCommand` could then check against these limits *before* sending a move to the driver.
* **Multi-Driver Support:** Refactor the code to manage multiple driver addresses. This would likely involve prefixing commands with the address (e.g., `e0 move 3200 30` or `e1 set_kp 1000`) and abstracting the `DRIVER_ADDR` constant.
* **Complete Response Parsing:** The code now parses `Status`, `Pulses`, `Encoder`, and `Error` responses. It should be expanded to parse all other responses and provide human-readable output for them.
* **Automatic Polling:** Implement a non-blocking "auto-poll" feature (e.g., `poll_pulses 100`) that automatically requests data (like pulse position) at a set interval (e.g., every 100ms).
* **Library Conversion:** Convert this standalone `.ino` sketch into a formal Arduino C++ library (a `.h` and `.cpp` file) with a class (e.g., `MKS_SERVO42C`) and public methods (e.g., `driver.goHome(30)`). This would be the ultimate step for usability in other projects.

## License

This project is licensed under the **MIT License**.

MIT License

Copyright (c) 2025 Lucas Barcaro

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.