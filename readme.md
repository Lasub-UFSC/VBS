# MKS SERVO42C - ESP32 Serial Command Interface

This is an Arduino sketch for the **ESP32 DOIT DEVKIT V1** that provides a full-featured, human-readable command interface for the MKS SERVO42C closed-loop stepper driver.

Instead of needing to construct and send raw hexadecimal packets (e.g., `E0 F3 01 D4`), you can send intuitive commands like `enable` or `move 3200 30` directly from the Serial Monitor.

This code acts as an advanced bridge, handling all command parsing, argument validation, and automatic 8-bit checksum calculation. It is intended to be a robust debugging tool, a foundation for a larger project, or a contribution to the MKS community.

This code is based on the **MKS SERVO42C V1.1.2 User Manual**.

## Features

* **Human-Readable Commands:** Control the driver with simple strings (e.g., `set_kp 1500`) instead of hex.
* **Full Command Implementation:** Includes all commands from the V1.1.2 manual (Motion, Homing, PID, Reading, and Configuration).
* **Automatic Checksum:** The correct 8-bit additive checksum is automatically calculated and appended to every command.
* **Data-Driven & Scalable:** The architecture uses `struct` arrays as "dictionaries" to map commands. Adding new commands is as simple as adding a new line to the array.
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

* Type `help` to see the main category menu.
* Type `help <number>` (e.g., `help 1`) to see the commands for that specific category.

### Command Categories

* `help 1`: Motion Commands (`enable`, `move`, `spin`, `set_acc`, etc.)
* `help 2`: Homing (Zero) Commands (`set_zero`, `goto_zero`, etc.)
* `help 3`: Read (Feedback) Commands (`read_encoder`, `read_error_angle`, etc.)
* `help 4`: Configuration (PID) Commands (`set_kp`, `set_ki`, `set_kd`)
* `help 5`: Configuration (Driver) Commands (`calibrate`, `set_work_mode`, `set_baud`, etc.)

## To-Do / Future Improvements

This code provides a solid foundation, but there is always room for improvement.

* **Multi-Driver Support:** Refactor the code to manage multiple driver addresses. This would likely involve prefixing commands with the address (e.g., `e0 move 3200 30` or `e1 set_kp 1000`) and abstracting the `DRIVER_ADDR` constant out of the helper functions.
* **Response Parsing & Validation:** Currently, the code prints the raw hex response (e.g., `Driver Response <- [ 0xE0 0x01 0xE1 ]`). A future improvement would be to parse this response, validate its checksum, and print a human-readable confirmation (e.g., `Command OK!` or `Encoder Value: 4096`).
* **Automatic Polling:** Implement a non-blocking "auto-poll" feature (e.g., `poll_encoder 100`) that automatically requests data (like encoder position) at a set interval (e.g., every 100ms).
* **Library Conversion:** Convert this standalone `.ino` sketch into a formal Arduino C++ library (a `.h` and `.cpp` file) with a class (e.g., `MKS_SERVO42C`) and public methods (e.g., `driver.move(3200, 30)`). This would be the ultimate step for usability in other projects.

## License

This project is licensed under the **MIT License**.

MIT License

Copyright (c) 2025 Lucas Barcaro

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.