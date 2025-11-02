# Changelog

All notable changes to this project will be documented in this file.

---

## [1.5.0] - 2025-11-02

This version is a **critical bugfix** release that corrects the logic in the `v1.4.9` software homing feature.

### Fixed

* **`go_home` Direction:** Fixed a critical bug in `executeGoHome()` where the `relativeMove` calculation was being incorrectly inverted (`-relativeMove`) before being sent to the motor. The logic is now correct, and the motor moves in the proper direction to return to the set home position.
* **`read_pulses` Parser:** Fixed a bug in `parseAndPrintResponse()` where the code was checking for `case 5:` for a `read_pulses` response. The manual and logs confirm the packet is **6 bytes** (1-byte addr + 4-byte data + 1-byte checksum). The router is now corrected to look for `case 6:`, which allows the `read_pulses` response to be parsed correctly.
* **Firmware Bug Hotfixes:** Re-instated the hotfixes for the driver's firmware bug where it sends an extra `0x00` byte after 3-byte (Status) and 8-byte (Encoder) packets.

### Changed

* **`messageTimeout`:** Increased from 10ms to **20ms**. This provides a safer time window to capture complete multi-byte packets from the driver and prevents corrupted/split messages from being read.
* **Code Comments:** All functions have been fully documented with Doxygen-style `@brief`, `@param`, and `@return` comments to improve readability and maintainability.

---

## [1.4.9]

This version was a **major architectural update** that abandoned the buggy, circular `read_encoder` (0x30) command for position tracking.

### Added

* **Software Absolute Homing:** Implemented the `set_home` and `go_home <speed>` commands.
* **State Machine:** Added a global state machine (`PendingAction`) to handle the asynchronous `request -> wait -> execute` logic required for `set_home` and `go_home`.
* **Pulse Position Tracking:** Added global variables (`absoluteHomeOffset_pulses`, `currentAbsolutePosition_pulses`) to store the 32-bit pulse count.

### Changed

* **Core Logic:** The state machine was re-architected to use the **`read_pulses` (0x33)** command as the source for position tracking. This is the correct, non-circular 32-bit counter that directly matches the `move` command's units.
* **Help Menu:** Updated the `help 2` and `help 3` menus to reflect the new software homing commands and clarify the difference between `read_pulses` (for position) and `read_encoder` (for raw angle).
* **Response Parser:** The `parseAndPrintResponse()` function was updated to support a new packet length (`case 6:`) to handle the `read_pulses` response.

### Known Bugs (Fixed in v1.5.0)

* This version was released with the two critical bugs that `v1.5.0` corrects.