# Y Register Map (Y0 output byte)

This file documents the bit-level mapping for the Y output byte stored at `EzApp::Y` offset 0 (byte offset 0). The code writes the full Y0 byte to the PCF8574 output port.

Location
- Y0 is read/written in `main/ez_dio.cpp` (see `ez_dio_task`): `app.readInt16(EzApp::Y, 0, y0);` and `app.writeInt16(EzApp::Y, 0, y0);`.

Bit mapping (LSB = bit 0)
| Bit | Mask | Name / purpose | Notes / code reference |
|---:|---:|---|---|
| 0 | 0x01 | Door Close (solenoid ON) | Set when closing the door (manual/auto). `y0 |= 0x0001` — `ez_dio_control_manual` / `ez_dio_control_auto` |
| 1 | 0x02 | Door Open (solenoid ON)  | Set when opening the door. `y0 |= 0x0002` — `ez_dio_control_manual` / `ez_dio_control_auto` |
| 2 | 0x04 | Pressure control (pressurize) | `y0 |= 0x0004` used for pressurize step (Step 30 etc.)
| 3 | 0x08 | Exhaust / Discharge | `y0 |= 0x0008` used for exhaust step (Step 50 etc.)
| 4 | 0x10 | Temperature control (heater) | `y0 |= 0x0010` used for temperature control step
| 5 | 0x20 | Oxygen supply valve | `y0 |= 0x0020` used to enable oxygen supply
| 6 | 0x40 | (unused / reserved) | No active usage in current code
| 7 | 0x80 | (unused / reserved) | No active usage in current code

Behavior notes
- The application maintains `y0` as a 16-bit signed value but only the lower byte is written to the PCF8574 device: `uint8_t out_byte = static_cast<uint8_t>(y0 & 0xFF);`.
- The main DIO loop updates `EzApp::Y` at byte offset 0 and then writes that byte to the PCF8574 I2C output address (`EzApp::PCF_OUTPUT_ADDR`). See `ez_dio_task` in `main/ez_dio.cpp`.
- Several state-machine steps set/clear individual bits (examples: Steps 20, 30, 32, 34, 36, 50, 80, 92). Door open/close use mutually exclusive bits 0 and 1.
- The `y0_toggle_task` contains commented test code to cycle bits 0..7 for debugging.

Code references
- `main/ez_dio.cpp` — read/write of `EzApp::Y` at offset 0, functions `ez_dio_control_manual()` and `ez_dio_control_auto()` implement the bit logic.

If you'd like, I can:
- Add per-bit comments near the `app.writeInt16(EzApp::Y, 0, y0);` call in `ez_dio.cpp` linking to this doc.
- Generate a small test task to toggle named outputs for validation (enable via `start_y0_toggle_test()`).
