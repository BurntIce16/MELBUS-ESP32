# Volvo MELBUS ESP32

## Goals

- Clean up code and convert to a library for ease of use
- Get text functioning on screen
- Document protocol
- Ensure correct wiring
- Enable Bluetooth audio streaming

## Resources

- [Volvo MELBUS Documentation (Archived)](https://web.archive.org/web/20180217204549/http://volvo.wot.lv/wiki/doku.php?id=melbus)
- [Auxiliary Input for Volvo HU Radios](https://gizmosnack.blogspot.com/2015/11/aux-in-volvo-hu-xxxx-radio.html?m=1)
- [AuxAdapter Website](https://www.auxadapter.se/?page_id=712)
- [Volvo MELBUS Wiki](https://volvo.wot.lv/doku.php?id=melbus)

## Current Issues

- Pin definitions need clarification
  - Arduino uses GPIO pins, while ESP32 uses pin ports
  - Adjust pin definitions for compatibility
- EEPROMClass update method missing
- Interrupt logic needs to be rewritten for hardware compatibility
