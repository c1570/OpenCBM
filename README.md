# XUM1541 WebUSB Interface

A modern web-based interface for controlling Commodore disk drives through the XUM1541 USB adapter using WebUSB technology.

## Features

✅ **Device Connection**
- Connect to XUM1541 devices via WebUSB
- Real-time connection status and device information
- Automatic device initialization and capability detection

✅ **Directory Operations**
- Read disk directory from CBM drives (`cbmctrl dir 8` equivalent)
- Display file listings with block counts and filenames
- PETSCII to ASCII conversion for proper display

✅ **Drive Control**
- Reset CBM drives (`cbmctrl reset` equivalent)
- Support for drives 8, 9, 10, and 11
- Real-time status and error reporting

✅ **D64 Disk Image Operations**
- Upload D64 files to drives (`d64copy DISK.D64 8` equivalent)
- Download disk contents as D64 files (`d64copy 8 DISK.D64` equivalent)
- Support for both 35-track and 40-track D64 images
- Progress tracking with track/sector information

## Browser Requirements

- **Chrome 61+** or **Edge 79+** (WebUSB support required)

## Hardware Requirements

- XUM1541 device (RP2040 variant)
- Compatible Commodore disk drive (1541, 1571, etc.)
- Proper IEC cables connecting drive to XUM1541
