# RP Pico Tester

A Windows tool plus Pico firmware to stress-test Raspberry Pi Pico (RP2040) boards, built for checking cheap AliExpress clones.

I'm a fan of the cheap RP Pico boards on AliExpress, but I noticed that some of the modules have problems. So I made this software to test their capabilities and see whether they crash under high load.

## What it tests

| Test | What it finds |
|---|---|
| **USB serial stream** | Sends CRC-protected 512-byte frames at full speed, the Pico echoes them back. Detects corrupted frames, lost frames, and USB disconnects. |
| **CPU stress (core 1)** | Runs integer math, the hardware divider, 64-bit math, a bitwise CRC32, soft-float and an 8 KB RAM pattern test in a loop on the second core, while core 0 handles the USB stream. Errors are reported to the PC every second. Catches heat or power-related faults that only appear under combined load. |
| **RAM test** | Data patterns, walking ones and zeros, address-line test, pseudo-random data and March C- over the free heap. |
| **Flash surface test** | Erases and rewrites every sector after the firmware image, with 6 patterns, reading each write back 4 times directly from the chip (bypassing the cache). Then re-reads all sectors for address aliasing and data loss. Reports bad bits, unstable reads, and erase/program timing. |
| **Flash size detection** | Reads the JEDEC ID and checks whether the address space wraps around, so a **fake-capacity chip** (for example a 2 MB chip reporting 4 MB) is flagged. |

It also has an **Upload UF2** button that puts connected boards into BOOTSEL mode and programs them with a `.uf2` file, which is handy when testing several boards.

## Features

- Finds every connected Pico automatically (USB VID `2E8A`) and shows its COM port.
- **Start / Stop** tests all connected Picos at the same time, with one live status line per board (TX/RX speed, frame counters, core-1 pass and error counters, temperature) and a PASS/FAIL summary per board when you stop.
- **Flash test** runs on a single board (exactly one Pico connected).
- **Upload UF2** programs every connected board with the newest `.uf2` next to the exe.

## Files

- `main.cpp` and `CMakeLists.txt`: Windows app (C++14, Win32 GUI, no external dependencies)
- `pico_tester.ino`: Pico firmware for the Arduino IDE

## Setup

### Pico firmware
1. In the Arduino IDE, install **Raspberry Pi Pico/RP2040** by Earle F. Philhower, III (Boards Manager).
2. Tools > Board: Raspberry Pi Pico. Tools > Flash Size: the size the board is *supposed* to have. No filesystem is needed.
3. Put `pico_tester.ino` in a folder named `pico_tester` and upload it.
4. To get a `.uf2` file for the Upload button: Sketch > Export Compiled Binary.

### Windows app
- **Visual Studio 2019/2022:** add `main.cpp` to a Windows Desktop project (Character Set: Unicode), or open the folder with `CMakeLists.txt`.
- Put your compiled `.uf2` next to `PicoTester.exe` if you want to use Upload UF2.

## Using it

1. Plug in one or more Picos running `pico_tester.ino`.
2. **Start**: the stream test runs until you press **Stop**. Leave "Also run CPU/RAM test on core 1" ticked for the hardest test. For hours-long runs, use a good cable and a good USB port.
3. **Flash test**: connect one board and press the button. About 0.5 s per 4 KB sector (a 2 MB chip takes about 5 minutes). The report ends with the detected flash size and PASS or FAIL.

How to read the counters:

- `bad`: corrupted frames received by the PC
- `lost`: frames that never came back
- `Pico rx bad`: frames that arrived corrupted at the Pico
- `core1 err int/crc/float/ram`: errors from the CPU and RAM stress test

Any non-zero error counter means the board (or your cable, port or hub) has a problem. Try another cable or port before blaming the board.

## Warnings

- The flash test erases everything after the firmware image up to the end of the chip. The running firmware is not touched, but any LittleFS or EEPROM data in that space is wiped.
- Upload UF2 programs **every** connected Pico with the same file.
- Flash wear: each tested sector sees 8 erase cycles (flash is typically rated around 100,000), which is negligible but not zero.

## Limitations

- The firmware image area (about 100-200 KB) cannot be erased while it is running, so it is only checked indirectly.
- The RAM test covers the free heap (most of the 264 KB). The firmware's own variables, stacks and USB buffers cannot be tested while running.
- USB full speed tops out around 1 MB/s per direction, and several boards on one hub share that bandwidth.
- Written for the RP2040 Pico. Pico 2 (RP2350) is untested.
- The Windows app was written without access to a Windows compiler. If you hit a build error, please open an issue.

## How it works

The PC sends frames of 512 bytes: a 4-byte sequence number, 504 bytes of pseudo-random data and a CRC32. The Pico checks the CRC, counts bad frames, and echoes good ones. Once per second it adds a status frame (sequence number with bit 31 set) carrying its own counters as text. The PC checks the sequence numbers to count lost frames.

The flash test is text-based: the app sends `FLASH`, the Pico prints progress lines and ends with `DONE`. Other commands are `STREAM`, `STREAM CPU`, `BOOT` and `PING`.

## Additional Info

This Program is vibe coded via claude by Anthropic. I do not guarantee the functionality of this program. Please report any Bug or problem.

## License

MIT License

Copyright (c) 2026 MasterAlive

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
