# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a Raspberry Pi Pico 2 (RP2350) GPIO interrupt example project using the Pico SDK with FreeRTOS. The project demonstrates how to use GPIO interrupts with FreeRTOS tasks. It implements two tasks:

1. **Toggle Task**: Toggles GPIO 3 every 1 second (and mirrors state to the LED)
2. **Interrupt Handler Task**: Runs idle and responds to GPIO interrupts on GPIO 2 when triggered by GPIO 3

The tasks communicate using a binary semaphore - the GPIO ISR signals the semaphore, which wakes up the interrupt handler task to process the event.

## Build System

This project uses CMake with the Raspberry Pi Pico SDK build system and FreeRTOS.

### Prerequisites
- PICO_SDK_PATH environment variable must be set (currently: `/home/le/pico/pico-sdk`)
- Pico SDK installed and properly configured
- ARM cross-compilation toolchain (arm-none-eabi-gcc)
- FreeRTOS-Kernel with RP2350 support:
  - **Official upstream**: https://github.com/FreeRTOS/FreeRTOS-Kernel
  - **MUST clone with `--recurse-submodules`** to get RP2350 ports from Community-Supported-Ports
  - Should be cloned in the project root directory
- PICO_PLATFORM set to `rp2350` for Raspberry Pi Pico 2 support

### Build Commands

```bash
# Setup FreeRTOS-Kernel (first time only, in project directory)
cd /home/le/pico/test/hello_gpio_irq
git clone --recurse-submodules https://github.com/FreeRTOS/FreeRTOS-Kernel.git

# Initial build setup (from project root)
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Debug -DPICO_BOARD=pico2 -DPICO_PLATFORM=rp2350 ../

# Build the project
make -j$(nproc)

# Clean build
make clean

# Rebuild from scratch
cd ..
rm -rf build
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Debug -DPICO_BOARD=pico2 -DPICO_PLATFORM=rp2350 ../
make -j$(nproc)
```

**Note**: The project expects FreeRTOS-Kernel in the project root directory. The FreeRTOS_Kernel_import.cmake will automatically detect it.

### Build Output Files

After building, the following files are generated in the `build/` directory:
- `hello_gpio_irq.uf2` - Flash firmware file (drag-and-drop to Pico in bootloader mode)
- `hello_gpio_irq.elf` - ELF executable with debug symbols
- `hello_gpio_irq.bin` - Raw binary firmware
- `hello_gpio_irq.hex` - Intel HEX format firmware
- `hello_gpio_irq.elf.map` - Memory map file
- `hello_gpio_irq.dis` - Disassembly listing

## Deployment

To flash the firmware to a Raspberry Pi Pico:

1. Hold the BOOTSEL button on the Pico while plugging it into USB
2. The Pico will appear as a mass storage device
3. Copy `build/hello_gpio_irq.uf2` to the Pico drive
4. The Pico will automatically reboot and run the firmware

## Project Structure

- `hello_gpio_irq.c` - Main source file with FreeRTOS tasks for GPIO interrupt handling
- `CMakeLists.txt` - CMake build configuration with FreeRTOS integration
- `FreeRTOSConfig.h` - FreeRTOS configuration for RP2350
- `pico_sdk_import.cmake` - Pico SDK integration script (standard boilerplate)
- `FreeRTOS_Kernel_import.cmake` - FreeRTOS kernel integration script
- `FreeRTOS-Kernel/` - FreeRTOS kernel source (cloned from https://github.com/FreeRTOS/FreeRTOS-Kernel)
- `build/` - Build output directory (generated)

## Hardware Configuration

- **GPIO 2**: Input pin with pull-down resistor, configured for interrupt on rising and falling edges
- **GPIO 3**: Output pin that toggles every 1 second (controlled by Toggle Task)
- **GPIO 25** (PICO_DEFAULT_LED_PIN): Onboard LED, mirrors GPIO 3 state
- **Required hardware setup**: Connect GPIO 3 to GPIO 2 with a jumper wire

## SDK Integration

The project uses:
- **Pico SDK** (`pico_stdlib`): Standard I/O over UART (enabled), GPIO hardware abstraction
- **FreeRTOS-Kernel**: Latest main branch with RP2350 support (Arm Cortex-M33 and RISC-V)
- **FreeRTOS-Kernel-Heap4**: Memory allocation scheme for FreeRTOS

## FreeRTOS Architecture

### Task Design

**Task 1: vToggleTask** (Priority 1)
- Toggles GPIO 3 and LED every 1 second using `vTaskDelay(pdMS_TO_TICKS(1000))`
- Runs continuously in a loop
- Lower priority than interrupt handler task

**Task 2: vInterruptHandlerTask** (Priority 2)
- Runs idle, waiting on binary semaphore
- Woken by GPIO ISR via `xSemaphoreGiveFromISR()`
- Processes and prints interrupt event information
- Higher priority ensures immediate response to interrupts

### Synchronization

- **Binary Semaphore** (`xInterruptSemaphore`): Used for ISR-to-task communication
- ISR gives semaphore using `xSemaphoreGiveFromISR()` with proper context switching
- Interrupt handler task blocks on `xSemaphoreTake()` with `portMAX_DELAY`

### Main Flow

1. Initialize stdio and wait for UART serial
2. Configure GPIO pins (2 = input with pull-down, 3 = output, 25 = LED)
3. Create binary semaphore for interrupt signaling
4. Enable GPIO interrupt with callback on rising/falling edges
5. Create Toggle Task (priority 1)
6. Create Interrupt Handler Task (priority 2)
7. Start FreeRTOS scheduler - never returns

### Key Functions

- `gpio_callback()`: ISR that stores interrupt info and signals semaphore to wake handler task
- `vToggleTask()`: FreeRTOS task that toggles GPIO 3 every second
- `vInterruptHandlerTask()`: FreeRTOS task that waits for and processes interrupts
- `gpio_event_string()`: Utility to convert interrupt event bitmask to human-readable string

## FreeRTOS Configuration

The `FreeRTOSConfig.h` is configured for RP2350 (Cortex-M33):
- CPU clock: 150MHz (RP2350 default)
- Tick rate: 1000Hz (1ms tick)
- Heap size: 128KB
- 4-bit priority levels (16 priority levels)
- Heap4 memory allocation scheme
- Preemptive scheduling enabled

## Serial Output

The program outputs debug information via UART at startup and during operation. Connect a USB-to-UART adapter to GPIO 0 (TX) and GPIO 1 (RX), then use a terminal program at 115200 baud (typically `/dev/ttyUSB0` on Linux) using tools like `minicom`, `screen`, or `picocom`.
- # This is for PICO 2 that uses RP2350.
- cmake with: cmake -DCMAKE_BUILD_TYPE=Debug -DPICO_BOARD=pico2 -DPICO_PLATFORM=rp2350 ../