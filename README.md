# GPIO Interrupt with FreeRTOS for Raspberry Pi Pico 2 (RP2350)

This project demonstrates how to use GPIO interrupts with FreeRTOS on the Raspberry Pi Pico 2 (RP2350). It implements a two-task system where one task toggles a GPIO pin while another task handles interrupts from that pin.

## Project Overview

The project uses FreeRTOS to manage two tasks that communicate via a binary semaphore:

1. **Toggle Task** (Priority 1): Toggles GPIO 3 every 1 second and mirrors the state to the onboard LED
2. **Interrupt Handler Task** (Priority 2): Runs idle, waiting for GPIO interrupts on GPIO 2. When triggered, it wakes up via semaphore and processes the interrupt event

### Hardware Setup

Connect GPIO 3 to GPIO 2 with a jumper wire to demonstrate the interrupt functionality.

- **GPIO 2**: Input pin with pull-down resistor, configured for interrupt on rising and falling edges
- **GPIO 3**: Output pin that toggles every 1 second
- **GPIO 25** (LED): Onboard LED that mirrors GPIO 3 state

## Prerequisites

Before building this project, ensure you have:

- **Pico SDK**: Installed and `PICO_SDK_PATH` environment variable set
- **ARM Toolchain**: `arm-none-eabi-gcc` for cross-compilation
- **CMake**: Version 3.13 or newer
- **Git**: For cloning the repository and initializing submodules
- **Hardware**: Raspberry Pi Pico 2 (RP2350)

## Getting Started

### 1. Clone the Repository

```bash
git clone git@github.com:guanleustc/debugtarget.git
cd debugtarget
```

### 2. Initialize FreeRTOS Submodule

**IMPORTANT**: The FreeRTOS-Kernel is included as a git submodule and must be initialized with its nested submodules to get the RP2350 ports:

```bash
git submodule update --init --recursive
```

This command will:
- Initialize the FreeRTOS-Kernel submodule
- Recursively initialize the Community-Supported-Ports submodule (contains RP2350 ARM and RISC-V ports)

### 3. Build the Project

```bash
# Create and enter build directory
mkdir -p build
cd build

# Configure with CMake
cmake -DCMAKE_BUILD_TYPE=Debug -DPICO_BOARD=pico2 -DPICO_PLATFORM=rp2350 ../

# Build
make -j$(nproc)
```

### 4. Flash to Pico 2

1. Hold the **BOOTSEL** button on your Pico 2 while plugging it into USB
2. The Pico will appear as a mass storage device
3. Copy `build/hello_gpio_irq.uf2` to the Pico drive
4. The Pico will automatically reboot and run the firmware

## Build Output

After a successful build, you'll find these files in the `build/` directory:

- `hello_gpio_irq.uf2` - Flash firmware (drag-and-drop to Pico)
- `hello_gpio_irq.elf` - ELF executable with debug symbols
- `hello_gpio_irq.bin` - Raw binary firmware
- `hello_gpio_irq.hex` - Intel HEX format
- `hello_gpio_irq.elf.map` - Memory map
- `hello_gpio_irq.dis` - Disassembly listing

## Project Structure

```
.
├── hello_gpio_irq.c              # Main source with FreeRTOS tasks
├── CMakeLists.txt                # CMake build configuration
├── FreeRTOSConfig.h              # FreeRTOS configuration for RP2350
├── pico_sdk_import.cmake         # Pico SDK integration
├── FreeRTOS_Kernel_import.cmake  # FreeRTOS kernel integration
├── FreeRTOS-Kernel/              # FreeRTOS submodule (with RP2350 support)
├── CLAUDE.md                     # Detailed technical documentation
├── .gitignore                    # Git ignore rules
├── .gitmodules                   # Git submodule configuration
└── build/                        # Build output (generated)
```

## FreeRTOS Configuration

The project is configured for RP2350 (Cortex-M33) with:
- **CPU Clock**: 150MHz (RP2350 default)
- **Tick Rate**: 1000Hz (1ms tick)
- **Heap Size**: 128KB
- **Priority Levels**: 32 (5-bit priority)
- **Memory Allocation**: Heap4
- **FPU**: Enabled
- **Pico SDK Interop**: Enabled for sync and time functions

## Serial Output

The firmware outputs debug information via USB serial (CDC). To view:

```bash
# Linux
screen /dev/ttyACM0 115200
# or
minicom -D /dev/ttyACM0 -b 115200

# macOS
screen /dev/tty.usbmodem* 115200
```

Expected output:
```
===========================================
Hello GPIO IRQ with FreeRTOS on RP2350
===========================================
Connect GPIO 3 to GPIO 2 with a jumper wire
===========================================

[Setup] GPIO 2 configured as input with pull-down
[Setup] GPIO 3 configured as output
[Setup] LED on GPIO 25 configured
[Setup] Interrupt semaphore created
[Setup] IRQ enabled on GPIO 2 for RISE and FALL edges

[Setup] ToggleTask created
[Setup] InterruptTask created

Starting FreeRTOS scheduler...

[ToggleTask] Started - will toggle GPIO 3 every 1 second
[InterruptHandlerTask] Started - waiting for interrupts on GPIO 2
[ToggleTask] GPIO 3 toggled to 1, GPIO 2 reads: 1
[InterruptHandlerTask] GPIO 2 interrupt: EDGE_RISE
...
```

## Rebuilding

To rebuild from scratch:

```bash
cd build
make clean
make -j$(nproc)

# Or completely rebuild:
cd ..
rm -rf build
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Debug -DPICO_BOARD=pico2 -DPICO_PLATFORM=rp2350 ../
make -j$(nproc)
```

## Troubleshooting

### FreeRTOS Submodule Not Found

If you see errors about missing FreeRTOS files, ensure you initialized the submodules:

```bash
git submodule update --init --recursive
```

### RP2350 Ports Missing

If CMake complains about missing RP2350 ports, verify the Community-Supported-Ports submodule:

```bash
ls FreeRTOS-Kernel/portable/ThirdParty/Community-Supported-Ports/GCC/
# Should show: RP2350_ARM_NTZ and RP2350_RISC-V
```

### PICO_SDK_PATH Not Set

Set the environment variable to your Pico SDK location:

```bash
export PICO_SDK_PATH=/path/to/pico-sdk
```

## Technical Details

For in-depth technical documentation about the FreeRTOS architecture, task design, synchronization mechanisms, and implementation details, see [CLAUDE.md](CLAUDE.md).

## License

This project is based on the Raspberry Pi Pico SDK examples and uses the BSD-3-Clause license.

## References

- [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk)
- [FreeRTOS Kernel](https://github.com/FreeRTOS/FreeRTOS-Kernel)
- [Raspberry Pi Pico Documentation](https://www.raspberrypi.com/documentation/microcontrollers/)
- [FreeRTOS Documentation](https://www.freertos.org/Documentation/RTOS_book.html)
