# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Zephyr RTOS firmware for a custom **1SJ LoRa Module** — an STM32L072CZ MCU paired with a Semtech SX1262 LoRa transceiver. The firmware exposes a UART shell for interactive control and runtime diagnostics.

## Build & Flash

This is a **Zephyr west** project built from within STM32CubeIDE or the Zephyr SDK CLI.

```bash
# Configure and build (from workspace root or project root)
west build -b 1sj_module

# Flash via J-Link
west flash --runner jlink

# Flash via STM32CubeProgrammer (SWD)
west flash --runner stm32cubeprogrammer

# Clean build
west build -t pristine
```

The board name is `1sj_module` (defined in `boards/1sj_module/`).

## Hardware

| Resource | Assignment |
|---|---|
| MCU | STM32L072CZ @ 32 MHz (HSI16 → PLL ×4 ÷2) |
| Shell UART | USART2, PA2 TX / PA3 RX, 115200 baud |
| SX1262 SPI | SPI2: PB13 SCK / PB14 MISO / PB15 MOSI, PB12 NSS |
| SX1262 control | RESET=PB1, BUSY=PA15, DIO1=PB0 |
| TCXO | 3.3 V via DIO3, 5 ms startup delay |
| Flash | 192 KiB; 32 KiB NVS storage at 0x28000 |

## Architecture

```
src/
  main.c          — Registers the "hello" shell command; shell starts automatically
  task_sx1262.c   — Dedicated Zephyr thread (2 KiB stack, priority 7) that
                    initialises the SX1262 and registers "lora" shell subcommands

boards/1sj_module/
  1sj_module.dts       — Device tree: clocks, pinmux, USART2, SPI2 + SX1262 node
  1sj_module_defconfig — Board Kconfig defaults (GPIO, serial, MPU)
  Kconfig.1sj_module   — Selects SOC_STM32L072XX
  board.cmake          — Flash runner configuration (J-Link / STM32CubeProgrammer)
  board.yml            — Board metadata

prj.conf — Application Kconfig: shell, logging, SPI, LoRa (CONFIG_LORA_SX126X)
```

The SX1262 is accessed through the Zephyr LoRa API (`zephyr/drivers/lora.h`) using the DT nodelabel `lora`. The static `lora_cfg` struct in `task_sx1262.c` is the single source of truth for modem parameters; shell commands read and write it at runtime.

## Shell Commands

Connect at 115200 baud to USART2 (PA2/PA3):

```
hello                              — greeting self-test
lora status                        — print current modem config
lora config <freq_hz> <sf> <bw> <cr> <preamble> <power_dbm>
                                   — e.g. lora config 868000000 10 125 5 8 14
lora send <text>                   — transmit a packet
lora recv [timeout_ms]             — receive a packet (default 5000 ms)
```

Standard Zephyr kernel shell commands (`kernel threads`, `device list`, etc.) are also available via `CONFIG_KERNEL_SHELL=y` and `CONFIG_DEVICE_SHELL=y`.

## Key Kconfig Options

| Symbol | Purpose |
|---|---|
| `CONFIG_LORA_SX126X=y` | Enables Semtech SX126x driver |
| `CONFIG_LORA_LOG_LEVEL_DBG=y` | Verbose LoRa driver logging |
| `CONFIG_LOG_BACKEND_UART=n` | Log goes to shell, not raw UART |
| `CONFIG_DEBUG_OPTIMIZATIONS=y` | `-Og` debug-friendly build |
