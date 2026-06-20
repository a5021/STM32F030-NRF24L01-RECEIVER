# STM32F030-NRF24L01-RECEIVER

[![Build](https://github.com/a5021/STM32F030-NRF24L01-RECEIVER/actions/workflows/build.yml/badge.svg)](https://github.com/a5021/STM32F030-NRF24L01-RECEIVER/actions/workflows/build.yml) [![MCU](https://img.shields.io/badge/MCU-STM32F030F4-00A9E0)]() [![Radio](https://img.shields.io/badge/Radio-nRF24L01-00A9E0)]() [![IDE](https://img.shields.io/badge/IDE-Make%20%7C%20EWARM%20%7C%20MDK--ARM%20%7C%20SES-00A9E0)]()

Wireless nRF24L01+ receiver based on STM32F030F4P6. Listens on RF channel 69, receives variable-length dynamic payloads, and dumps received bytes as hex over USART at 115200 baud. Designed as a companion receiver for the STM32F030F4P6-WIRELESS-MULTISENSOR transmitter.

## Features

- Register-level, bare-metal firmware (no HAL, no CMSIS-DSP)
- nRF24L01+ in Enhanced ShockBurst RX mode, dynamic payload length, NoACK
- 3-byte address width, channel 69
- Clock scaling for active power reduction: 2 MHz during RX, 125 kHz during IRQ wait
- Peripheral clock gating: SPI1 and USART1 disabled during idle, enabled on demand
- Independent watchdog (IWDG) with LSI, kicked every idle loop iteration
- Receives all available payloads from RX FIFO in burst, prints hex bytes + LF
- SWD debug on PA13/PA14 (optional `SWD_DISABLED` build flag)

## Hardware Specification

| Component | Detail |
|-----------|--------|
| MCU | STMicroelectronics STM32F030F4P6 (ARM Cortex-M0, 16 KB Flash, 4 KB RAM) |
| Radio | Nordic nRF24L01+ (SPI1, 1 MHz, channel 69, PRX mode) |
| Debug | SWD on PA13/PA14 |
| Clock | HSI 8 MHz, core HCLK = 2 MHz (Г·4), idle HCLK = 125 kHz (Г·512) |

## Pin Assignment

| Signal | Pin | Peripheral | Notes |
|--------|-----|------------|-------|
| NRF_CE | PA2 | GPIO output | Chip enable, active high |
| NRF_CSN | PA3 | GPIO output | SPI chip select, active low (default HIGH) |
| NRF_IRQ | PA4 | GPIO input | Pull-up, active low (falling edge triggers RX) |
| SPI_SCK | PA5 | SPI1 | 1 MHz |
| SPI_MISO | PA6 | SPI1 |  |
| SPI_MOSI | PA7 | SPI1 |  |
| USART_TX | PA9 | USART1 | 115200 baud, polling |
| USART_RX | PA10 | USART1 | 115200 baud |
| SWCLK | PA14 | AF0 | SWD (can disable with `SWD_DISABLED`) |
| SWDIO | PA13 | AF0 | SWD (can disable with `SWD_DISABLED`) |

## Radio Protocol

The receiver is configured to match the transmitter's packet structure:

| Parameter | Value |
|-----------|-------|
| RF Channel | 69 |
| Air data rate | 2 Mbps |
| Address width | 3 bytes |
| CRC | 2 bytes (CRC16) |
| Auto-ACK | Disabled |
| Dynamic payload | Enabled |
| TX power | Lowest (NRF24L01+ default) |

Received payload bytes are printed as contiguous hex digits via USART, followed by LF (`0x0A`). Each packet ends with a backspace character (`0x08`) sent as a visual separator.

## Firmware Architecture

```
  Cold boot
     |
  +-------------------+
  | Clock init        |
  | (HSI, HPRE_DIV4)  |
  +-------------------+
     |
  +-------------------+
  | GPIO init         |
  | SPI1 init (1 MHz) |
  | USART1 init       |
  | (115200 baud)     |
  +-------------------+
     |
  +-------------------+
  | IWDG init (LSI)   |
  +-------------------+
     |
  +-------------------+
  | nRF24L01+ init:   |
  | - 3-byte addr     |
  | - Ch 69           |
  | - NoAA, DPL       |
  | - PRX, PWR_UP     |
  | - CE HIGH (RX)    |
  +-------------------+
     |
  +-------------------+
  | GATE peripherals  |
  | OFF (SPI1, USART) |
  +-------------------+
     |
  +=================================+
  |     Main loop (idle)            |
  |  HCLK = 125 kHz (HPRE_DIV512)  |
  |  while (IRQ line HIGH) {        |
  |      IWDG->KR = refresh        |
  |  }                              |
  +=================================+
     |                         ^
     | IRQ goes LOW            |
     v                         |
  +-------------------+       |
  | HCLK = 2 MHz      |       |
  | PERIPH CLOCKS ON  |       |
  +-------------------+       |
     |                         |
     v                         |
  +-------------------+       |
  | RX FIFO loop:      |       |
  | do {               |       |
  |   R_RX_PL_WID     |       |
  |   if (pSize > 32)  |       |
  |     FLUSH_RX      |       |
  |   R_RX_PAYLOAD    |       |
  |   print HEX bytes  |       |
  | } while (!RX_EMPTY)|       |
  +-------------------+       |
     |                         |
     v                         |
  +-------------------+       |
  | CLR RX_DR status  |       |
  | PERIPH CLOCKS OFF |       |
  | Send backspace    |-------+
  +-------------------+
```

## Power Management

| State | Core Clock | Peripherals | Current (est.) |
|-------|-----------|-------------|----------------|
| Active (RX) | 2 MHz | SPI1 + USART1 ON | ~15 mA |
| Idle (wait IRQ) | 125 kHz | SPI1 + USART1 OFF | ~2 mA |

Peripheral clocks are fully gated (`RCC->APB2ENR = 0`) during the IRQ wait loop. The core clock is reduced from 2 MHz to 125 kHz via `HPRE_DIV512` to minimise dynamic current while waiting for a packet.

## Watchdog

The IWDG is clocked from LSI (~40 kHz) with prescaler /4 and reload value 1000, yielding a timeout of approximately 100 ms. It is refreshed every iteration of the idle wait loop. If the firmware hangs during RX processing, the watchdog triggers a system reset.

## Getting Started

### Prerequisites

- ARM GCC toolchain (`arm-none-eabi-gcc`)
- GNU Make

### Build & Flash

```sh
make
make program       # ST-LINK (ST-LINK_CLI.exe)
```

### IDEs

| IDE | Path |
|-----|------|
| IAR EWARM | `EWARM/` |
| Keil MDK-ARM (uVision) | `STM32F030-NRF24L01-RECEIVER.uvprojx` |
| SEGGER Embedded Studio | `SES/` |

## Project Structure

```
src/
+-- main.c                   Application entry point and main loop
+-- system_stm32f0xx.c       CMSIS system initialisation
inc/
+-- CMSIS/                   CMSIS-CORE headers
+-- stm32f031x6.h            MCU header
+-- stm32f0xx.h              Family header
+-- system_stm32f0xx.h       System header
EWARM/                       IAR Embedded Workbench project
SES/                         SEGGER Embedded Studio project
STM32F030-NRF24L01-RECEIVER.uvprojx   Keil uVision project
STM32F031F6Px_FLASH.ld       Linker script
startup_stm32f031x6.s        GCC startup
Makefile                     GCC build system
```
