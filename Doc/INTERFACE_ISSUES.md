# Interface Investigation Log

Updated: 2026-07-29

This document records interface constraints and routing changes for the current
hardware configuration.

## Resolved: multi-link SWJ replies

SWJ output now uses a fixed mask for UD/UART2, UA/UART3, UG/UART8, and native
USB CDC. Valid LX IMU frames received on UART1 and addressed to MCU or the
broadcast address, plus replies addressed to SWJ, are mirrored to all host
links instead of selecting one global return path.

SWJ broadcast commands are forwarded only to UART1/LX IMU. They are not sent to
GPS on UART4, PTV7 input on UART5, LORA input on UART7, or another host link.

## UB PTV7 direct-query limitation

UART5/UB is PTV7 input-only for optical-flow (`0x51`) and range (`0x34`) data.
Its parsed data continues to be converted to generic `0x33` and `0x34` frames
for the LX IMU on UART1.

PTv8 frames addressed to the optical-flow device are intentionally dropped.
There is no direct PTv8 upper-computer configuration route to UB; this avoids
the obsolete route that sent those requests to UART4/GPS.
The generic UART5 and UART7 PTv8 device-address routes are also dropped, so
host traffic cannot be forwarded to the PTV7 sensor or LORA receiver.

## UART7 LORA remote receiver

UF/UART7 is a LORA3A22 remote-control receiver at `115200 8N1`. It accepts only
the paired module's transparent 18-byte receive frames; the flight controller
does not configure, pair, or send telemetry through the LORA module.

Mode 3 is now LORA, not CRSF. A valid frame selects it immediately. If no valid
frame arrives for 500 ms, the normal receiver `fail_safe` path is used. This
also covers the transmitter's 120-second idle low-power behavior.

## Debugger VCOM observation

The active Windows debugger device identifies as `ANO CMSIS-DAP VCOM` on COM13.
The UART2 source comment referring to an internal CH343 link is therefore not a
reliable description of the attached debugger hardware.

## UART5 receive maintenance risk

`drvU5DataCheck()` wraps its receive index against `U4RXBUFSIZE`. Both
constants are currently 256, so this has no runtime effect today, but it will
become a defect if either buffer size changes.

UART5 is deliberately wired to a PTV7 byte receiver rather than the PTv8
parser. Its ISR must retain the early `return` required by that parser.

## Evidence limitation

Host TX/RX counters prove that bytes move on the host serial link; they do not
prove that the LX IMU accepted a command or that UB produced a valid PTV7 frame.
Live multi-link and PTV7 payload checks are still required after flashing.

## Live capture status

The connected debugger exposes `ANO CMSIS-DAP` VCOM as COM13. AnoAssistant was
holding COM13 exclusively during the original capture attempt, so no raw device
bytes could be inspected independently.
