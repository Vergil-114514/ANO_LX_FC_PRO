# Interface Investigation Log

Updated: 2026-07-31

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

## Resolved: PTV7 host display routing

Valid UB `0x51` optical-flow payloads of type 0, 1, or 2 now publish the
`0x31` optical-flow display frame from the LX IMU identity to the SWJ host
links. Type 1 remains the only format that updates the generic velocity input
and can participate in the flight-control loop. Types 0 and 2 are display-only.

PTV7 parser reads use explicit little-endian decoding and reject short payloads
before accessing fields. Valid `0x34` range frames continue to produce generic
distance and velocity frames for UART1/LX IMU.

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

## Resolved: UART and USB transport safety

UART transmit rings now reserve one slot and enqueue a complete frame or drop
it with counters; they no longer overwrite unread bytes and accidentally appear
empty. UART receive overflow is counted and drops the incoming byte. UART3 and
UART8 transmit rings are 2048 bytes, while the UART1 and UART4 receive rings
are 512 bytes. The UART5 wrap check now uses `U5RXBUFSIZE`.

USB CDC uses the same whole-frame queue policy. It no longer clears its busy
flag after a fixed timeout: only endpoint completion releases the next transfer.
Reset, disconnect, and reconfiguration clear pending USB data to avoid sending
stale frames into a new host session.

## Resolved: GPS parser bounds

UART4 accepts only supported UBX messages whose payload and checksum fit inside
the 500-byte receive buffer. NAV-PVT frames must also be large enough for the
fields used by the flight controller. This prevents a malformed length field
from writing past the GPS buffer.

## UART4 Mini5 UWB positioning

UART4/UC is configured as a mutually exclusive navigation input. The current
build selects Mini5 UWB at `115200 8N1`; UBX GPS remains available only when
the configuration is switched back to GPS. Mini5 tag T7 `mc` records require
all four range bits (`MASK=0x0F`), ordered `RSEQ`, and ranges between 100 mm
and 50 m before the four-anchor 2D solver updates its state.

The current wiring assumes T7 TX emits newline-terminated `mc ... a7:A`
records. The Mini5 manual only demonstrates this report through A0 USB, so T7
output must be confirmed directly at `115200 8N1` before flight-controller
integration is accepted.

The configured anchor phase centers are A0 `(-300,-300)`, A1 `(300,-300)`,
A2 `(-300,300)`, and A3 `(300,300)` mm. A valid solution is emitted as the
LX IMU's host-only `0x32` frame with raw UWB X/Y in mm and Z=0. It is not sent
to UART1 and cannot affect the IMU position estimator or flight control.

Raw UWB coordinates remain unfiltered for host display. A separate alpha-beta
filter rejects solutions with residuals above 300 mm or implausible horizontal
steps and is used by the current automatic chase controller.

## Mode 3 car-link

The project includes the fixed car-heading frame parser
`AA 55 01 SEQ FLAGS YAW_X100_LE CRC16_LE`, with CRC16-CCITT-FALSE over bytes
0 through 8. `FLAGS.bit0` is start, `bit1` is abort, `bit2` selects the
dynamic-landing task, and `bit3` latches the car-at-A event. The frame parser
is received on UART8/UG from a SeekFree wireless UART module. UG is fixed at
115200 8N1 and is no longer a PTv8 host-data link; PTv8 telemetry remains on
UART2/UD, UART3/UA, and native USB CDC. The module is used as a receive-only
transparent link, so its TX connects to UG RX, with shared ground and power;
its RX, RTS, and CMD pins are not required by the flight controller.
The car should send a new frame at 20 Hz or faster to satisfy the 200 ms
heading timeout.

Mode 3 is requested automatically at power-on and remains owned by the
automatic task. START is accepted only while the filtered UWB position is
fresh; that same sample is frozen as H relative to A before the car moves.
After Mode 3 is confirmed, the vehicle is locked, filtered UWB, car heading,
PTV7 type-1 optical flow, ToF altitude, and LX IMU 0x07 velocity data are
valid, and emergency-stop is clear, the sequence is unlock -> takeoff to
150 cm -> hover for 3 s -> IMU Mode 2 velocity tracking.

The payload branch descends to 100 cm, holds a valid centered solution for
500 ms, rechecks that solution immediately before releasing the payload servo
once, then returns to cruise height. The dynamic branch descends to the
configurable 15 cm contact threshold, requires current alignment and a
vertical speed within 20 cm/s before every lock attempt, waits 5 seconds, and
repeats the Mode 3 unlock/takeoff sequence.
Both branches return to cruise height and hold their current position in Mode 2
while the car independently returns to A. After `CAR_AT_A`, they use the saved
UWB target to return to H and land. `COMPLETE` remains locked until power is
cycled so the payload servo reinitializes to its clamped state.

LORA mode switching, stick lock/unlock, calibration gestures, and receiver
failsafe return-home, and the CH7 manual payload-servo test are ignored from
power-on. During chase, UWB, heading, optical-flow, or ToF loss retains the
last target; the next valid data updates the target immediately. A low-altitude
payload release, car lock, or H-point landing only advances after current
sensor data is valid; the PTV7 type-1 flow and ToF height frames must each be
newer than 500 ms, and the LX IMU 0x07 velocity frame must also be newer than
500 ms. A car ABORT frame or emergency-stop enters the land failsafe.

If takeoff does not reach the target within 15 seconds, the task enters a land
failsafe. A 90-second task timeout enters the same state. It retries the land
command every 500 ms; once fresh ToF altitude is 30 cm or lower, it retries
locking until the IMU reports locked. Unknown ToF continues landing rather
than resetting the task.

The former UD PTv8 single-motor test frame is disabled and cannot override the
normal physical PWM outputs 2, 4, 6, and 8.

## Evidence limitation

Host TX/RX counters prove that bytes move on the host serial link; they do not
prove that the LX IMU accepted a command or that UB produced a valid PTV7 frame.
Live multi-link and PTV7 payload checks are still required after flashing.

## Live capture status

The connected debugger exposes `ANO CMSIS-DAP` VCOM as COM13. AnoAssistant was
holding COM13 exclusively during the original capture attempt, so no raw device
bytes could be inspected independently.
