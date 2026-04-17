#!/usr/bin/env python3
"""
maze_runner.py — Project NUEVO Maze Navigation Script
======================================================
Runs on the Raspberry Pi (nuevo_ui side) and communicates with the Arduino
firmware over UART using the compact TLV protocol (v4).

Sequence executed:
  1. Move forward 50 cm
  2. Turn left 90°
  3. Move forward 10 cm
  4. Lift elevator 5 cm  (Stepper 1, i.e. stepper index 0)
  5. Close gripper        (Servo 1, i.e. servo channel 0)

Hardware mapping (from config.h / TLV_Payloads.md):
  DC Motor 0  — left  rear wheel  (ODOM_LEFT_MOTOR)
  DC Motor 1  — right rear wheel  (ODOM_RIGHT_MOTOR)
  Stepper 0   — elevator height control
  Servo ch 0  — gripper open/close

Protocol framing (COMMUNICATION_PROTOCOL.md §3.2):
  Frame header: magic[4] | numTotalBytes:u16 | crc16:u16 |
                deviceId:u8 | frameNum:u8 | numTlvs:u8 | flags:u8
  TLV entry:   tlvType:u8 | tlvLen:u8 | payload[tlvLen]

TLV type IDs (TLV_Payloads.md):
  SYS_HEARTBEAT   =  1   (↓ RPi → Arduino, keep-alive)
  SYS_CMD         =  3   (↓ START / STOP / RESET / ESTOP)
  SYS_STATE       =  2   (↑ Arduino → RPi, streamed)
  DC_ENABLE       = 16   (↓)
  DC_SET_POSITION = 17   (↓)
  STEP_ENABLE     = 32   (↓)
  STEP_MOVE       = 33   (↓)
  SERVO_ENABLE    = 48   (↓)
  SERVO_SET       = 49   (↓, single-channel variant)

Usage:
  python3 maze_runner.py [--port /dev/ttyAMA0] [--dry-run]
"""

import argparse
import math
import struct
import time
import threading
import sys

# ---------------------------------------------------------------------------
# Try to import pyserial; fall back gracefully for offline testing.
# ---------------------------------------------------------------------------
try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("[WARN] pyserial not found — running in DRY-RUN mode automatically.")

# ===========================================================================
# ROBOT GEOMETRY & HARDWARE CONSTANTS  (mirror config.h)
# ===========================================================================

WHEEL_DIAMETER_MM   = 74.0          # mm  — config.h WHEEL_DIAMETER_MM
WHEEL_BASE_MM       = 333.0         # mm  — config.h WHEEL_BASE_MM
ENCODER_PPR         = 1440          # ticks/rev at 4x — config.h ENCODER_PPR
ENCODER_MODE        = 4             # 4x counting

# Ticks per mm of wheel travel
TICKS_PER_MM = (ENCODER_PPR * ENCODER_MODE) / (math.pi * WHEEL_DIAMETER_MM)
# Note: ENCODER_PPR is already the 4x value per config.h comment
# ("Pulses per revolution (manufacturer spec, for encoder_4x)")
# so effective ticks/rev = ENCODER_PPR = 1440, no further ×4 needed.
TICKS_PER_MM = ENCODER_PPR / (math.pi * WHEEL_DIAMETER_MM)

# Stepper elevator — adjust STEPS_PER_MM to match your lead-screw pitch.
# Example: 200 steps/rev motor, 1/16 microstepping, 2 mm/rev lead screw
#   → (200 × 16) / 2 = 1600 steps/mm
# Override at runtime with --steps-per-mm if your hardware differs.
DEFAULT_STEPPER_STEPS_PER_MM = 100  # ← tune to your lead-screw

# Servo pulse widths (µs) for gripper on channel 0
SERVO_OPEN_US  = 2000   # fully open
SERVO_CLOSE_US = 1000   # fully closed

# Drive speed limits (ticks/sec) — keep conservatively slow for maze work
DRIVE_MAX_VEL_TICKS = int(TICKS_PER_MM * 80)   # ~80 mm/s
TURN_MAX_VEL_TICKS  = int(TICKS_PER_MM * 40)   # slower for turns

# Motor IDs (0-based, matching config.h)
LEFT_MOTOR_ID  = 0
RIGHT_MOTOR_ID = 1
STEPPER_ID     = 0   # Stepper 1 in 1-based user terms → index 0
SERVO_CH       = 0   # Servo 1  in 1-based user terms → channel 0

# UART settings
BAUD_RATE   = 200000
DEVICE_ID   = 0x01   # Arduino device ID
FRAME_MAGIC = b'\xDE\xAD\xBE\xEF'

# Heartbeat interval while waiting for motion to complete
HEARTBEAT_INTERVAL_S = 0.1   # 10 Hz matches firmware expectation

# How long to poll for motion completion before timing out
MOTION_TIMEOUT_S = 30.0

# ===========================================================================
# CRC-16 CCITT (poly 0x1021, init 0xFFFF) — matches firmware
# ===========================================================================

def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc

# ===========================================================================
# TLV FRAME BUILDER
# ===========================================================================

class TLVFrameBuilder:
    """Builds a single compact TLV frame containing one or more TLV entries."""

    def __init__(self, frame_num: int = 0):
        self._tlvs: list[bytes] = []
        self._frame_num = frame_num & 0xFF

    def add_tlv(self, tlv_type: int, payload: bytes) -> "TLVFrameBuilder":
        """Append one TLV entry (type byte + length byte + payload)."""
        if len(payload) > 255:
            raise ValueError(f"TLV payload too large: {len(payload)} bytes")
        self._tlvs.append(bytes([tlv_type & 0xFF, len(payload)]) + payload)
        return self

    def build(self) -> bytes:
        """Serialise the complete frame including header and CRC."""
        tlv_blob = b"".join(self._tlvs)
        # Header fields after magic + length + crc:
        #   deviceId(1) frameNum(1) numTlvs(1) flags(1)
        header_tail = struct.pack(
            "<BBBB",
            DEVICE_ID,
            self._frame_num,
            len(self._tlvs),
            0,          # flags reserved
        )
        # numTotalBytes includes magic(4) + numTotalBytes(2) + crc(2) + header_tail(4) + tlvs
        # i.e. the entire frame byte count
        payload_body = header_tail + tlv_blob
        num_total = 4 + 2 + 2 + len(payload_body)   # magic + u16 + u16 + rest
        size_bytes = struct.pack("<H", num_total)
        crc_input  = size_bytes + payload_body
        crc_val    = crc16_ccitt(crc_input)
        crc_bytes  = struct.pack("<H", crc_val)
        return FRAME_MAGIC + size_bytes + crc_bytes + payload_body

# ===========================================================================
# PAYLOAD BUILDERS  (matching TLV_Payloads.h packed structs, little-endian)
# ===========================================================================

def payload_heartbeat(timestamp_ms: int) -> bytes:
    """PayloadHeartbeat — 5 bytes: u32 timestamp, u8 flags."""
    return struct.pack("<IB", timestamp_ms & 0xFFFFFFFF, 0)

def payload_sys_cmd(command: int) -> bytes:
    """PayloadSysCmd — 4 bytes: u8 command, u8[3] reserved."""
    return struct.pack("<BBBB", command, 0, 0, 0)

SYS_CMD_START = 1
SYS_CMD_STOP  = 2

def payload_dc_enable(motor_id: int, mode: int) -> bytes:
    """PayloadDCEnable — 4 bytes: u8 motorId, u8 mode, u8[2] reserved."""
    return struct.pack("<BBBB", motor_id, mode, 0, 0)

DC_MODE_DISABLED = 0
DC_MODE_POSITION = 1

def payload_dc_set_position(motor_id: int, target_ticks: int, max_vel_ticks: int) -> bytes:
    """PayloadDCSetPosition — 12 bytes: u8 motorId, u8[3] reserved, i32 targetTicks, i32 maxVelTicks."""
    return struct.pack("<BBBBii", motor_id, 0, 0, 0, target_ticks, max_vel_ticks)

def payload_step_enable(stepper_id: int, enable: int) -> bytes:
    """PayloadStepEnable — 4 bytes: u8 stepperId, u8 enable, u8[2] reserved."""
    return struct.pack("<BBBB", stepper_id, enable, 0, 0)

STEP_MOVE_RELATIVE = 1
STEP_MOVE_ABSOLUTE = 0

def payload_step_move(stepper_id: int, target: int, move_type: int = STEP_MOVE_RELATIVE) -> bytes:
    """PayloadStepMove — 8 bytes: u8 stepperId, u8 moveType, u8[2] reserved, i32 target."""
    return struct.pack("<BBBBi", stepper_id, move_type, 0, 0, target)

def payload_servo_enable(channel: int, enable: int) -> bytes:
    """PayloadServoEnable — 4 bytes: u8 channel, u8 enable, u8[2] reserved."""
    return struct.pack("<BBBB", channel, enable, 0, 0)

def payload_servo_set_single(channel: int, pulse_us: int) -> bytes:
    """PayloadServoSetSingle — 4 bytes: u8 channel, u8 count=1, u16 pulseUs."""
    return struct.pack("<BBH", channel, 1, pulse_us)

# TLV type IDs
TLV_SYS_HEARTBEAT   =  1
TLV_SYS_CMD         =  3
TLV_DC_ENABLE       = 16
TLV_DC_SET_POSITION = 17
TLV_STEP_ENABLE     = 32
TLV_STEP_MOVE       = 33
TLV_SERVO_ENABLE    = 48
TLV_SERVO_SET       = 49

# ===========================================================================
# SERIAL INTERFACE / DRY-RUN WRAPPER
# ===========================================================================

class RobotSerial:
    """
    Thin wrapper around a pyserial port.  In dry-run mode it just prints
    what would be sent and returns dummy data for reads.
    """

    def __init__(self, port: str, dry_run: bool = False):
        self._dry_run = dry_run or not SERIAL_AVAILABLE
        self._frame_num = 0
        self._start_time = time.monotonic()
        if not self._dry_run:
            self._ser = serial.Serial(port, BAUD_RATE, timeout=0.05)
            print(f"[SERIAL] Opened {port} at {BAUD_RATE} baud")
        else:
            print("[DRY-RUN] Serial not opened — frames will be printed as hex.")

    def _next_frame_num(self) -> int:
        n = self._frame_num
        self._frame_num = (self._frame_num + 1) & 0xFF
        return n

    def _now_ms(self) -> int:
        return int((time.monotonic() - self._start_time) * 1000)

    def send_frame(self, builder: TLVFrameBuilder) -> None:
        raw = builder.build()
        if self._dry_run:
            print(f"  [TX] {raw.hex(' ').upper()}")
        else:
            self._ser.write(raw)

    def send_heartbeat(self) -> None:
        fb = TLVFrameBuilder(self._next_frame_num())
        fb.add_tlv(TLV_SYS_HEARTBEAT, payload_heartbeat(self._now_ms()))
        self.send_frame(fb)

    def close(self):
        if not self._dry_run:
            self._ser.close()

# ===========================================================================
# MOTION HELPERS
# ===========================================================================

def ticks_for_distance(distance_mm: float) -> int:
    """Convert a linear distance (mm) to encoder ticks."""
    return int(round(distance_mm * TICKS_PER_MM))

def ticks_for_turn(angle_deg: float) -> int:
    """
    Convert a turn angle to the number of ticks each wheel must travel.
    For a differential drive:
        arc = (angle_rad / (2*pi)) * pi * wheelBase = angle_rad * wheelBase / 2
    Returns ticks for the outer wheel; inner wheel gets the negative.
    """
    angle_rad = math.radians(angle_deg)
    arc_mm = angle_rad * WHEEL_BASE_MM / 2.0
    return int(round(arc_mm * TICKS_PER_MM))

def steps_for_lift(distance_mm: float, steps_per_mm: float) -> int:
    """Convert a lift height (mm) to stepper steps."""
    return int(round(distance_mm * steps_per_mm))

# ===========================================================================
# MAZE RUNNER
# ===========================================================================

class MazeRunner:
    """
    Orchestrates the maze sequence by sending TLV commands to the Arduino.

    The sequence:
      Step 1 — Move forward 50 cm
      Step 2 — Turn left 90°
      Step 3 — Move forward 10 cm
      Step 4 — Lift elevator 5 cm
      Step 5 — Close gripper
    """

    def __init__(self, bot: RobotSerial, steps_per_mm: float, dry_run: bool):
        self._bot       = bot
        self._spm       = steps_per_mm
        self._dry_run   = dry_run
        self._frame_num = 0

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _fn(self) -> int:
        n = self._frame_num
        self._frame_num = (self._frame_num + 1) & 0xFF
        return n

    def _send(self, *tlvs: tuple[int, bytes]) -> None:
        """Build and send a frame with one or more (type, payload) pairs."""
        fb = TLVFrameBuilder(self._fn())
        for tlv_type, payload in tlvs:
            fb.add_tlv(tlv_type, payload)
        self._bot.send_frame(fb)

    def _wait(self, seconds: float, reason: str) -> None:
        """
        Wait for 'seconds', sending heartbeats every HEARTBEAT_INTERVAL_S.

        In a full implementation this would instead poll DC_STATE_ALL /
        STEP_STATE_ALL telemetry to detect when motion is actually complete.
        Time-based waiting is used here because reading back state requires
        parsing incoming TLV frames — that infrastructure lives in the existing
        nuevo_bridge and is intentionally not duplicated in this file.

        To integrate with the bridge, replace _wait() calls with an async
        callback that waits for the relevant state field to signal idle.
        """
        print(f"  [WAIT] {reason} ({seconds:.1f}s) …")
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self._bot.send_heartbeat()
            time.sleep(HEARTBEAT_INTERVAL_S)

    def _enable_drive_motors(self, enable: bool) -> None:
        mode = DC_MODE_POSITION if enable else DC_MODE_DISABLED
        self._send(
            (TLV_DC_ENABLE, payload_dc_enable(LEFT_MOTOR_ID,  mode)),
            (TLV_DC_ENABLE, payload_dc_enable(RIGHT_MOTOR_ID, mode)),
        )

    def _move_straight(self, distance_mm: float) -> None:
        """Drive both wheels the same number of ticks (forward = positive)."""
        ticks = ticks_for_distance(distance_mm)
        print(f"  [DC] Straight {distance_mm:.0f} mm → {ticks} ticks each")
        self._send(
            (TLV_DC_SET_POSITION, payload_dc_set_position(LEFT_MOTOR_ID,  ticks, DRIVE_MAX_VEL_TICKS)),
            (TLV_DC_SET_POSITION, payload_dc_set_position(RIGHT_MOTOR_ID, ticks, DRIVE_MAX_VEL_TICKS)),
        )

    def _turn_left(self, angle_deg: float) -> None:
        """
        Left turn: left wheel moves backward, right wheel moves forward.
        Positions are relative to the current encoder zero — we reset
        position before each motion segment via DC_RESET_POSITION (type 24),
        which is already handled by enabling the motor in POSITION mode
        with a fresh target.  For simplicity we issue relative targets
        from the current position by tracking accumulated ticks.
        """
        ticks = ticks_for_turn(angle_deg)
        print(f"  [DC] Turn left {angle_deg:.0f}° → ±{ticks} ticks")
        # Left wheel reverses, right wheel advances
        self._send(
            (TLV_DC_SET_POSITION, payload_dc_set_position(LEFT_MOTOR_ID,  -ticks, TURN_MAX_VEL_TICKS)),
            (TLV_DC_SET_POSITION, payload_dc_set_position(RIGHT_MOTOR_ID,  ticks, TURN_MAX_VEL_TICKS)),
        )

    def _lift_elevator(self, height_mm: float) -> None:
        """Move stepper up by height_mm using a relative STEP_MOVE."""
        steps = steps_for_lift(height_mm, self._spm)
        print(f"  [STEP] Lift {height_mm:.0f} mm → {steps} steps (relative)")
        # Enable stepper first (in case it was disabled)
        self._send(
            (TLV_STEP_ENABLE, payload_step_enable(STEPPER_ID, 1)),
        )
        time.sleep(0.05)   # let enable settle
        self._send(
            (TLV_STEP_MOVE, payload_step_move(STEPPER_ID, steps, STEP_MOVE_RELATIVE)),
        )

    def _close_gripper(self) -> None:
        """Command servo channel 0 to the closed pulse width."""
        print(f"  [SERVO] Close gripper (ch {SERVO_CH} → {SERVO_CLOSE_US} µs)")
        self._send(
            (TLV_SERVO_ENABLE, payload_servo_enable(SERVO_CH, 1)),
        )
        time.sleep(0.05)
        self._send(
            (TLV_SERVO_SET, payload_servo_set_single(SERVO_CH, SERVO_CLOSE_US)),
        )

    # ------------------------------------------------------------------
    # Top-level sequence
    # ------------------------------------------------------------------

    def run(self) -> None:
        print("=" * 60)
        print("NUEVO Maze Runner — starting sequence")
        print(f"  TICKS_PER_MM          : {TICKS_PER_MM:.3f}")
        print(f"  STEPPER STEPS_PER_MM  : {self._spm}")
        print(f"  DRIVE_MAX_VEL_TICKS/s : {DRIVE_MAX_VEL_TICKS}")
        print("=" * 60)

        # ── Bootstrap / startup ────────────────────────────────────────
        print("\n[0] Bootstrap: sending SYS_CMD START and initial heartbeat")
        self._send(
            (TLV_SYS_CMD, payload_sys_cmd(SYS_CMD_START)),
        )
        time.sleep(0.2)
        self._bot.send_heartbeat()
        time.sleep(0.1)

        # ── Enable drive motors in POSITION mode ───────────────────────
        print("\n[INIT] Enabling DC motors 0 & 1 in POSITION mode")
        self._enable_drive_motors(True)
        time.sleep(0.1)

        # ── Step 1: Move forward 50 cm ─────────────────────────────────
        print("\n[STEP 1] Move forward 500 mm")
        self._move_straight(500.0)
        # Estimate travel time: 500 mm / 80 mm·s⁻¹ ≈ 6.25 s + margin
        travel_time = (500.0 / 80.0) + 1.5
        self._wait(travel_time, "forward 500 mm")

        # ── Step 2: Turn left 90° ──────────────────────────────────────
        print("\n[STEP 2] Turn left 90°")
        self._turn_left(90.0)
        # Arc per wheel ≈ π/2 × 333/2 = 261 mm; at 40 mm/s ≈ 6.5 s + margin
        turn_arc_mm = math.radians(90.0) * WHEEL_BASE_MM / 2.0
        turn_time = (turn_arc_mm / 40.0) + 1.5
        self._wait(turn_time, "90° left turn")

        # ── Step 3: Move forward 10 cm ─────────────────────────────────
        print("\n[STEP 3] Move forward 100 mm")
        self._move_straight(100.0)
        travel_time2 = (100.0 / 80.0) + 1.0
        self._wait(travel_time2, "forward 100 mm")

        # ── Step 4: Disable drive, lift elevator 5 cm ──────────────────
        print("\n[STEP 4] Lift elevator 50 mm")
        self._enable_drive_motors(False)   # stop DC motors before manipulator ops
        time.sleep(0.1)
        self._lift_elevator(50.0)
        # Estimate: 50 mm × steps_per_mm steps at firmware default speed
        # Conservative: allow 5 s
        self._wait(5.0, "elevator lift 50 mm")

        # ── Step 5: Close gripper ──────────────────────────────────────
        print("\n[STEP 5] Close gripper")
        self._close_gripper()
        self._wait(1.5, "gripper close")

        # ── Done ───────────────────────────────────────────────────────
        print("\n[DONE] Maze sequence complete.")
        print("       Sending SYS_CMD STOP.")
        self._send(
            (TLV_SYS_CMD, payload_sys_cmd(SYS_CMD_STOP)),
        )
        self._bot.send_heartbeat()

# ===========================================================================
# CLI ENTRY POINT
# ===========================================================================

def main() -> None:
    parser = argparse.ArgumentParser(
        description="NUEVO Maze Runner — sends TLV commands to the Arduino over UART."
    )
    parser.add_argument(
        "--port",
        default="/dev/ttyAMA0",
        help="Serial port connected to Arduino Serial2 (default: /dev/ttyAMA0)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print TLV frames as hex without opening a serial port.",
    )
    parser.add_argument(
        "--steps-per-mm",
        type=float,
        default=DEFAULT_STEPPER_STEPS_PER_MM,
        help=(
            f"Stepper steps per mm of elevator travel "
            f"(default: {DEFAULT_STEPPER_STEPS_PER_MM}). "
            "Tune to match your lead-screw pitch and microstepping setting."
        ),
    )
    args = parser.parse_args()

    bot = RobotSerial(port=args.port, dry_run=args.dry_run)
    try:
        runner = MazeRunner(bot, steps_per_mm=args.steps_per_mm, dry_run=args.dry_run)
        runner.run()
    except KeyboardInterrupt:
        print("\n[INTERRUPTED] Sending ESTOP.")
        fb = TLVFrameBuilder(0)
        fb.add_tlv(TLV_SYS_CMD, payload_sys_cmd(4))   # ESTOP = 4
        bot.send_frame(fb)
    finally:
        bot.close()


if __name__ == "__main__":
    main()