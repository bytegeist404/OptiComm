#!/usr/bin/env python3
"""
test_error_rate.py

Sendet zufällige Bit-Frames mit dem vorhandenen transmitter_thread und misst
Empfangsmetriken, indem es auf erzeugte Dateien im Verzeichnis `received/`
wartet. Protokolliert Ergebnisse in CSV (test_results.csv) und in einer kurzen Logdatei.
"""

import csv
import enum
import os
import queue
import random
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Deque, List, Optional

import serial

PORT_TRANSMITTER = "/dev/ttyACM0"
BAUDRATE = 115200

BIT_DURATION = 0.05
PREAMBLE = [int(c) for c in "1010101010101010101010101010101011100010"]


class TxCommand(enum.Enum):
    SET_MODE = enum.auto()
    STOP = enum.auto()


@dataclass
class Command:
    type: TxCommand
    value: object = None
    payload: Optional[List[int]] = None


def encode_bytes_to_bits(data: bytes) -> List[int]:
    """Big-endian bit order per byte (b7..b0)"""
    bits: List[int] = []
    for b in data:
        for i in range(7, -1, -1):
            bits.append((b >> i) & 1)
    return bits


def transmitter_thread(tx: serial.Serial, cmd_q: "queue.Queue[Command]") -> None:
    mode = "off"
    bit_q: Deque[int] = deque()
    next_bit_ts: float = time.monotonic()

    try:
        while True:
            try:
                cmd: Optional[Command] = cmd_q.get_nowait()
            except queue.Empty:
                cmd = None

            if cmd is not None:
                if cmd.type == TxCommand.SET_MODE:
                    mode = str(cmd.value)
                    bit_q.clear()
                    next_bit_ts = time.monotonic()

                    if mode == "msg":
                        payload_bits = cmd.payload or []
                        length_bits = encode_bytes_to_bits(
                            len(payload_bits).to_bytes(4, "big")
                        )
                        full_bits = PREAMBLE + length_bits + payload_bits
                        bit_q = deque(full_bits)

                    if mode == "bits":
                        bit_q = deque(cmd.payload or [])

                    if mode in ("0", "1"):
                        bit_q.append(1 if mode == "1" else 0)

                elif cmd.type == TxCommand.STOP:
                    break

            now = time.monotonic()
            if now < next_bit_ts:
                time.sleep(0.001)
                continue

            if not bit_q:
                if mode == "msg":
                    cmd_q.put(Command(TxCommand.SET_MODE, "0"))
                time.sleep(0.001)
                continue

            bit = bit_q.popleft()
            try:
                tx.write(b"\x01" if bit else b"\x00")
            except Exception as e:
                print(f"Serial write error: {e}")
                continue

            next_bit_ts = now + BIT_DURATION

    except KeyboardInterrupt:
        print("Keyboard Interrupt - Stopping")

    try:
        tx.write(b"\x00")
    except Exception:
        pass


def bits_to_bytes(bits):
    n = (len(bits) // 8) * 8
    bits = bits[:n]
    out = bytearray()
    for i in range(0, n, 8):
        byte = 0
        for b in bits[i : i + 8]:
            byte = (byte << 1) | int(b)
        out.append(byte)
    return bytes(out)


# ---------- Konfiguration ----------
NUM_FRAMES = None  # wie viele Frames senden (oder setze None für Endlosschleife)
MIN_BITS = 10  # min payload Länge in Bits
MAX_BITS = 2000  # max payload Länge in Bits (für Testlauf nicht zu groß)
RECEIVED_DIR = Path("received")
RESULT_CSV = Path("test_results.csv")
LOG_FILE = Path("test.log")

# Timeout: erwartete Dauer der Übertragung = (preamble + 32 + payload_bits) * BIT_DURATION
# Wir erlauben ein Vielfaches dieses Wertes als Timeout (z. B. 2x) plus fester Slack
FRAME_TIMEOUT_FACTOR = 2.0
FRAME_TIMEOUT_SLACK = 5.0  # Sekunden extra


# ---------- Hilfsfunktionen ----------
def ensure_clean_received_dir():
    RECEIVED_DIR.mkdir(exist_ok=True)
    # entferne nur dateien darin
    for child in RECEIVED_DIR.iterdir():
        if child.is_file():
            child.unlink()


def wait_for_received_file(timeout: float) -> Path:
    """
    Wartet auf das erste *.bin file im received dir. Liefert Path oder None bei Timeout.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        files = list(RECEIVED_DIR.glob("*.bin"))
        if files:
            # wähle die älteste Datei (nach mtime)
            files.sort(key=lambda p: p.stat().st_mtime)
            return files[0]
        time.sleep(0.05)
    return None


def bits_to_bytes_local(bits: List[int]) -> bytes:
    """Wir brauchen dieselbe Konvention wie receiver.bits_to_bytes (MSB first per byte)."""
    # Re-implement fallback if receiver.bits_to_bytes not available
    n = (len(bits) // 8) * 8
    bits = bits[:n]
    out = bytearray()
    for i in range(0, n, 8):
        byte = 0
        for b in bits[i : i + 8]:
            byte = (byte << 1) | int(b)
        out.append(byte)
    return bytes(out)


def bytes_to_bits_be(data: bytes) -> List[int]:
    """Big-endian bit order per byte (b7..b0) - entspricht transmitter.encode_bytes_to_bits."""
    bits = []
    for b in data:
        for i in range(7, -1, -1):
            bits.append((b >> i) & 1)
    return bits


# ---------- Testlauf ----------
def test_error_rate(num_frames: int = NUM_FRAMES):
    cmd_q: queue.Queue[Command] = queue.Queue()

    # Setup serial port to transmitter Arduino
    tx = serial.Serial(PORT_TRANSMITTER, BAUDRATE, timeout=0)
    time.sleep(1)

    # Start transmitter thread that sends bits to Arduino
    t = threading.Thread(target=transmitter_thread, args=(tx, cmd_q), daemon=True)
    t.start()

    ensure_clean_received_dir()

    # CSV header
    with open(RESULT_CSV, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(
            [
                "frame_id",
                "payload_bits",
                "expected_bytes",
                "received_bytes",
                "sent_duration_s",
                "wait_time_s",
                "frame_success",
                "bit_errors",
                "ber",  # bit error rate (errors / expected_bits)
            ]
        )

    frame_count = 0
    total_bits_sent = 0
    total_bit_errors = 0
    total_frames_success = 0
    total_wait_time = 0.0
    total_transmit_time = 0.0
    time_start_all = time.monotonic()

    try:
        while True:
            if num_frames is not None and frame_count >= num_frames:
                break

            frame_count += 1
            # Random payload bits
            payload_length = random.randint(MIN_BITS, MAX_BITS)
            payload_bits = [
                1 if random.random() > 0.5 else 0 for _ in range(payload_length)
            ]

            # Sender erwartet payload bits for 'msg' mode already
            cmd_q.put(Command(TxCommand.SET_MODE, "msg", payload=payload_bits))

            # Compute expected transmission duration for timeout
            PREAMBLE_LEN = 16  # same as PREAMBLE in transmitter.py
            expected_sec = (PREAMBLE_LEN + 32 + payload_length) * BIT_DURATION
            timeout = expected_sec * FRAME_TIMEOUT_FACTOR + FRAME_TIMEOUT_SLACK

            t_sent = time.monotonic()
            # Wait for file to appear
            rec_file = wait_for_received_file(timeout=timeout)
            wait_time = time.monotonic() - t_sent

            if rec_file is None:
                # timeout
                with open(LOG_FILE, "a") as lf:
                    lf.write(
                        f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] Frame {frame_count}: TIMEOUT (no file in {timeout:.1f}s)\n"
                    )
                # no file -> this frame counted as failure
                total_transmit_time += expected_sec
                total_wait_time += wait_time
                # continue to next frame
                continue

            # read received bytes
            try:
                with open(rec_file, "rb") as f:
                    received_data = f.read()
            except Exception as e:
                with open(LOG_FILE, "a") as lf:
                    lf.write(
                        f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] Frame {frame_count}: error reading file {rec_file}: {e}\n"
                    )
                rec_file.unlink(missing_ok=True)
                continue

            # Clean up file after reading
            rec_file.unlink(missing_ok=True)

            # The receiver writes full bytes converted from bits; compute expected bytes using same conversion
            expected_bytes = bits_to_bytes(payload_bits[:payload_length])
            expected_bits_for_compare = bytes_to_bits_be(expected_bytes)
            received_bits = bytes_to_bits_be(received_data)

            # Compare bits: count mismatches and missing bits
            compare_len = len(expected_bits_for_compare)
            received_len = len(received_bits)
            min_len = min(compare_len, received_len)
            mismatches = sum(
                1
                for i in range(min_len)
                if expected_bits_for_compare[i] != received_bits[i]
            )
            missing_bits = max(0, compare_len - received_len)
            bit_errors = mismatches + missing_bits
            ber = bit_errors / compare_len if compare_len > 0 else 0.0

            frame_success = bit_errors == 0 and received_len == compare_len

            # Logging metrics
            with open(RESULT_CSV, "a", newline="") as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(
                    [
                        frame_count,
                        payload_length,
                        len(expected_bytes),
                        len(received_data),
                        expected_sec,
                        wait_time,
                        int(frame_success),
                        bit_errors,
                        f"{ber:.6f}",
                    ]
                )

            with open(LOG_FILE, "a") as lf:
                lf.write(
                    f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] Frame {frame_count}: sent_bits={payload_length} expected_bytes={len(expected_bytes)} received_bytes={len(received_data)} errors={bit_errors} ber={ber:.6f} wait_s={wait_time:.3f}\n"
                )

            # Update aggregates
            total_bits_sent += compare_len
            total_bit_errors += bit_errors
            total_frames_success += 1 if frame_success else 0
            total_wait_time += wait_time
            total_transmit_time += expected_sec

            # Small cooldown between frames
            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\nKeyboard interrupt received, stopping transmitter")
    finally:
        # Send stop command to transmitter thread and cleanup
        try:
            cmd_q.put(Command(TxCommand.STOP))
            t.join(timeout=2.0)
        except Exception:
            pass
        try:
            tx.close()
        except Exception:
            pass

    # Summary
    duration_all = time.monotonic() - time_start_all
    total_frames = frame_count
    frame_success_rate = (
        total_frames_success / total_frames if total_frames > 0 else 0.0
    )
    overall_ber = total_bit_errors / total_bits_sent if total_bits_sent > 0 else 0.0
    avg_latency = total_wait_time / total_frames if total_frames > 0 else 0.0
    effective_throughput_bps = (
        (total_bits_sent - total_bit_errors) / total_transmit_time
        if total_transmit_time > 0
        else 0.0
    )

    summary = {
        "frames_sent": total_frames,
        "frames_success": total_frames_success,
        "frame_success_rate": frame_success_rate,
        "total_bits_sent": total_bits_sent,
        "total_bit_errors": total_bit_errors,
        "overall_ber": overall_ber,
        "avg_wait_time_s": avg_latency,
        "effective_throughput_bps": effective_throughput_bps,
        "total_wall_time_s": duration_all,
    }

    print("\n=== Test Summary ===")
    for k, v in summary.items():
        print(f"{k:30s}: {v}")

    with open(LOG_FILE, "a") as lf:
        lf.write(f"\n=== SUMMARY ===\n")
        for k, v in summary.items():
            lf.write(f"{k}: {v}\n")

    return summary


if __name__ == "__main__":
    # Run the test (adjust NUM_FRAMES at top as desired)
    test_error_rate()
