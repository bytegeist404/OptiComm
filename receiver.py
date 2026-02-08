import configparser
import math
import os
import struct
import time
from pathlib import Path

import serial

PORT_RECEIVER = "/dev/ttyACM1"
BAUDRATE = 115200
SYNC_BYTE = 0xAA

BIT_DURATION = 0.05
BIT_DURATION_US = int(BIT_DURATION * 1_000_000)
PREAMBLE = [int(c) for c in "1010101011100010"]

STREAM_FILE = Path(".stream")
RECEIVED_DIR = Path("received")

RECEIVED_DIR.mkdir(parents=True, exist_ok=True)
RECORD = struct.Struct("<IHffBB")


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


def bits_to_uint(bits):
    value = 0
    for b in bits:
        value = (value << 1) | int(b)
    return value


def read_sample(tr: serial.Serial):
    while True:
        b = tr.read(1)
        if not b:
            return None, None
        if b[0] == SYNC_BYTE:
            break

    data = tr.read(6)
    if len(data) != 6:
        return None, None

    ts, raw = struct.unpack("<IH", data)

    if raw > 1023:
        return None, None

    return ts, raw


def main():
    rx = serial.Serial(PORT_RECEIVER, BAUDRATE, timeout=0)
    time.sleep(1)

    state = None

    raw = None
    raw_ema = None
    RAW_EMA_TAU = 0.1 * BIT_DURATION_US
    DELTA = 0.1

    noise_ema = 0
    NOISE_EMA_TAU = 0.05 * BIT_DURATION_US

    last_transition_ts = None
    next_sample_ts = None
    preamble_found = None
    payload_length = None
    bit_history = None

    def reset():
        nonlocal state, last_transition_ts, next_sample_ts, preamble_found, payload_length, bit_history
        state = None

        last_transition_ts = None
        next_sample_ts = None
        preamble_found = False
        payload_length = None
        bit_history = []
        print("reset")

    reset()

    if os.path.isfile(STREAM_FILE):
        os.remove(STREAM_FILE)
    try:
        with open(STREAM_FILE, "ab") as f:
            now = None
            while True:
                prev_now = now
                now, raw = read_sample(rx)

                if raw is None or prev_now is None:
                    continue

                dt = (now - prev_now) & 0xFFFFFFFF

                if raw_ema is None:
                    raw_ema = raw
                raw_ema_alpha = 1 - math.exp(-dt / RAW_EMA_TAU)
                raw_ema += raw_ema_alpha * (raw - raw_ema)

                slope = (raw - raw_ema) / DELTA
                slope = (1 if slope > 0 else -1) * max(0, abs(slope) - 50)

                noise_ema_alpha = 1 - math.exp(-dt / NOISE_EMA_TAU)
                noise_ema += noise_ema_alpha * (abs(slope) - noise_ema)

                transition_flag = False
                if abs(slope) > noise_ema * 3 and (
                    last_transition_ts is None
                    or now - last_transition_ts > 0.5 * BIT_DURATION_US
                ):
                    last_transition_ts = now
                    transition_flag = True
                    state = int(slope > 0)
                    if state:
                        next_sample_ts = last_transition_ts + BIT_DURATION_US * 0.5

                if last_transition_ts is not None:
                    if (
                        not preamble_found
                        and now - last_transition_ts > len(PREAMBLE) * BIT_DURATION_US
                    ):
                        print("Preamble transition timeout")
                        reset()
                        continue
                f.write(
                    RECORD.pack(
                        now,
                        raw,
                        slope,
                        noise_ema,
                        transition_flag,
                        int(bool(next_sample_ts is not None and now >= next_sample_ts))
                        << 1,
                    )
                )
                f.flush()

                if next_sample_ts is not None and now >= next_sample_ts:
                    if state is None:
                        continue

                    next_sample_ts += BIT_DURATION_US
                    bit_history.append(int(state))

                    if not preamble_found:
                        bit_history = bit_history[-len(PREAMBLE) :]
                        if bit_history == PREAMBLE:
                            print("Detected start marker")
                            preamble_found = True
                            bit_history = []
                        continue

                    if payload_length is None:
                        if len(bit_history) < 32:
                            continue

                        payload_length = bits_to_uint(bit_history[:32])
                        bit_history = []
                        if payload_length <= 0:
                            print(f"Invalid payload length: {payload_length}, aborting")
                            reset()
                        else:
                            print(
                                f"Received Payload length: {payload_length}, eta: {payload_length * BIT_DURATION}"
                            )
                        continue

                    if len(bit_history) % int(5 * 1_000_000 / BIT_DURATION_US) == 0:
                        print(
                            f"Received {len(bit_history)} / {payload_length} bits... (eta: {len(bit_history) * BIT_DURATION} / {payload_length * BIT_DURATION}s)"
                        )

                    if len(bit_history) > payload_length:
                        data = bits_to_bytes(bit_history[:payload_length])
                        filename = RECEIVED_DIR / f"{int(time.time())}.bin"
                        with open(filename, "wb") as fp:
                            fp.write(data)
                        print(f"Wrote received data to: {filename}")
                        reset()
                        continue

    except KeyboardInterrupt:
        print("Keyboard Interrupt - Stopping.")
    finally:
        rx.close()
        f.close()


if __name__ == "__main__":
    main()
