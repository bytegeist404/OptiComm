import configparser
import math
import os
import struct
import time
from pathlib import Path

import serial

cfg = configparser.ConfigParser()
cfg.read("config.ini")


def get(section, key, fallback=None, cast=str):
    try:
        val = cfg.get(section, key)
        return cast(val)
    except Exception:
        return fallback


PORT_RECEIVER = get("serial", "port_reciever", "/dev/ttyACM1", str)
BAUDRATE = get("serial", "baudrate", 115200, int)
SYNC_BYTE = get("serial", "sync_byte", "0xAA", str)
if isinstance(SYNC_BYTE, str) and SYNC_BYTE.startswith("0x"):
    SYNC_BYTE = int(SYNC_BYTE, 16)
else:
    SYNC_BYTE = int(SYNC_BYTE)

BIT_DURATION = get("timing", "bit_duration", 0.2, float)
PREAMBLE = [int(c) for c in get("format", "preamble", "1010101011100010", str)]

STREAM_FILE = Path(get("io", "stream_file", ".stream", str))
RECEIVED_DIR = Path(get("io", "received_dir", "received", str))
BUFFERING = int(get("io", "buffering", -1, int))

AMP_VAR_THRESHOLD = get("thresholds", "amp_var_threshold", 10, int)


RECEIVED_DIR.mkdir(parents=True, exist_ok=True)
RECORD = struct.Struct("<dfffB7x")


def log_sample(f, ts, raw, ema, thr, transition, sampled):
    flags = (transition << 0) | (sampled << 1)
    f.write(RECORD.pack(ts, raw, ema, thr, flags))


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
    if tr.in_waiting < 3:
        return None
    b = tr.read(1)
    if not b or b[0] != SYNC_BYTE:
        return None
    data = tr.read(2)
    if len(data) != 2:
        return None
    return struct.unpack("<H", data)[0]


def main():
    rx = serial.Serial(PORT_RECEIVER, BAUDRATE, timeout=0)
    time.sleep(1)

    os.makedirs("received", exist_ok=True)
    f = open(STREAM_FILE, "ab", buffering=BUFFERING)

    state = None

    raw = None
    raw_ema = None
    RAW_EMA_TAU = 0.01 * BIT_DURATION
    DELTA = 1

    noise_ema = 0
    NOISE_EMA_TAU = 0.05 * BIT_DURATION

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
    try:
        now = time.monotonic()
        while True:
            prev_now = now
            now = time.monotonic()
            dt = now - prev_now

            prev_raw = raw
            next_raw = read_sample(rx)
            if next_raw is None:
                continue
            raw = float(next_raw)

            if prev_raw is None:
                continue

            if raw_ema is None:
                raw_ema = raw
            raw_ema_alpha = 1 - math.exp(-dt / RAW_EMA_TAU)
            raw_ema += raw_ema_alpha * (raw - raw_ema)

            slope = (raw - raw_ema) / DELTA
            slope = (1 if slope > 0 else -1) * max(0, abs(slope) - 50)

            noise_ema_alpha = 1 - math.exp(-dt / NOISE_EMA_TAU)
            noise_ema += noise_ema_alpha * (abs(slope) - noise_ema)

            transition = False
            if abs(slope) > noise_ema * AMP_VAR_THRESHOLD and (
                last_transition_ts is None
                or now - last_transition_ts > 0.5 * BIT_DURATION
            ):
                last_transition_ts = now
                transition = True
                state = int(slope > 0)
                next_sample_ts = last_transition_ts + BIT_DURATION * 0.5

            if last_transition_ts is not None:
                if (
                    not preamble_found
                    and now - last_transition_ts > len(PREAMBLE) * BIT_DURATION
                ):
                    print("Preamble transition timeout")
                    reset()
                    continue

            log_sample(
                f,
                ts=now,
                raw=raw,
                ema=slope,
                thr=noise_ema,  # amp_min,
                transition=int(transition),  # int(transition),
                sampled=int(next_sample_ts is not None and now >= next_sample_ts),
            )

            if next_sample_ts is not None and now >= next_sample_ts:
                if state is None:
                    continue

                next_sample_ts += BIT_DURATION
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

                if len(bit_history) % int(5 / BIT_DURATION) == 0:
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
