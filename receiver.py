import math
import struct
import time
from collections import deque

import serial

# CONFIG

PORT_RECEIVER = "/dev/ttyACM1"
BAUDRATE = 115200
SYNC_BYTE = 0xAA

START_MARKER = [int(c) for c in "1010101011100010"]
END_MARKER = [int(c) for c in "0001110101101000"]
RECORD = struct.Struct("<dfffB7x")


def log_sample(f, ts, raw, ema, thr, transition, sampled):
    flags = (transition << 0) | (sampled << 1)
    f.write(RECORD.pack(ts, raw, ema, thr, flags))


def bits_to_bytes(bits: list[int]) -> bytes:
    n = len(bits) // 8 * 8
    bits = bits[:n]

    out = bytearray()
    for i in range(0, n, 8):
        byte = 0
        for b in bits[i : i + 8]:
            byte = (byte << 1) | b
        out.append(byte)

    return bytes(out)


def bits_to_uint(bits: list[int]) -> int:
    value = 0
    for b in bits:
        value = (value << 1) | int(b)
    return value


def read_sample(tr: serial.Serial) -> int | None:
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
    SAMPLE_RATE = 100
    CALIBRATION_TRANSITIONS = 8

    f = open(".stream", "ab", buffering=0)

    try:
        rx = serial.Serial(PORT_RECEIVER, BAUDRATE, timeout=0)
    except Exception as e:
        return
    time.sleep(2)

    ema_tau = 1
    a = 1 - math.exp(-1 / (SAMPLE_RATE * ema_tau))
    ema = None

    hyst_delta = 100

    prev_state = None
    curr_state = False
    transition_detection_ts_q = deque(maxlen=CALIBRATION_TRANSITIONS)

    calibrated_bit_duration = None
    recording = False
    next_sample_ts = None
    tmp_bit_q = deque(maxlen=16)
    payload_length_q = deque(maxlen=32)
    payload_length = None
    bit_history = None

    def reset_state():
        nonlocal prev_state, curr_state, transition_detection_ts_q, calibrated_bit_duration, recording, next_sample_ts, tmp_bit_q, payload_length_q, payload_length, bit_history
        prev_state = None
        curr_state = False
        transition_detection_ts_q = deque(maxlen=CALIBRATION_TRANSITIONS)

        calibrated_bit_duration = None
        recording = False
        next_sample_ts = None
        tmp_bit_q = deque(maxlen=16)
        payload_length_q = deque(maxlen=32)
        payload_length = None
        bit_history = None
        print("reset state, ready to receive data")

    reset_state()
    try:
        while True:
            raw = read_sample(rx)
            if raw is None:
                continue

            if ema is None:
                ema = raw
                continue

            if not recording:
                ema = ema + a * (raw - ema)

            thr = ema - hyst_delta if curr_state else ema + hyst_delta
            prev_state = curr_state
            curr_state = raw > thr

            now = time.time()
            transition = prev_state != curr_state
            if transition:
                transition_detection_ts_q.append(now)

            calibration_updated = False
            if (
                not recording
                and len(transition_detection_ts_q) >= CALIBRATION_TRANSITIONS
            ):
                ts = list(transition_detection_ts_q)[-CALIBRATION_TRANSITIONS:]
                dts = [ts[i + 1] - ts[i] for i in range(len(ts) - 1)]

                dts_sorted = sorted(dts)
                median = dts_sorted[len(dts_sorted) // 2]

                abs_dev = [abs(dt - median) for dt in dts]
                abs_dev_sorted = sorted(abs_dev)
                mad = abs_dev_sorted[len(abs_dev_sorted) // 2]

                if mad != 0:
                    K = 3.0
                    kept = [dt for dt in dts if abs(dt - median) <= K * mad]

                    MIN_FRACTION = 0.7
                    if len(kept) > MIN_FRACTION * len(dts):
                        calibration_updated = True
                        next_cbd = sum(kept) / len(kept)

                        if calibrated_bit_duration is None:
                            calibrated_bit_duration = next_cbd
                        else:
                            calibrated_bit_duration = calibrated_bit_duration + 0.5 * (
                                next_cbd - calibrated_bit_duration
                            )

                        print(f"Calibrated: {calibrated_bit_duration}")

                transition_detection_ts_q.popleft()

                if calibration_updated:
                    next_sample_ts = None

            if calibrated_bit_duration and (next_sample_ts is None or transition):
                next_sample_ts = now + calibrated_bit_duration / 2

            sampled = False
            if calibrated_bit_duration and next_sample_ts and now >= next_sample_ts:
                tmp_bit_q.append(curr_state)
                next_sample_ts += calibrated_bit_duration
                sampled = True

                if recording:
                    if payload_length is None:
                        payload_length_q.append(curr_state)

                        if len(payload_length_q) == 32:
                            payload_length = bits_to_uint(list(payload_length_q))
                            print(
                                f"Received Payload length: {payload_length}, eta: {payload_length * calibrated_bit_duration:.4f}s"
                            )

                            bit_history = []
                    else:
                        if bit_history is None:
                            continue
                        bit_history.append(curr_state)

                        if len(bit_history) % int(5 / calibrated_bit_duration) == 0:
                            print(
                                f"Received: {len(bit_history)} / {payload_length} bits"
                            )

                        if len(bit_history) >= payload_length:
                            print(f"Finished Recording {payload_length} bits")

                            recording = False

                            payload_bits = bit_history[:payload_length]

                            data = bits_to_bytes(payload_bits)
                            filename = f"received/{int(time.time())}.bin"
                            with open(filename, "wb") as fp:
                                fp.write(data)
                            print(f"Wrote received data to {filename}")

                            reset_state()

                if not recording and list(tmp_bit_q) == START_MARKER:
                    recording = True
                    bit_history = []
                    print(f"Started Recording, ema: {ema}")

            log_sample(
                f,
                ts=time.time(),
                raw=raw,
                ema=ema or 0.0,
                thr=thr,
                transition=(
                    int(prev_state != curr_state) if prev_state is not None else 0
                ),
                sampled=int(sampled),
            )
            time.sleep(0.001)
    except Exception as e:
        print(e)
    except KeyboardInterrupt as e:
        print("Keyboard Interrupt - Stopping.")
    finally:
        f.close()
        rx.close()


if __name__ == "__main__":
    main()
