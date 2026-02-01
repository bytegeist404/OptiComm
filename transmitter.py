import enum
import queue
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import List, Optional

import serial

# CONFIG
PORT_TRANSMITTER = "/dev/ttyACM0"
BAUDRATE = 115200

BIT_DURATION = 0.5
START_MARKER: List[int] = [
    int(c) for c in 4 * "10101010" + "11100010"
]  # Turn on and off a few times, to make sure the ema detected the mean value


class TxCommand(enum.Enum):
    SET_MODE = enum.auto()
    STOP = enum.auto()


@dataclass
class Command:
    type: TxCommand
    value: object = None
    payload: Optional[List[int]] = None


def encode_bytes_to_bits(data: bytes):
    bits = []
    for b in data:
        for i in range(7, -1, -1):
            bits.append((b >> i) & 1)
    return bits


def transmitter_thread(tx: serial.Serial, cmd_q: queue.Queue[Command]) -> None:
    mode = "off"
    bit_q: deque = deque()
    next_bit_ts: float = time.time()

    try:
        while True:
            try:
                cmd: Command | None = cmd_q.get_nowait()
            except queue.Empty:
                cmd = None

            if cmd is not None:
                if cmd.type == TxCommand.SET_MODE:
                    mode = str(cmd.value)
                    bit_q.clear()
                    next_bit_ts: float = time.time()

                    if mode == "msg":
                        payload_bits = cmd.payload or []
                        length_bits = encode_bytes_to_bits(
                            len(payload_bits).to_bytes(4, "big")
                        )
                        full_bits = START_MARKER + length_bits + payload_bits + [0]
                        bit_q = deque(full_bits)

                    if mode == "off":
                        bit_q.append(0)

                    if mode == "on":
                        bit_q.append(1)

                elif cmd.type == TxCommand.STOP:
                    break

            if mode == "msg" and not bit_q:
                cmd_q.put(Command(TxCommand.SET_MODE, "off"))
                continue

            now = time.time()
            if now < next_bit_ts or not bit_q:
                continue

            bit = bit_q.popleft()
            tx.write(b"\x01" if bit else b"\x00")
            next_bit_ts = now + BIT_DURATION
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Keyboard Interrupt - Stopping")

    tx.write(b"\x00")


def main() -> None:
    cmd_q: queue.Queue[Command] = queue.Queue()

    tx = serial.Serial(PORT_TRANSMITTER, BAUDRATE, timeout=0)
    time.sleep(1)

    t = threading.Thread(target=transmitter_thread, args=(tx, cmd_q), daemon=True)
    t.start()

    try:
        while True:
            cmdline = input("mode (off/on/msg <text>/file <path>): ").strip()

            if not cmdline:
                continue

            if cmdline.startswith("msg "):
                text = cmdline[4:]
                data = text.encode("utf-8")
                bits = encode_bytes_to_bits(data)
                cmd_q.put(Command(TxCommand.SET_MODE, value="msg", payload=bits))

            elif cmdline.startswith("file "):
                path = cmdline[5:].strip()
                with open(path, "rb") as f:
                    data = f.read()

                bits = encode_bytes_to_bits(data)
                cmd_q.put(Command(TxCommand.SET_MODE, value="msg", payload=bits))

            elif cmdline in ("off", "on"):
                cmd_q.put(Command(TxCommand.SET_MODE, value=cmdline))

            elif cmdline in ("quit", "exit"):
                cmd_q.put(Command(TxCommand.STOP))
                break

            else:
                cmd_q.put(Command(TxCommand.SET_MODE, value=cmdline))

    except KeyboardInterrupt:
        print("\nKeyboard interrupt received, stopping transmitter")
        cmd_q.put(Command(TxCommand.STOP))
        t.join(timeout=2.0)
        tx.close()


if __name__ == "__main__":
    main()
