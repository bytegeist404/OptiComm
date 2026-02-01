from __future__ import annotations

import configparser
import enum
import queue
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Deque, List, Optional

import serial

cfg = configparser.ConfigParser()
cfg.read("config.ini")


def get(section: str, key: str, fallback=None, cast=str):
    try:
        val = cfg.get(section, key)
        return cast(val)
    except Exception:
        return fallback


PORT_TRANSMITTER = get("serial", "port_transmitter", "/dev/ttyACM0", str)
BAUDRATE = get("serial", "baudrate", 115200, int)

BIT_DURATION = get("timing", "bit_duration", 0.2, float)
PREAMBLE = [int(c) for c in get("format", "preamble", "1010101011100010", str)]


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


def main() -> None:
    cmd_q: queue.Queue[Command] = queue.Queue()

    tx = serial.Serial(PORT_TRANSMITTER, BAUDRATE, timeout=0)
    time.sleep(1)

    t = threading.Thread(target=transmitter_thread, args=(tx, cmd_q), daemon=True)
    t.start()

    info_str = (
        "command:\n"
        "  off | 0              turn laser off\n"
        "  on  | 1              turn laser on\n"
        "  msg <text>           transmit UTF-8 text\n"
        "  file <path>          transmit file contents\n"
        "  bits <01...>         transmit raw bit string\n"
        "  quit | q | exit | e  stop transmitter\n"
        "  help | h             help\n"
    )
    print(info_str, end="")

    try:
        while True:
            cmdline = input("> ").strip()
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

            elif cmdline.startswith("bits "):
                text = cmdline[5:]
                bits = [int(c) for c in text]
                cmd_q.put(Command(TxCommand.SET_MODE, value="bits", payload=bits))

            elif cmdline in ("off", "0"):
                cmd_q.put(Command(TxCommand.SET_MODE, value="0"))

            elif cmdline in ("on", "1"):
                cmd_q.put(Command(TxCommand.SET_MODE, value="1"))

            elif cmdline in ("quit", "q", "exit", "e", "stop"):
                cmd_q.put(Command(TxCommand.STOP))
                break

            elif cmdline in ("help", "h"):
                print(info_str, end="")

    except KeyboardInterrupt:
        print("\nKeyboard interrupt received, stopping transmitter")
        cmd_q.put(Command(TxCommand.STOP))

    t.join(timeout=2.0)
    try:
        tx.close()
    except Exception:
        pass


if __name__ == "__main__":
    main()
