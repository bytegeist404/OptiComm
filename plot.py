# import struct
# from dataclasses import dataclass

# import matplotlib.pyplot as plt
# import numpy as np

# RECORD = struct.Struct("<IHffBB")

# RX_FILE = ".stream"
# TX_FILE = "tx.log"

# # ---------------- RX ----------------


# FLAG_TRANSITION = 1 << 0
# FLAG_SAMPLED = 1 << 1


# class Rx:
#     ts: list = []
#     raw: list = []
#     raw_ema: list = []
#     noise_ema: list = []
#     transition_ts: list = []
#     sampled_ts: list = []


# with open(RX_FILE, "rb") as f:
#     size = RECORD.size
#     while True:
#         chunk = f.read(size)
#         if len(chunk) < size:
#             break
#         ts_u32, raw_u16, raw_ema_f, noise_ema_f, transition_b, sampled_b = (
#             RECORD.unpack(chunk)
#         )

#         Rx.ts.append(ts_u32)
#         Rx.raw.append(float(raw_u16))
#         Rx.raw_ema.append(float(raw_ema_f))
#         Rx.noise_ema.append(float(noise_ema_f))

#         if transition_b & FLAG_TRANSITION:
#             Rx.transition_ts.append(ts_u32)
#         if sampled_b & FLAG_SAMPLED:
#             Rx.sampled_ts.append(ts_u32)

# Rx.ts = np.asarray(Rx.ts, dtype=float)
# Rx.raw = np.asarray(Rx.raw, dtype=float)
# Rx.raw_ema = np.asarray(Rx.raw_ema, dtype=float)
# Rx.noise_ema = np.asarray(Rx.noise_ema, dtype=float)
# Rx.transition_ts = np.asarray(Rx.transition_ts, dtype=float)
# Rx.sampled_ts = np.asarray(Rx.sampled_ts, dtype=float)

# Rx.ts *= 1e-6
# Rx.transition_ts *= 1e-6
# Rx.sampled_ts *= 1e-6


# # ---------------- TX ----------------
# class Tx:
#     ts = []
#     state = []


# with open(TX_FILE) as f:
#     for line in f:
#         ts, state = line.strip().split(",")
#         Tx.ts.append(float(ts))
#         Tx.state.append(int(state))
# print(Rx.sampled_ts, Rx.transition_ts)

# Tx.ts = np.asarray(Tx.ts, dtype=float)
# Tx.state = np.asarray(Tx.state, dtype=float)

# # filter relevant data
# t0 = 11.6
# t1 = 12.3
# mask = (Rx.ts > t0) & (Rx.ts < t1)
# Rx.ts = Rx.ts[mask]
# Rx.raw = Rx.raw[mask]
# Rx.raw_ema = Rx.raw_ema[mask]
# Rx.noise_ema = Rx.noise_ema[mask]

# mask = (Rx.transition_ts > t0) & (Rx.transition_ts < t1)
# Rx.transition_ts = Rx.transition_ts[mask]
# mask = (Rx.sampled_ts > t0) & (Rx.sampled_ts < t1)
# Rx.sampled_ts = Rx.sampled_ts[mask]


# t0 = min(Rx.ts)
# Rx.ts -= t0
# Rx.transition_ts -= t0
# Rx.sampled_ts -= t0
# Tx.ts -= min(Tx.ts)

# mid = Rx.raw.mean()
# state = Rx.raw > mid
# edges = np.where((~state[:-1]) & (state[1:]))[0]

# if len(edges) == 0:
#     t0 = 0
# else:
#     t0 = Rx.ts[edges[0] + 1]

# Tx.ts += t0


# # ----- after you have built Rx.* and Tx.* arrays and performed filtering/aligning -----

# # DEBUG: inspect arrays (first few values)
# np.set_printoptions(precision=6, suppress=True)
# print("RX.ts  [sample]   :", Rx.ts[:10])
# print("RX.trans [sample] :", Rx.transition_ts[:10])
# print("RX.samp  [sample] :", Rx.sampled_ts[:10])
# print("TX.ts  [sample]   :", Tx.ts[:10])

# # Now plotting — no additional 1e-6 factors below
# aspect = 16 / 9
# width = 23.39 / 4
# height = width / aspect * 1.35  # a little taller for 2 panels

# fig, (ax1, ax2) = plt.subplots(
#     2, 1, figsize=(width, height), sharex=True, gridspec_kw={"height_ratios": [2.5, 1]}
# )

# # RX raw
# ax1.plot(Rx.ts, Rx.raw, color="tab:blue", label="RX raw")
# ax1.set_ylabel("ADC value")
# ax1.grid(True, alpha=0.3)

# # TX shaded regions (Tx.ts should be in seconds and aligned already)
# for i in range(len(Tx.state) - 1):
#     if Tx.state[i] == 1:
#         ax1.axvspan(
#             Tx.ts[i],
#             Tx.ts[i + 1],
#             color="tab:red",
#             alpha=0.25,
#             linewidth=0,
#             label="_nolegend_",
#         )

# # Transitions (solid) — Rx.transition_ts already in seconds, do NOT multiply again
# if Rx.transition_ts.size:
#     for t in Rx.transition_ts:
#         ax1.axvline(t, color="c", linewidth=1.2, alpha=0.9)
# else:
#     print("No transitions in window")

# # Sample instants (dashed) — Rx.sampled_ts already in seconds, do NOT multiply again
# if Rx.sampled_ts.size:
#     for t in Rx.sampled_ts:
#         ax1.axvline(t, color="m", linestyle="--", linewidth=1.0, alpha=0.8)
# else:
#     print("No sampled points in window")

# # Legend (clean proxies)
# ax1.plot([], [], color="tab:blue", label="RX raw")
# ax1.plot([], [], color="tab:red", alpha=0.25, linewidth=8, label="TX high")
# ax1.plot([], [], color="c", linewidth=1.2, label="Transition detected")
# ax1.plot([], [], color="m", linestyle="--", linewidth=1.0, label="Sample time")
# ax1.legend(loc="upper right")

# # Slope / noise
# ax2.plot(Rx.ts, Rx.raw_ema, color="tab:green", label="Slope")
# ax2.plot(Rx.ts, Rx.noise_ema, color="tab:orange", label="Noise EMA")
# ax2.set_ylabel("Detector signals")
# ax2.set_xlabel("Time (s)")
# ax2.grid(True, alpha=0.3)
# ax2.legend(loc="upper right")

# plt.suptitle("Receiver signal, detection pipeline, and transmitter state")
# plt.tight_layout()

# plt.savefig(
#     "rx_vs_tx_extended_fixed.png",
#     dpi=1200,
#     bbox_inches="tight",
#     pad_inches=0.02,
# )
# plt.show()


import matplotlib.pyplot as plt
import numpy as np

# ==========================
# Configuration
# ==========================

PREAMBLE = "1010101011100010"  # your sync pattern
BITRATE_LABEL = "Bits"

# ==========================
# Helpers
# ==========================


def bytes_to_bits(data: bytes):
    bits = []
    for b in data:
        for i in range(7, -1, -1):
            bits.append((b >> i) & 1)
    return bits


def uint32_to_bits(n):
    return [(n >> i) & 1 for i in range(31, -1, -1)]


def build_message(payload: bytes):
    preamble_bits = [int(b) for b in PREAMBLE]

    payload_bits = bytes_to_bits(payload)

    length_bits = uint32_to_bits(len(payload_bits))

    return preamble_bits, length_bits, payload_bits


def digital_wave(bits):
    t = np.arange(len(bits) + 1)
    y = np.repeat(bits, 2)
    y = np.append(y, y[-1])

    t_plot = np.repeat(t[:-1], 2)
    t_plot = np.append(t_plot, t_plot[-1] + 1)

    return t_plot, y


# ==========================
# Example message (CHANGE THIS)
# ==========================

payload = b"Hello"  # any bytes you want

# ==========================
# Build structure
# ==========================

preamble_bits, length_bits, payload_bits = build_message(payload)

all_bits = preamble_bits + length_bits + payload_bits
t, y = digital_wave(all_bits)

# ==========================
# Plot
# ==========================


# # Now plotting — no additional 1e-6 factors below
aspect = 8 / 2
width = 23.39 / 4
height = width / aspect * 1.35  # a little taller for 2 panels

fig, ax = plt.subplots(figsize=(width, height))

ax.step(t, y, where="post", linewidth=2)

ax.set_ylim(-0.3, 1.3)
ax.set_yticks([0, 1])
ax.set_yticklabels(["Laser aus", "Laser an"])
ax.set_xlabel(BITRATE_LABEL)
ax.set_title("Struktur einer übertragenen Nachricht")

# --------------------------
# Region highlighting
# --------------------------

p_len = len(preamble_bits)
l_len = len(length_bits)
m_len = len(payload_bits)

ax.axvspan(0, p_len, alpha=0.15, color="tab:blue")
ax.axvspan(p_len, p_len + l_len, alpha=0.15, color="tab:green")
ax.axvspan(p_len + l_len, p_len + l_len + m_len, alpha=0.15, color="tab:orange")

ax.text(p_len / 2, 1.15, "Preamble", ha="center", color="tab:blue")
ax.text(p_len + l_len / 2, 1.15, "Länge (32 Bit)", ha="center", color="tab:green")
ax.text(p_len + l_len + m_len / 2, 1.15, "Payload", ha="center", color="tab:orange")

# --------------------------
# Paper look
# --------------------------

ax.spines["top"].set_visible(False)
ax.spines["right"].set_visible(False)
ax.grid(True, axis="x", alpha=0.25)

plt.tight_layout()

# Best export formats
plt.savefig("message_structure.pdf", bbox_inches="tight")  # vector (best!)
plt.savefig("message_structure.png", dpi=1200, bbox_inches="tight")

plt.show()
