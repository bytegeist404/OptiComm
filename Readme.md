# 📡 OptiComm — Optical Laser Communication System

*A Jugend Forscht Project*

OptiComm is a simple but fully functional **laser-based data transmission system**.
It sends arbitrary text data from one Arduino to another using:

* A **laser diode** (transmitter)
* A **photodiode / LDR / photoresistor** (receiver)
* A simple **on/off optical encoding scheme**
* Two PCs connected to the Arduinos via USB

This repository includes:

* Arduino code for the **sender** and **receiver**
* Python scripts for the **sending PC** and **receiving PC**
* A simple serial-based transport layer using start/stop bits

The goal is to create a robust, understandable communication setup that beginners can read, modify, and expand.

---

# 🗂 Repository Structure

```
OptiComm/
│
├── sender/
│   └── sender.ino
│
├── receiver/
│   └── receiver.ino
│
├── sender.py
├── receiver.py
└── requirements.txt
```

---

# 🔦 How the System Works (High-Level)

1. **User types text** on the *sending PC*
2. `sender.py` sends this text to the sender Arduino over USB
3. The Arduino converts every character into bits and encodes them as **laser on/off pulses**
4. At the receiver side, the photodiode reads incoming light pulses
5. The receiver Arduino decodes the optical signal back into bytes
6. `receiver.py` prints the reconstructed messages on the receiving PC

This makes your laser link behave like a very slow “optical USB cable.”

---

# 🧠 Bit Encoding Protocol (Very Simple UART-Style)

Each transmitted byte uses:

```
1 start bit  = 1 (laser on)
8 data bits  = LSB first
1 stop bit   = 0 (laser off)
```

Timing:

```
BIT_DURATION_MS = 40
```

— so one byte takes *10 × 40 ms = 400 ms*
≈ **2.5 bytes per second**
≈ **20 bits/s**

This slow speed makes the system easier to debug and less sensitive to noise.

---

# ⚙ Arduino Sender (`sender/sender.ino`)

### Responsibilities:

* Read bytes coming from the PC over serial
* Convert each byte into bits
* Turn the laser ON/OFF for each bit

### Key functions:

#### `sendBit(bool bit)`

Turns the laser HIGH or LOW for exactly 40 ms.

#### `sendByte(uint8_t b)`

* Sends a start bit (`1`)
* Sends the 8 data bits (least significant bit first)
* Sends a stop bit (`0`)
* Adds a 2-bit pause

The loop simply waits for incoming serial bytes and transmits them immediately.

---

# 📥 Arduino Receiver (`receiver/receiver.ino`)

### Responsibilities:

* Sample the photodiode input
* Subtract background light using a moving average filter
* Reconstruct bits by averaging the entire bit window
* Print recovered bytes over USB

### Bit reading
The method:

* Continuously samples during the whole bit window
* Averages all samples
* Compares average to threshold

#### `readBit()`

```cpp
float avg = 0.;
float samples = 0.;

while (millis() - start < BIT_DURATION_MS) {
    avg += readSignal();
    samples++;
}

return avg / samples > SIGNAL_THRESHOLD;
```

This makes the system **far more resistant to noise, flicker, and timing jitter**.

#### Background subtraction

`readSignal()` maintains a moving average background:

```
background = background*0.98 + photo_val*0.02
signal = photo_val - background
```

This allows it to **adapt to changing room lighting**.

---

# 🖥 Sender PC Script (`sender.py`)

### Responsibilities:

* Open serial connection to the sender Arduino
* Allow the user to type text
* Send typed text as raw bytes to the Arduino

```
> Hello
```

This sends: `H e l l o` byte by byte.

You can terminate with **CTRL+C**.

---

# 💻 Receiver PC Script (`receiver.py`)

### Responsibilities:

* Connect to the receiver Arduino
* Print whatever it receives

It simply listens and prints lines like:

```
[RECEIVER] Received: 72 (H)
```

Runs until CTRL+C.

---

# 🛠 Installing & Running the Python Scripts

### 1. Install Python 3.8+

[https://www.python.org/downloads/](https://www.python.org/downloads/)

### 2. Install required packages

Inside the project folder:

```bash
pip install -r requirements.txt
```

(Only requires `pyserial`.)

### 3. Identify the serial ports

On **Windows**
Check Device Manager → “Ports (COM…)”

On **Linux/macOS**
Use:

```bash
ls /dev/tty*
```

Update `PORT` in sender.py / receiver.py:

```python
PORT = 'COM3'
```

### 4. Connect hardware

* Sender PC ↔ Sender Arduino ↔ Laser
* Receiver PC ↔ Receiver Arduino ↔ Photodiode

### 5. Run the scripts

#### On sender PC:

```bash
python sender.py
```

#### On receiver PC:

```bash
python receiver.py
```

#### On just one PC:
It is completely possible to run the full optical communication setup on **one computer**.
You will use the same hardware arrangement (laser → photodiode), but both Arduinos will be plugged into the same machine via USB.

##### ✔ What stays the same:

* Sender Arduino still transmits laser pulses
* Receiver Arduino still decodes them
* Python scripts remain unchanged

##### ✔ What changes:

You must assign each script to the correct **COM port**.

##### 1. Plug Both Arduinos into Your PC

Windows will automatically assign each device a different COM port, for example:

* **Sender Arduino → COM4**
* **Receiver Arduino → COM5**

macOS/Linux uses paths like `/dev/ttyUSB0` or `/dev/cu.usbmodem14101`.

Both devices can be connected at the same time with no conflicts — each one has its own serial interface.


##### 2. Identify the COM Port for Each Arduino

###### On Windows:

1. Open **Device Manager**
2. Expand **Ports (COM & LPT)**
3. Plug/unplug each Arduino and observe which COM port appears/disappears
4. Label them:

| Arduino  | COM Port (example) |
| -------- | ------------------ |
| Sender   | COM4               |
| Receiver | COM5               |

###### On Linux/macOS:

Run:

```bash
ls /dev/tty*
```

Then plug/unplug each Arduino to see which one shows up.


##### 3. Edit the Python Scripts

###### sender.py

Set the port of the **sender Arduino**:

```python
PORT = 'COM4'
```

###### receiver.py

Set the port of the **receiver Arduino**:

```python
PORT = 'COM5'
```

(Your actual COM numbers may differ.)

##### 4. Open Two Terminals

###### Terminal A — Sender:

```bash
python sender.py
```

###### Terminal B — Receiver:

```bash
python receiver.py
```

Both scripts can run simultaneously because each one opens a **different serial port**.

This is all that is required — no networking, no additional software.

---

# 🔬 How to Test the Optical Link

1. Upload `sender.ino` and `receiver.ino`
2. Open both Python scripts
3. Aim the laser at the photodiode
4. Type something into the sender PC terminal
5. Verify it shows correctly on the receiver PC

---

# 🧪 Ideas for Improvements

* Automatic bit timing calibration
* Manchester encoding
* Higher baud rates
* Error detection (checksums/parity)
* A 3D-printed optical focusing system
* Use LEDs for safe classroom demos