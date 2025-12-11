import serial

PORT = 'COM3'
BAUD = 115200

ser = serial.Serial(PORT, BAUD)
print(f"[Receiver PC] Connected to Arduino on {PORT} at {BAUD} baud.")
print("Waiting for laser-transmitted data...\n")

while True:
    try:
        line = ser.readline().decode(errors="ignore").strip()
        if line:
            print("[RECEIVER]", line)
    except KeyboardInterrupt:
        print("\nExiting...")
        break
