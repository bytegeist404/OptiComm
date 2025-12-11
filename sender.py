import serial

PORT = 'COM3'
BAUD = 115200

ser = serial.Serial(PORT, BAUD)
print(f"[Sender PC] Connected to Arduino on {PORT} at {BAUD} baud.")
print("Type messages to send. Press CTRL+C to exit.")

while True:
    try:
        text = input("> ")
        ser.write(text.encode("utf-8"))
    except KeyboardInterrupt:
        print("\nExiting...")
        break
