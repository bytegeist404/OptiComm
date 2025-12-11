import serial, socket, threading

ser = serial.Serial('COM3', 115200)

HOST = '127.0.0.1'
PORT = 5000

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((HOST, PORT))
sock.listen(1)

print("Receiver ready. Waiting for sender terminal...")
conn, addr = sock.accept()
print("Sender terminal connected.")

def forward_from_sender():
    """Receive data from sender terminal and forward to Arduino."""
    while True:
        data = conn.recv(1024)
        if not data:
            break
        ser.write(data)

def read_from_arduino():
    """Read lines from Arduino and print to this terminal."""
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        print("[ARDUINO]", line)

threading.Thread(target=forward_from_sender, daemon=True).start()
threading.Thread(target=read_from_arduino, daemon=True).start()

# Keep alive
while True:
    pass
