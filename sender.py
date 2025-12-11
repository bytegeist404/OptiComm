# sender.py
import socket

HOST = '127.0.0.1'
PORT = 5000

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))

print("Connected to receiver. Type to send:")

while True:
    text = input("> ")
    sock.send(text.encode("utf-8"))
 