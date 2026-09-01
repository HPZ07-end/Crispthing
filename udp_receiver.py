import socket

HOST = "0.0.0.0"
PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((HOST, PORT))

print(f"Listening for UDP packets on {HOST}:{PORT}")
print("Press Ctrl+C to stop.")

try:
    while True:
        data, address = sock.recvfrom(4096)
        text = data.decode("utf-8", errors="replace")
        print(f"{address[0]}:{address[1]} -> {text.rstrip()}")
except KeyboardInterrupt:
    print("\nStopped.")
finally:
    sock.close()