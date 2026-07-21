import socket

HOST = "0.0.0.0"
PORT = 5005  # 必须与手机端设置一致

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((HOST, PORT))

print(f"正在监听 UDP 端口 {PORT}...")
print("按 Ctrl+C 停止。\n")

try:
    while True:
        data, address = sock.recvfrom(2048)

        try:
            message = data.decode("utf-8").strip()
        except UnicodeDecodeError:
            message = repr(data)

        print(f"{address[0]}:{address[1]} -> {message}")

except KeyboardInterrupt:
    print("\n监听已停止。")

finally:
    sock.close()