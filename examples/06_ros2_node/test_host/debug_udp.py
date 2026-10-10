import socket
import time

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(('0.0.0.0', 7447))
s.settimeout(1.0)

print("[debug_udp] Listening on 0.0.0.0:7447 for 5 seconds...")
start = time.time()
count = 0
while time.time() - start < 2.0:
    try:
        data, addr = s.recvfrom(2048)
        count += 1
        print(f"[{count}] Recv from {addr}: {len(data)} bytes | hex={data[:16].hex()}...")
    except socket.timeout:
        pass

print(f"[debug_udp] Total received: {count} packets")
