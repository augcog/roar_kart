import socket
import time
import struct
import json

# Define the destination IP address and port
ip_address = "10.42.0.4"
port = 1883

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Define the message you want to send

try:
    # Receive State
    for i in range(10):
        message = b"s"
        print(f"    [{time.time()}] Sent: {message}")
        sock.sendto(message, (ip_address, port))
        data, addr = sock.recvfrom(1024)  # Buffer size is 1024 bytes
        print(f"    Message received [{len(data)}] from  [{addr}]: [{data}]")

        print()
        time.sleep(0.05)

    # Send Action
    for i in range(10):
        message_dict = {
            "target_speed": 10.0,
            "steering_angle": 5.0,
            "brake": 0.0,
            "reverse": False,
        }
        message_str = json.dumps(message_dict)
        message = message_str.encode("utf-8")
        print(f"    [{time.time()}] Sent: {message}")
        sock.sendto(message, (ip_address, port))
        time.sleep(0.05)


except socket.error as e:
    print("Error occurred while sending UDP packet:", e)
finally:
    # Close the socket
    sock.close()
