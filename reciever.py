import socket

# ESP32 IP address and port
ESP32_IP = "192.168.13.111"  # Replace with your ESP32's IP address
ESP32_PORT = 12345          # ESP32's UDP port (not used in Python script)

# Python script's IP and port
PYTHON_IP = "0.0.0.0"       # Listen on all available interfaces
PYTHON_PORT = 12345      # Port to listen on (must match ESP32's remotePort)

# Create a UDP socket
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_socket.bind((PYTHON_IP, PYTHON_PORT))  # Bind to the specified IP and port

print(f"Waiting for data from ESP32 on port {PYTHON_PORT}...")

try:
    while True:
        # Receive data from ESP32
        data, addr = udp_socket.recvfrom(1024)  # Buffer size is 1024 bytes
        print(f"Received from {addr}: {data.decode('utf-8')}")

except KeyboardInterrupt:
    print("Script terminated by user.")

finally:
    # Close the socket
    udp_socket.close()
    print("UDP socket closed.")