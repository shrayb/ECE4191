import socket

# Define the server IP address and port
server_ip = '118.138.20.161'  # Replace with the actual IP address of the receiving computer
server_port = 137  # Use the same port number as the server

# Create a socket object and bind it to the server address
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((server_ip, server_port))

# Listen for incoming connections
server_socket.listen(1)
print(f"Server is listening on {server_ip}:{server_port}")

# Accept a connection from a client
client_socket, client_address = server_socket.accept()
print(f"Accepted connection from {client_address}")

# Receive and print data from the client
while True:
    data = client_socket.recv(1024).decode()
    if not data:
        break
    print(f"Received: {data}")

# Close the client and server sockets
client_socket.close()
server_socket.close()
