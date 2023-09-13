import socket

# Define the server IP address and port
server_ip = '118.138.20.161'  # Replace with the actual IP address of the receiving computer
server_port = 137  # Use the same port number as the server

# Create a socket object and connect to the server
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((server_ip, server_port))

# Send text data to the server
while True:
    data = input("Enter text to send (or 'exit' to quit): ")
    if data == 'exit':
        break
    client_socket.send(data.encode())

# Close the client socket
client_socket.close()
