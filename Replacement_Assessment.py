import socket

# Set up a socket using AF_INET interface, streaming protocol (TCP)
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

host_address = '192.168.1.5'  # Replace this with your actual IP Address, otherwise you will only accept local communciations
port = 12345  # Replace with a suitable port

# Try to connect to server
s.connect((host_address, port))
while True:
    # Ask user to enter text to send
    text = "Team 13 :)"
    if text == 'quit':
        break

    # Send text to server
    s.send(bytes(text, 'UTF-8'))

    # Receive message in response
    data = s.recv(1024)

    print('I received', data)
    break

# Close connection
s.close()