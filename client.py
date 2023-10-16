import json
import socket
import threading
from cryptography.fernet import Fernet
import time

class Client():
    def __init__(self):
        # Client Socket
        # Host Address
        # Port
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        host_address = '172.20.10.9'  # Replace this with Server IP address
        port = 12346  # Replace with server suitable port
        self.client_socket.connect((host_address, port))
        print("CONNECTED TO SERVER")
        # Receive private key
        key = self.client_socket.recv(1024)
        print("RECEIVED KEY")
        self.cipher_suite = Fernet(key)

    def send_message(self, JSON_object):
        # stringify JSON_object
        JSON_object_str = json.dumps(JSON_object)
        encrypted_message = self.cipher_suite.encrypt(JSON_object_str.encode())
        # Send the message to the server to broadcast
        self.client_socket.send(encrypted_message)


if __name__ == '__main__':
    client = Client()
    for index in range(100):
        JSON_object = {"pose": [500, 500, 45]}
        client.send_message(JSON_object)
        time.sleep(0.001)
    '''
    JSON_object = {"robot_id": x, "location": x, "status": x, "bin": x, "path": x}
    # Robot ID -
    # Location - [x, y]
    # Status - "Loading", "Unloading", "Moving", "Idle"
    # Bin - 1, 2, 3
    # Current Path - [[x, y]...]
    '''
