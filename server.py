#!/usr/bin/env python3
import sys

sys.path.insert(1, '/home/lingc/ECE4191G11/ROS_PC/src/robot_controller/robot_controller')

import rclpy
from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor
from robot_interfaces.msg import JSONData

import json
import socket
from cryptography.fernet import Fernet
import threading


class Server(Node):
    def __init__(self):
        super().__init__("server_node")  # name of node in ros2
        self.get_logger().info("Server node initialised")
        # Generate private keys to send to clients
        self.key = Fernet.generate_key()
        self.cipher_suite = Fernet(self.key)
        # Set up a socket using AF_INET interface, streaming protocol (TCP)
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.host_address = '192.168.39.239'  # Replace with actual machine it is on
        self.port = 12346
        # Bind socket to address and port
        self.server_socket.bind((self.host_address, self.port))
        # Queue up 2 connections at a time
        self.server_socket.listen(2)
        self.client_list = []

        # Comms Publisher
        self.comms_publisher = self.create_publisher(JSONData, "json_data", 10)

        self.server_loop()

    def handle_client(self, client_socket):
        self.get_logger().info("handling client")
        client_socket.send(self.key)
        while True:
            try:
                # Receive an encrypted message from the client_socket
                encrypted_message = client_socket.recv(1024)
                # Broadcast the message to all other clients
                self.recieve_msg(encrypted_message)

            except Exception as e:
                break
    
    def recieve_msg(self, encrypted_message):
        decrypted_message = self.cipher_suite.decrypt(encrypted_message).decode()
        msg = JSONData()  # Should be a string, handle the object in path planner
        msg.json_data = decrypted_message
        self.comms_publisher.publish(msg)
        self.get_logger().info(f"Message recieved: {msg.json_data}")

    def server_loop(self):
        while True:
            # Accept incoming client connections
            client_socket, client_address = self.server_socket.accept()
            self.client_list.append(client_socket)
            # Start a thread to handle the client
            client_handler = threading.Thread(target=self.handle_client, args=(client_socket,))
            client_handler.start()

def main(args=None):
    try:
        rclpy.init(args=args)

        server_node = Server()

        executor = MultiThreadedExecutor()
        executor.add_node(server_node)
        executor.spin()

    except KeyboardInterrupt:
        server_node.get_logger().info("Server node shutdown")
    rclpy.shutdown()