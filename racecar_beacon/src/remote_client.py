#!/usr/bin/env python

import socket

HOST = '127.0.0.1'
# This process should listen to a different port than the PositionBroadcast client.
PORT = 65432

# Create a socket and connect to the server
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # IPv4, TCP
client.connect(HOST, PORT)

# Send data to the server
data = "Hello, Server!"
client.send(data.encode())

# Receive and print the server's response
response = client.recv(1024).decode()
print(f"Server response: {response}")

# Close the client socket
client.close()