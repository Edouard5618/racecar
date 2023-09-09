#!/usr/bin/env python

import socket

HOST = '192.168.137.215'
# This process should listen to a different port than the PositionBroadcast client.
PORT = 65431

# Create a socket and connect to the server
client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # IPv4, UDP

client.connect((HOST, PORT))

# Receive and print the server's response
while True:
    response = client.recv(1024).decode()
    print("Server response:"+ str(response))
    if response == "EXIT":
        break



# Close the client socket
client.close()