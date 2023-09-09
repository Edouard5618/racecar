#!/usr/bin/env python

import socket

HOST = ''
# This process should listen to a different port than the PositionBroadcast client.
PORT = 65431

# Create a socket and connect to the server
client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # IPv4, UDP
client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

client.bind((HOST, PORT))

# Receive and print the server's response
while True:
    print("Wait for message")
    response, source_address = client.recvfrom(1024)
    print("Server response:"+ str(response.decode()))
    if response == "EXIT":
        break



# Close the client socket
client.close()
