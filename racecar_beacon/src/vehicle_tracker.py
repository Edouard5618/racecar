#!/usr/bin/env python

import socket
import struct

HOST = ''
# This process should listen to a different port than the PositionBroadcast client.
PORT = 65431

# Create a socket and connect to the server
client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # IPv4, UDP
client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

client.bind((HOST, PORT))
print("Wait for message")
# Receive and print the server's response
while True:
    response, source_address = client.recvfrom(1024)
    response = struct.unpack(">3f i", response)
    pos = response[:3]
    ID = response[3]
    print("Server response pos: "+ str(pos) + " and ID: "+ str(ID))
    
    if response == "EXIT":
        break


# Close the client socket
client.close()
