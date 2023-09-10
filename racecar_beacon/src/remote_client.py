#!/usr/bin/env python

import socket
import time
import struct

HOST = '10.0.2.1'
# This process should listen to a different port than the PositionBroadcast client.
PORT = 65432

# Create a socket and connect to the server
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # IPv4, TCP
client.connect((HOST, PORT))

# Send data to the server
data = "RPOS"
client.sendall(data.encode())
# Receive and print the server's response
response = client.recv(1024)
response = struct.unpack("3f i", response)
pos = response[:3]
print("Server response for RPOS: "+ str(pos))


# Send data to the server
data = "OBSF"
client.sendall(data.encode())
# Receive and print the server's response
response = client.recv(1024)
response = struct.unpack("4i", response)
obstacle = response[0]
print("Server response for OBSF: "+ str(obstacle))


# Send data to the server
data = "RBID"
client.sendall(data.encode())
# Receive and print the server's response
response = client.recv(1024)
response = struct.unpack("4i", response)
ID = response[0]
print("Server response for RBID: "+ str(ID))


# Close the client socket
client.close()
