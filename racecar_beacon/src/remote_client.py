#!/usr/bin/env python

import socket

#HOST = '127.0.0.1'
HOST = '192.168.137.215'
# This process should listen to a different port than the PositionBroadcast client.
PORT = 65432

# Create a socket and connect to the server
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # IPv4, TCP
client.connect(HOST, PORT)

# Send data to the server
data = "RPOS"
client.sendall(data.encode())

# Receive and print the server's response
response = client.recv(1024).decode()
print("Server response:", response)

#except socket.error as e:
#print("Erreur de connexion au serveur :", e)


# Close the client socket
client.close()
