# Ryan Sauer
# CIS 427
# Socket Program
# This program is the client script for the yamotd protocol

from socket import *


SERVER_NAME = "192.168.11.133"                            # Server IP (User defiend)
SERVER_PORT = 10600                         # Server Port (Predefined)
CLIENT_SOCKET = socket(AF_INET, SOCK_STREAM)    # Client Socket Creation


def send_message(var):
    CLIENT_SOCKET.connect((SERVER_NAME, SERVER_PORT))
    CLIENT_SOCKET.send(str(var).encode())
