# Ryan Sauer
# CIS 427
# Socket Program
# This program is the client script for the yamotd protocol

from socket import *


MSGS= ["MSGGET\n", "MSGSTORE\n", "QUIT\n"]  # Predefined yamotd messages
DELIM = '\n'                                # Response Delim

SERVER_NAME = ""                            # Server IP (User defiend)
SERVER_PORT = 11427                         # Server Port (Predefined)
CLIENT_SOCKET = socket(AF_INET, SOCK_STREAM)    # Client Socket Creation


# Sends packet and decodes response
# RETURNS: decoded server response
def send_packet(PACKET):
    CLIENT_SOCKET.send(PACKET.encode())
    server_packet = CLIENT_SOCKET.recv(1024)
    server_message = server_packet.decode().split(DELIM)
    return server_message


# MSGGET Protocol
def get_pkt():
    server_message = send_packet(MSGS[0])
    if server_message[0] == '200 OK':           # If 200, print message
        print(server_message[1])
    else:                                       # Else print error
        print("Error:\n" + server_message[0])
    
# MSGSTORE Protocol
def store_pkt(NEW_MESSAGE):
    server_message = send_packet(MSGS[1])
    if server_message[0] == '200 OK':           # If 200, send new message
        server_message = send_packet(NEW_MESSAGE + '\n')
    else:                                       # Else print error
        print("Error:\n" + server_message[0])
        return
    print(server_message[0])                    # Prints server response

# QUIT Protocol
def quit_pkt():
    server_message = send_packet(MSGS[2])
    print(server_message[0])                    # Prints server response
    if server_message[0] == '200 OK':           # Succesful disconnects from server
        print("Client succesfully disconnected from server")
        CLIENT_SOCKET.close()
        return True
    else:
        return False


print("YAMOTD Protocol Initiated")
SERVER_NAME = str(input("Please input the server you wish to communicate with: "))

CLIENT_SOCKET.connect((SERVER_NAME, SERVER_PORT))

while True:
    USER_INPUT = input("[1] MSGGET\n[2] MSGSTORE\n[3] QUIT\nWhat kind of message would you like to send: ").upper()

    if USER_INPUT == '1' or USER_INPUT == 'MSGGET':
        get_pkt()
    elif USER_INPUT == '2' or USER_INPUT == 'MSGSTORE':
        NEW_MSG = input("Please input a new message of the day: ").replace('\n', ' ')
        store_pkt(NEW_MSG)
    elif USER_INPUT == '3' or USER_INPUT == 'QUIT':
        if quit_pkt():
            break
    else:
        print(USER_INPUT + ' is an unkown message type')
