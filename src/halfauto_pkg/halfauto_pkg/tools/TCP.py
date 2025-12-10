#!/usr/bin/env python3
"""Standalone TCP server module (ROS removed).

Provides TcpServer class (unchanged) and a helper
`send_delivery_command` to send a message and wait for reply.

Also includes a simple CLI when run as __main__ to start the server,
accept a client, and interactively send messages.
"""
import argparse
import logging
import socket
import select

class TcpServer:
    def __init__(self, port, allowed_ip):
        self.server_port = port
        self.allowed_ip = allowed_ip
        self.server_socket = None
        self.client_socket = None
        self.timeout_delivery_cmd = 100
        self.timeout_door_open = 100

    def start(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(("0.0.0.0", self.server_port))
        self.server_socket.listen(1)
        return True

    def accept_client(self):
        while True:
            client_sock, client_addr = self.server_socket.accept()
            client_ip = client_addr[0]
            if client_ip == self.allowed_ip:
                self.client_socket = client_sock
                return True
            else:
                client_sock.close()

    def send_message(self, msg):
        if self.client_socket is None:
            return False
        sent = self.client_socket.send(msg.encode())
        return sent == len(msg)

    def receive_message(self, timeout_sec):
        if self.client_socket is None:
            return ""
        ready = select.select([self.client_socket], [], [], timeout_sec)
        if ready[0]:
            data = self.client_socket.recv(1024)
            return data.decode()
        return ""

    def close_socket(self):
        if self.client_socket:
            self.client_socket.close()
            self.client_socket = None
        if self.server_socket:
            self.server_socket.close()
            self.server_socket = None

    def clear_socket_buffer(self):
        if self.client_socket is None:
            return
        self.client_socket.setblocking(0)
        try:
            while True:
                data = self.client_socket.recv(1024)
                if not data:
                    break
        except Exception:
            pass
        self.client_socket.setblocking(1)

g_tcp_server = None

def send_delivery_command(tcp_server: TcpServer, message: str, timeout: int = None):
    """Send a delivery message to the connected client and wait for a reply.

    Args:
        tcp_server: an instance of TcpServer with an accepted client.
        message: the message string to send (will be encoded to bytes).
        timeout: optional timeout seconds to wait for reply. If None, selects
                 based on message content (door commands use door timeout).

    Returns:
        (success: bool, reply: str). If success is False, reply may be empty.
    """
    if tcp_server is None:
        logging.error("tcp server not available")
        return False, ""

    tcp_server.clear_socket_buffer()
    if not tcp_server.send_message(message):
        logging.error("Failed to send message to MCU: %s", message)
        return False, ""

    t = timeout if timeout is not None else tcp_server.timeout_delivery_cmd
    if "door" in message or "bigDoorOpen" in message:
        t = tcp_server.timeout_door_open

    reply = tcp_server.receive_message(t)
    if not reply:
        logging.error("Timeout or no response for: %s", message)
        return False, ""

    logging.info("MCU reply: %s", reply)
    return True, reply


def main():
    parser = argparse.ArgumentParser(description="TCP server for MCU (no ROS)")
    parser.add_argument("--port", type=int, default=5001, help="server port")
    parser.add_argument("--allowed-ip", type=str, default="192.168.57.111",
                        help="allowed client IP (only accept this)")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")

    tcp_server = TcpServer(args.port, args.allowed_ip)
    if not tcp_server.start():
        logging.error("Failed to start TCP server.")
        return 2

    logging.info("Waiting for MCU (client) to connect on port %s...", args.port)
    try:
        if not tcp_server.accept_client():
            logging.error("Failed to accept client.")
            return 3
    except KeyboardInterrupt:
        logging.info("Interrupted while waiting for client.")
        tcp_server.close_socket()
        return 1

    logging.info("MCU connected from %s", tcp_server.client_socket.getpeername())

    try:
        # Interactive loop: read lines from stdin and send to MCU
        logging.info("Enter messages to send to MCU. Ctrl-C or EOF to quit.")
        while True:
            try:
                line = input('> ')
            except EOFError:
                break

            line = line.strip()
            if not line:
                continue

            success, reply = send_delivery_command(tcp_server, line)
            if success:
                print(f"REPLY: {reply}")
            else:
                print("No reply or failed to send message")

    except KeyboardInterrupt:
        logging.info("Shutting down on user request")
    finally:
        tcp_server.close_socket()


if __name__ == '__main__':
    main()
