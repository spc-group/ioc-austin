import logging
import time
import socket


# Set up logging
log = logging.getLogger(__name__)


class RobotDisconnected(ConnectionError):
    ...


class RobotDriver():
    robot_ip: str = ""
    port: str = ""

    def __init__(self, robot_ip, port, timeout):
        self.robot_ip = robot_ip
        self.port = port
        # Create the socket object
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(timeout)
        self.connect()

    def connect(self):
        self.sock.connect((self.robot_ip, self.port))
        # Receive initial "Connected" Header
        self.sock.recv(1096)

    def send_and_receive(self, command):
        try:
            self.sock.sendall((command + '\n').encode())
            return self.get_reply()
        except (ConnectionResetError, ConnectionAbortedError):
            msg = 'The connection was lost to the robot. Please connect and try running again.'
            log.error(msg)
            raise RobotDisconnected(msg)

    def get_reply(self):
        """Read one line from the socket.

        Returns
        =======

        response
          text until new line

        """
        collected = b''
        while True:
            part = self.sock.recv(1)
            if part != b"\n":
                collected += part
            elif part == b"\n":
                break
        return collected.decode("utf-8")

    def mood(self):
        """Joke routine. How is the robot feeling right now?"""
        moods = ["bored", "happy", "sad", "sleepy", "hungry"]
        now = time.time()
        mood_idx = int((now / 10) % len(moods))
        return moods[mood_idx]

    def transfer(self, pos1, pos2):
        print(f"Moving robot from {pos1} to {pos2}.")
        # Move the robot here. ``time.sleep`` is a placeholder for the
        # robot doing slow things.
        time.sleep(5)
        print("done", flush=True)
