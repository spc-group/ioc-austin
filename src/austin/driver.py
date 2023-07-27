import logging
import time
from copy import deepcopy

import numpy as np

import socket

from urx import Robot, RobotException
import austin.robotiq_gripper as robotiq_gripper


# Set up logging
log = logging.getLogger(__name__)

# Input for parameters
homej0 = [-23.18, -78.18, 137.67, -153.64, -88.58, 284.37]  # unit: degree


#
class RobotDisconnected(ConnectionError):
    ...


class RobotDriver:
    robot_ip: str = ""
    port: str = ""
    is_connected: bool = False

    def __init__(self, robot_ip, port, timeout):
        self.robot_ip = robot_ip
        self.port = port
        # Create the socket object
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(timeout)
        self.connect()

        self.ur = None

        self.gripper = None
        self.gripper_pos_opn = 40
        self.gripper_vel = 0.5
        self.gripper_for = 0.2

    def connect(self):
        try:
            self.ur = self.sock.connect((self.robot_ip, self.port))
        except Exception as exp:
            msg = f"Could not connect to robot: {self.robot_ip}"
            log.error(msg)
            self.is_connected = False
        else:
            self.is_connected = True
            # Receive initial "Connected" Header
            self.sock.recv(1096)

    def send_and_receive(self, command):
        try:
            self.sock.sendall((command + "\n").encode())
            return self.get_reply()
        except (
            ConnectionResetError,
            ConnectionAbortedError,
            BrokenPipeError,
            TimeoutError,
        ):
            msg = f"The connection was lost to the robot ({self.robot_ip}:{self.port})."
            msg += " Please connect and try running again."
            log.warning(msg)
            raise RobotDisconnected(msg)

    def get_reply(self):
        """Read one line from the socket.

        Returns
        =======

        response
          text until new line

        """
        collected = b""
        while True:
            part = self.sock.recv(1)
            if part != b"\n":
                collected += part
            elif part == b"\n":
                break
        return collected.decode("utf-8")

    # gripper functions
    def activate_gripper(self):
        """
        activate Hand-E gripper
        """
        try:
            # GRIPPER SETUP:
            self.gripper = robotiq_gripper.RobotiqGripper()
            print("Connecting to gripper...")
            self.gripper.connect(self.robot_ip, self.port)

        except Exception as err:
            print("Gripper error: ", err)

        else:
            if self.gripper.is_active():
                print("Gripper already active")
            else:
                print("Activating gripper...")
                self.gripper.activate()
                print("Opening gripper...")
                self.gripper.move_and_wait_for_pos(
                    self.gripper_pos_opn, self.gripper_vel, self.gripper_frc
                )

    def gripper_act_status(self):
        """
        check gripper being actived or not
        """
        return self.gripper.is_active()

    def disconnect_gripper(self):
        """
        disconnect Hand-E gripper
        """
        return self.gripper.disconnect()

    def gripper_cls_position(self):
        """
        gripper minimum position
        """
        return self.gripper.get_closed_position()

    def gripper_opn_position(self):
        """
        gripper maximum position
        """
        return self.gripper.get_open_position()

    def gripper_cal(self):
        """
        calibrate gripper position
        """
        return self.gripper.auto_calibrate()

    def gripper_cur_position(self):
        """
        get gripper current postion
        """
        return self.gripper.get_current_position()

    def gripper_move(self, gripper_pos=30, gripper_vel=0.5, gripper_frc=0.2):
        """
        move gripper to a postion with speed and force in percentage
        """
        return self.gripper.move_and_wait_for_pos(gripper_pos, gripper_vel, gripper_frc)

    # transfer functions
    def get_joint_angles(self):
        """
        get current joint angles
        """
        return self.connect.getj()

    def movej(self, home_loc=homej0, acc=0.5, vel=0.2, wait=True):
        """
        Description: Moves the robot to the home location.
        """
        print("Homing the robot...")
        return self.ur.movej(home_loc, acc, vel, wait=True)
        print("Robot moved to home location")

    def pickj(
        self,
        pick_goal,
        acc=0.5,
        vel=0.2,
        wait=True,
        gripper_pos_opn=40,
        gripper_pos_cls=2,
        gripper_vel=40,
        gripper_frc=20,
    ):
        """Pick up from first goal position"""
        above_goal = deepcopy(pick_goal)
        above_goal[2] += 0.05

        print("Moving to above goal position")
        self.ur.movel(above_goal, acc, vel, wait=True)

        print("Opening gripper")
        self.gripper.move_and_wait_for_pos(gripper_pos_opn, gripper_vel, gripper_frc)

        print("Moving to goal position")
        self.ur.movel(pick_goal, acc, vel, wait=True)

        print("Closing gripper")
        self.gripper.move_and_wait_for_pos(gripper_pos_cls, gripper_vel, gripper_frc)

        print("Moving back to above goal position")
        self.ur.movel(above_goal, acc, vel, wait=True)

    def placej(
        self,
        place_goal,
        acc=0.5,
        vel=0.2,
        wait=True,
        gripper_pos_opn=40,
        gripper_pos_cls=2,
        gripper_vel=40,
        gripper_frc=20,
    ):
        """Place down at second goal position"""

        above_goal = deepcopy(place_goal)
        above_goal[2] += 0.05

        print("Moving to above goal position")
        self.ur.movej(above_goal, acc, vel, wait=True)

        print("Moving to goal position")
        self.ur.movej(place_goal, acc, vel, wait=True)

        print("Opennig gripper")
        self.gripper.move_and_wait_for_pos(gripper_pos_opn, gripper_vel, gripper_frc)

        print("Moving back to above goal position")
        self.ur.movej(above_goal, acc, vel, wait=True)
