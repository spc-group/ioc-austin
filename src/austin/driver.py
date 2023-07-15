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
homej0 = [1,2,3,4,5,6]

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
        self.acceleration = 0.5
        self.velocity = 0.2
               
        self.gripper = None
        self.gripper_close = 2 # 0 is closed
        self.gripper_open = 40
        self.gripper_speed = 40 # 0-50
        self.gripper_force = 20 # 0-100
        

        

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

    # transfer functions 
    def get_joint_angles(self):
        return self.connect.getj()

    def homej(self, home_location = None):
        """
        Description: Moves the robot to the home location.
        """
        print("Homing the robot...")
        if home_location:
            home_loc = home_location
        else:
            home_loc = homej0
        self.ur.movej(home_loc, self.acceleration, self.velocity, wait=True)
        print("Robot moved to home location")
    
    def pickj(self, pick_goal):
        '''Pick up from first goal position'''
        above_goal = deepcopy(pick_goal)
        above_goal[2] += 0.05  
    
        print('Moving to above goal position')
        self.ur.movel(above_goal, self.acceleration, self.velocity, wait=True)
    
        print('Opening gripper')
        self.gripper.move_and_wait_for_pos(self.gripper_open, self.gripper_speed, self.gripper_force)    
    
        print('Moving to goal position')
        self.ur.movel(pick_goal, self.acceleration, self.velocity, wait=True)
    
        print('Closing gripper')
        self.gripper.move_and_wait_for_pos(self.gripper_close, self.gripper_speed, self.gripper_force)
    
        print('Moving back to above goal position')
        self.ur.movel(above_goal, self.acceleration, self.velocity, wait=True)

    def placej(self, place_goal):

        '''Place down at second goal position'''

        above_goal = deepcopy(place_goal)
        above_goal[2] += 0.05

        print('Moving to above goal position')
        self.ur.movej(above_goal, self.acceleration, self.velocity, wait=True)
        
        print('Moving to goal position')
        self.ur.movej(place_goal, self.acceleration, self.velocity, wait=True)
        
        print('Opennig gripper')
        self.gripper.move_and_wait_for_pos(self.griper_open, self.gripper_speed, self.gripper_force)        

        print('Moving back to above goal position')
        self.ur.movej(above_goal, self.acceleration, self.velocity, wait=True)

        print('Moving to home position')
        self.ur.movej(self.home_joint, self.acceleration, self.velocity, wait=True)    
    
    # gripper functions     
    def activate_gripper(self):
        """
        activate Hand-E gripper
        """
        try:
            # GRIPPER SETUP:
            self.gripper = robotiq_gripper.RobotiqGripper()
            print('Connecting to gripper...')
            self.gripper.connect(self.robot_ip, self.port)
    
        except Exception as err:
            print("Gripper error: ", err)
    
        else:
            if self.gripper.is_active():
                print('Gripper already active')
            else:
                print('Activating gripper...')
                self.gripper.activate()
                print('Opening gripper...')
                self.gripper.move_and_wait_for_pos(self.gripper_open, self.gripper_speed, self.gripper_force)
                
    def gripper_act_status(self):
        self.gripper.is_active
    
    def disconnect_gripper(self):
        """
        disconnect Hand-E gripper
        """
        self.gripper.disconnect
        
    def gripper_cls_position(self):
        self.gripper.get_closed_position
        
    def gripper_opn_position(self):
        self.gripper.get_open_position
        
    def gripper_cal(self):
        self.gripper.auto_calibrate
        
    def gripper_cur_position(self):
        self.gripper.get_current_position
        
    def gripper_move(self, position:int, speed:int, force:int):
        self.gripper.move_and_wait_for_pos(position, speed, force)
        
        
        
    
        