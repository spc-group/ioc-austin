import logging
import time

# Set up logging
log = logging.getLogger(__name__)

class RobotDriver():

    def transfer(self, pos1, pos2):
        print(f"Moving robot from {pos1} to {pos2}.")
        # Move the robot here. ``time.sleep`` is a placeholder for the
        # robot doing slow things.
        time.sleep(5)
        print("done", flush=True)
