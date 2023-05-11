import logging
import time

# Set up logging
log = logging.getLogger(__name__)

class RobotDriver():

    def load_sample(self, value):
        log.info(f"Moving robot to {value}.")
        print(f"Moving robot to {value}...", flush=True)
        # Move the robot here. ``time.sleep`` is a placeholder for the
        # robot doing slow things.
        time.sleep(20)
        print("done", flush=True)
