import logging
import time


# Set up logging
log = logging.getLogger(__name__)

class RobotDriver():

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
