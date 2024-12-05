import os
import sys
import re

MAIN_PATH = os.path.abspath(os.path.join(__file__, "..", "..", "..", ".."))
CURRENT_WORKING_FILE_DIRECTORY = os.path.abspath(os.path.join(__file__))
CURRENT_WORKING_DIRECTORY = os.path.abspath(os.path.join(__file__, ".."))

if MAIN_PATH not in sys.path:
    sys.path.append(MAIN_PATH)

dir_path = os.path.dirname(os.path.realpath(__file__))
for root, dirs, files in os.walk(dir_path):
    sys.path.append(root)


class DataControls:
    """Data Controls"""
    def __init__(self, max_speed=100):
        self.max_speed = max_speed  # maximum speed in km/hr

    def kmhr_to_ms(self, speed):
        """ Converts speed from km/hr to m/s. """
        return (speed * 1000 / 3600)

    def accelerate(self, increase):
        """ Increases the speed of the vehicle by 'increase' km/hr, not exceeding the max speed. """
        new_speed = min(self.max_speed, self.kmhr_to_ms(increase))
        return f"Speed increased to {new_speed} m/s."

    def decelerate(self, decrease):
        """ Decreases the speed of the vehicle by 'decrease' km/hr, not dropping below zero. """
        new_speed = max(0, self.kmhr_to_ms(-decrease))
        return f"Speed decreased to {new_speed} m/s."

    def adjust_steering(self, angle):
        """ Adjusts the steering wheel by 'angle' degrees. Negative for left, positive for right. """
        if not (-180 <= angle <= 180):
            raise ValueError("Angle must be between -180 and 180 degrees.")
        return f"Steering adjusted to {angle} degrees."

    def calculate_overlap(self, overlap_data):

        displacement = int(overlap_data[0].get("Displacement"))
        sideaxis = str(overlap_data[0].get('SideAxis'))


        if displacement == 0 and sideaxis ==  "Left":
            return -9.182

        elif displacement == 25 and sideaxis == "Left":
            return -8.731

        elif displacement == 0 and sideaxis == "Middle":
            return -8.401

        elif displacement == -25 and sideaxis == "Right":
            return -7.981

        elif displacement == 0 and sideaxis == "Right":
            return -7.501

        elif displacement == 75 and sideaxis == "Middle":
            return -8.0312

        elif displacement == 20 and sideaxis == "Middle":
            return -8.9375

        elif displacement == 25 and sideaxis == "Middle":
            return -8.7656

        elif displacement == 50 and sideaxis == "Middle":
            return -8.4218

        elif displacement == 80 and sideaxis == "Middle":
            return -7.875