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


class VehicleControlMovements:
    """VehicleControlMovements"""
    def __init__(self, max_speed=100):
        self.max_speed = max_speed  # maximum speed in km/hr

    def place_ahead(self, value):
        """ Place the vehicle ahead by a given distance in meters. """
        return abs(value)

    def place_behind(self, value):
        """ Place the vehicle behind by a given distance in meters. """
        return -abs(value)

    def move_backward(self, value):
        """ Moves the vehicle backward by the specified distance in meters. """
        return -abs(value)

    def move_forward(self, value):
        """ Moves the vehicle forward by the specified distance in meters. """
        return abs(value)

    def stay_still(self):
        """ Vehicle remains stationary. """
        return 0

    def move_left(self, value):
        """ Moves the vehicle left by the specified distance in meters. """
        return abs(value)

    def move_right(self, value):
        """ Moves the vehicle right by the specified distance in meters. """
        return abs(value)

    def center_lane(self):
        """ Centers the vehicle in the lane. """
        return 0

    def left_position(self, value):
        """ Adjusts the vehicle to a position 'value' meters to the left of the current lane center. """
        return abs(value)

    def right_position(self, value):
        """ Adjusts the vehicle to a position 'value' meters to the right of the current lane center. """
        return -abs(value)

    def split_string(self, input_string):
        """ Splits a string into alphabetic and numeric parts, converting numeric strings to integers. """
        parts = re.findall(r'[A-Za-z]+|\d+', input_string)
        return [int(part) if part.isdigit() else part for part in parts]
