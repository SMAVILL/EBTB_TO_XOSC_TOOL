import os
import sys

MAIN_PATH = os.path.abspath(os.path.join(__file__, "..", ".."))
CURRENT_WORKING_FILE_DIRECTORY = os.path.abspath(os.path.join(__file__))
CURRENT_WORKING_DIRECTORY = os.path.abspath(os.path.join(__file__, ".."))

if MAIN_PATH not in sys.path:
    sys.path.append(MAIN_PATH)

dir_path = os.path.dirname(os.path.realpath(__file__))
for root, dirs, files in os.walk(dir_path):
    sys.path.append(root)


XlmrFile = "XlmrFile"
VehicleLength = "VehicleLength"
VehicleWidth = "VehicleWidth"
ObjectID = "ObjectID"
Width = "Width"
Length = "Length"
Height = "Height"
AssetID = "AssetID"
LandmarkOffset = "LandmarkOffset"
Displacement = "Displacement"
LaneSelection = "LaneSelection"


