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

# Object Actions
Obj_ChangeLane = "Obj_ChangeLane"
Obj_Deactivate = "Obj_Deactivate"
Obj_Initialize = "Obj_Initialize"
Obj_PositionWaypointTrajectoryAbsSys = "Obj_PositionWaypointTrajectoryAbsSys"
Obj_PositionWaypointTrajectoryAbsSysLane = "Obj_PositionWaypointTrajectoryAbsSysLane"
Obj_SetLateralDisplacement = "Obj_SetLateralDisplacement"
Obj_SetLateralReference = "Obj_SetLateralReference"
Obj_SetLateralRelativePosition = "Obj_SetLateralRelativePosition"
Obj_SetLongitudinalPosition = "Obj_SetLongitudinalPosition"
Obj_SetLongitudinalReference = "Obj_SetLongitudinalReference"
Obj_SetLongitudinalRelativePosition = "Obj_SetLongitudinalRelativePosition"
Obj_SetLongitudinalSpeed = "Obj_SetLongitudinalSpeed"

# Object
ObjP_Setup = "ObjP_Setup"

