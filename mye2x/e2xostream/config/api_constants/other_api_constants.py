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


# BUS Actions
Ethernet_SetSignal = "Ethernet_SetSignal"
Ethernet_InvalidateE2EProtection = "Ethernet_InvalidateE2EProtection"
Ethernet_SetSignalInvalid = "Ethernet_SetSignalInvalid"
Ethernet_SuspendPduTriggering = "Ethernet_SuspendPduTriggering"

# Environment Actions
Env_SetTrafficLightState = "Env_SetTrafficLightState"

# Time Based Actions
TBA_RunDiagnosticService = "TBA_RunDiagnosticService"
TBA_WriteEvaluationEvent = "TBA_WriteEvaluationEvent"

# Events
E_ADASState = "E_ADASState"
E_ChangeACCSpeed = "E_ChangeACCSpeed"
E_ChangeVSLSpeed = "E_ChangeVSLSpeed"
E_CompareSignal = "E_CompareSignal"
E_ConfigurationCollisionAvoidanceFunction = "E_ConfigurationCollisionAvoidanceFunction"
E_ConfigurationDrivingFunction = "E_ConfigurationDrivingFunction"
E_DiagnosticResult = "E_DiagnosticResult"
E_DistanceTimeBased = "E_DistanceTimeBased"
E_IDCSystemState = "E_IDCSystemState"
E_Landmark = "E_Landmark"
E_ObjectCollision = "E_ObjectCollision"
E_ObjectDistanceLaneBased = "E_ObjectDistanceLaneBased"
E_ParkAppActionFinished = "E_ParkAppActionFinished"
E_ParkingFinished = "E_ParkingFinished"
E_ParkingSpaceDetected = "E_ParkingSpaceDetected"
E_PrepareVehicle = "E_PrepareVehicle"
E_SetBeltState = "E_SetBeltState"
E_StepOut = "E_StepOut"
E_SwitchToACCDriving = "E_SwitchToACCDriving"
E_SwitchToVSLDriving = "E_SwitchToVSLDriving"
E_SysVehicleVelocity = "E_SysVehicleVelocity"
E_Time = "E_Time"
E_TimeToCollision = "E_TimeToCollision"

# Environment
EnvP_ParkingBay = "EnvP_ParkingBay"
EnvP_ParkingBayStyle = "EnvP_ParkingBayStyle"
EnvP_RoadNetwork = "EnvP_RoadNetwork"

