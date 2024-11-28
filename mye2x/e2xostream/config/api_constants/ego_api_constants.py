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

# Driver Actions
Dri_AcknowledgeCPMToastMessage = "Dri_AcknowledgeCPMToastMessage"
Dri_ChangeACCSpeed = "Dri_ChangeACCSpeed"
Dri_ChangeVSLSpeed = "Dri_ChangeVSLSpeed"
Dri_ConfigureCollisionAvoidanceFunction = "Dri_ConfigureCollisionAvoidanceFunction"
Dri_ConfigureDrivingFunction = "Dri_ConfigureDrivingFunction"
Dri_ConfigureParkingFunction = "Dri_ConfigureParkingFunction"
Dri_ParkAppClose = "Dri_ParkAppClose"
Dri_ParkAppStart = "Dri_ParkAppStart"
Dri_PlaceHandCloseToDoorHandle = "Dri_PlaceHandCloseToDoorHandle"
Dri_PrepareVehicle = "Dri_PrepareVehicle"
Dri_PressAPASoftKey = "Dri_PressAPASoftKey"
Dri_PressRMASoftKey = "Dri_PressRMASoftKey"
Dri_PressSteeringWheelButton = "Dri_PressSteeringWheelButton"
Dri_SetACCDistance = "Dri_SetACCDistance"
Dri_SetAccelerationPedal = "Dri_SetAccelerationPedal"
Dri_SetBeltState = "Dri_SetBeltState"
Dri_SetBrakePedal = "Dri_SetBrakePedal"
Dri_SetCarwashMode = "Dri_SetCarwashMode"
Dri_SetDrivingProgram = "Dri_SetDrivingProgram"
Dri_SetElectricalTrailerHitchPosition = "Dri_SetElectricalTrailerHitchPosition"
Dri_SetESPMode = "Dri_SetESPMode"
Dri_SetHazardWarningLights = "Dri_SetHazardWarningLights"
Dri_SetIndicatorState = "Dri_SetIndicatorState"
Dri_SetLateralDisplacement = "Dri_SetLateralDisplacement"
Dri_SetLateralReference = "Dri_SetLateralReference"
Dri_SetLockState = "Dri_SetLockState"
Dri_SetLongitudinalReference = "Dri_SetLongitudinalReference"
Dri_SetLongitudinalSpeed = "Dri_SetLongitudinalSpeed"
Dri_SetMirrorPosition = "Dri_SetMirrorPosition"
Dri_SetParkDisplayView = "Dri_SetParkDisplayView"
Dri_SetParkingBrake = "Dri_SetParkingBrake"
Dri_SetParkSwitchState = "Dri_SetParkSwitchState"
Dri_SetSteeringWheelAngle = "Dri_SetSteeringWheelAngle"
Dri_SetSteeringWheelForce = "Dri_SetSteeringWheelForce"
Dri_SetSteeringWheelTorque = "Dri_SetSteeringWheelTorque"
Dri_SetVehicleDoor = "Dri_SetVehicleDoor"
Dri_SetVehicleSpeedUnit = "Dri_SetVehicleSpeedUnit"
Dri_SetWinterTireSpeedLimiter = "Dri_SetWinterTireSpeedLimiter"
Dri_StepOut = "Dri_StepOut"
Dri_SwitchACCVSLMode = "Dri_SwitchACCVSLMode"
Dri_SwitchGear = "Dri_SwitchGear"
Dri_SwitchToACCDriving = "Dri_SwitchToACCDriving"
Dri_SwitchToVSLDriving = "Dri_SwitchToVSLDriving"
Dri_TakeHandsOff = "Dri_TakeHandsOff"

# System Actions
Sys_ReleaseAcceleratorPedalRequest = "Sys_ReleaseAcceleratorPedalRequest"
Sys_SetADASISv2Attribute = "Sys_SetADASISv2Attribute"
Sys_SetAttentionAssist = "Sys_SetAttentionAssist"
Sys_SetECOAssistRequest = "Sys_SetECOAssistRequest"
Sys_SetELVIRARecommendation = "Sys_SetELVIRARecommendation"
Sys_SetEPSHandsOffDetection = "Sys_SetEPSHandsOffDetection"
Sys_SetESPState = "Sys_SetESPState"
Sys_SetHandsOnCapacitiveSteeringDetection = "Sys_SetHandsOnCapacitiveSteeringDetection"
Sys_SetIgnitionState = "Sys_SetIgnitionState"
Sys_SetProductionMode = "Sys_SetProductionMode"
Sys_SetSnowChainMode = "Sys_SetSnowChainMode"
Sys_SetTireFriction = "Sys_SetTireFriction"
Sys_SetTireState = "Sys_SetTireState"
Sys_SetTrailerPlugState = "Sys_SetTrailerPlugState"
Sys_SetTransportMode = "Sys_SetTransportMode"

# System/Ego Vehicle
SysP_Vehicle = "SysP_Vehicle"

