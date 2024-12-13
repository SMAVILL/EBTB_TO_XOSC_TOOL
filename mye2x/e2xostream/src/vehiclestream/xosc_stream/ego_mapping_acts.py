import os
import sys
from e2xostream.stk.scenariogeneration import xosc, prettyprint, ScenarioGenerator
from e2xostream.stk.vehicledynamics.VehicleControl import VehicleControlMovements as vehiclecontrol
from e2xostream.stk.vehicledynamics.DataControl import DataControls as datacontrol
from e2xostream.stk.vehicledynamics.VehicleScenarioSetup import VehicleScenario
from e2xostream.src.vehiclestream.ebtb_stream import EBTBAnalyzer, EBTB_API_data
from e2xostream.config import default_properties, global_parameters, settings
from e2xostream.config.api_constants import (api_methods_constants as ApiMethods,
                                             ego_api_constants as EgoAPI,
                                             obj_api_constants as ObjAPI,
                                             other_api_constants as OtherAPI)
from e2xostream.src.scenario_generator import basescenario as BS
from e2xostream.src.acts_algo import ego_acts, obj_acts
from e2xostream.stk.fun_register_dispatch import FunctionRegisterDispatcher
from e2xostream.src.E2X_Convert import E2XOStream


MAIN_PATH = os.path.abspath(os.path.join(__file__, "..", "..", "..", ".."))
CURRENT_WORKING_FILE_DIRECTORY = os.path.abspath(os.path.join(__file__))
CURRENT_WORKING_DIRECTORY = os.path.abspath(os.path.join(__file__, ".."))

if MAIN_PATH not in sys.path:
    sys.path.append(MAIN_PATH)

dir_path = os.path.dirname(os.path.realpath(__file__))
for root, dirs, files in os.walk(dir_path):
    sys.path.append(root)


class EgoScnearioActs:
    def __init__(self, egoname, states_analysis, paramlist_analysis, state_events, param_events, esmini_path):

        self.states_analysis = states_analysis
        self.paramlist_analysis = paramlist_analysis
        self.state_events = state_events
        self.param_events = param_events
        self.esmini_path = esmini_path
        self.egoname = egoname
        self.open_scenario_version = 0
        self.VehicleControls = vehiclecontrol()
        self.Data_Controls = datacontrol()
        self.VehicleDefines = VehicleScenario()
        self.EGO_algo_acts = ego_acts.Ego_Acts(egoname, states_analysis, paramlist_analysis,
                                               state_events, param_events, esmini_path)
        self.register_dispatch = FunctionRegisterDispatcher()




    def __call__(self, all_ego_events):
        self.maneuver_api_mapping()
        self.check_api_dispatch_function(all_ego_events)

    def maneuver_api_mapping(self):

        self.register_dispatch.register(EgoAPI.Dri_PrepareVehicle, self.prepare_vehicle)
        self.register_dispatch.register(OtherAPI.E_PrepareVehicle,self.E_PrepareVehicle)
        self.register_dispatch.register(EgoAPI.Dri_SetLongitudinalSpeed,self.initial_acceleration)
        self.register_dispatch.register(EgoAPI.Dri_SetAccelerationPedal, self.throttle_acts)
        self.register_dispatch.register(EgoAPI.Dri_SetBrakePedal, self.brake_acts)
        self.register_dispatch.register(EgoAPI.Dri_SetLateralDisplacement, self.dri_setLateralDisplacement)
        self.register_dispatch.register(EgoAPI.Dri_SetLateralReference, self.dri_setLateralReference)
        #self.register_dispatch.register(OtherAPI.E_ObjectDistanceLaneBased, self.ego_ObjectDistanceLaneBased)
        self.register_dispatch.register(OtherAPI.E_Time, self.e_time)
        self.register_dispatch.register(OtherAPI.E_SysVehicleVelocity, self.E_sysvehiclevelocity)
        self.register_dispatch.register(OtherAPI.E_DistanceTimeBased, self.ego_E_DistanceTimeBased)
        self.register_dispatch.register(OtherAPI.E_ObjectCollision,self.ego_Eobjectcollision)
        self.register_dispatch.register(OtherAPI.E_TimeToCollision, self.ego_E_timetocollision)
        self.register_dispatch.register(OtherAPI.E_Landmark,self.ego_E_Landmark)
        self.register_dispatch.register(EgoAPI.Dri_SetIndicatorState,self.dri_setIndicatorState)
        self.register_dispatch.register(EgoAPI.Dri_SetVehicleDoor,self.ego_dri_SetVehicleDoor)
        self.register_dispatch.register(EgoAPI.Dri_SwitchGear, self.ego_dri_SwitchGear)
        self.register_dispatch.register(OtherAPI.Ethernet_SetSignal,self.ego_ethernet_setsignal)
        self.register_dispatch.register(OtherAPI.Ethernet_InvalidateE2EProtection, self.ego_ethernet_invalidateE2E)
        self.register_dispatch.register(OtherAPI.Ethernet_SetSignalInvalid, self.ego_ethernet_setsignalinvalid)
        self.register_dispatch.register(OtherAPI.Ethernet_SuspendPduTriggering, self.ego_ethernet_suspendPDUTriggering)
        self.register_dispatch.register(EgoAPI.Dri_SetParkingBrake, self.ego_dri_SetParkingBrake)
        self.register_dispatch.register(EgoAPI.Dri_SetSteeringWheelAngle, self.ego_dri_SetSteeringWheelAngle)
        self.register_dispatch.register(EgoAPI.Dri_AcknowledgeCPMToastMessage, self.ego_dri_AcknowledgeCPMToastMessage)
        self.register_dispatch.register(EgoAPI.Dri_ChangeACCSpeed, self.ego_Dri_ChangeACCSpeed)
        self.register_dispatch.register(EgoAPI.Dri_ChangeVSLSpeed, self.ego_Dri_ChangeVSLSpeed)
        self.register_dispatch.register(EgoAPI.Dri_ConfigureCollisionAvoidanceFunction, self.ego_Dri_ConfigureCollisionAvoidanceFunction)
        self.register_dispatch.register(EgoAPI.Dri_ConfigureDrivingFunction, self.ego_Dri_ConfigureDrivingFunction)
        self.register_dispatch.register(EgoAPI.Dri_ConfigureParkingFunction, self.ego_Dri_ConfigureParkingFunction)
        self.register_dispatch.register(EgoAPI.Dri_PlaceHandCloseToDoorHandle, self.ego_Dri_PlaceHandCloseToDoorHandle)
        self.register_dispatch.register(EgoAPI.Dri_PressAPASoftKey, self.ego_Dri_PressAPASoftKey)
        self.register_dispatch.register(EgoAPI.Dri_PressRMASoftKey, self.ego_Dri_PressRMASoftKey)
        self.register_dispatch.register(EgoAPI.Dri_PressSteeringWheelButton, self.ego_Dri_PressSteeringWheelButton)
        self.register_dispatch.register(EgoAPI.Dri_SetACCDistance, self.ego_Dri_SetACCDistance)
        self.register_dispatch.register(EgoAPI.Dri_SetBeltState, self.ego_Dri_SetBeltState)
        self.register_dispatch.register(EgoAPI.Dri_SetCarwashMode, self.ego_Dri_SetCarwashMode)
        self.register_dispatch.register(EgoAPI.Dri_SetDrivingProgram, self.ego_Dri_SetDrivingProgram)
        self.register_dispatch.register(EgoAPI.Dri_SetElectricalTrailerHitchPosition, self.ego_Dri_SetElectricalTrailerHitchPosition)
        self.register_dispatch.register(EgoAPI.Dri_SetESPMode, self.ego_Dri_SetESPMode)
        self.register_dispatch.register(EgoAPI.Dri_SetHazardWarningLights, self.ego_Dri_SetHazardWarningLights)
        self.register_dispatch.register(EgoAPI.Dri_SetLockState, self.ego_Dri_SetLockState)
        self.register_dispatch.register(EgoAPI.Dri_SetMirrorPosition, self.ego_Dri_SetMirrorPosition)
        self.register_dispatch.register(EgoAPI.Dri_SetParkDisplayView, self.ego_Dri_SetParkDisplayView)
        self.register_dispatch.register(EgoAPI.Dri_SetParkSwitchState, self.ego_Dri_SetParkSwitchState)
        self.register_dispatch.register(EgoAPI.Dri_SetVehicleSpeedUnit, self.ego_Dri_SetVehicleSpeedUnit)
        self.register_dispatch.register(EgoAPI.Dri_SetWinterTireSpeedLimiter, self.ego_Dri_SetWinterTireSpeedLimiter)
        self.register_dispatch.register(EgoAPI.Dri_SwitchACCVSLMode, self.ego_Dri_SwitchACCVSLMode)
        self.register_dispatch.register(EgoAPI.Dri_SwitchToACCDriving, self.ego_Dri_SwitchToACCDriving)
        self.register_dispatch.register(EgoAPI.Dri_SwitchToVSLDriving, self.ego_Dri_SwitchToVSLDriving)
        self.register_dispatch.register(EgoAPI.Dri_TakeHandsOff, self.ego_Dri_TakeHandsOff)
        self.register_dispatch.register(EgoAPI.Sys_ReleaseAcceleratorPedalRequest, self.ego_Sys_ReleaseAcceleratorPedalRequest)
        self.register_dispatch.register(EgoAPI.Sys_SetADASISv2Attribute, self.ego_Sys_SetADASISv2Attribute)
        self.register_dispatch.register(EgoAPI.Sys_SetAttentionAssist, self.ego_Sys_SetAttentionAssist)
        self.register_dispatch.register(EgoAPI.Sys_SetECOAssistRequest, self.ego_Sys_SetECOAssistRequest)
        self.register_dispatch.register(EgoAPI.Sys_SetELVIRARecommendation, self.ego_Sys_SetELVIRARecommendation)
        self.register_dispatch.register(EgoAPI.Sys_SetEPSHandsOffDetection, self.ego_Sys_SetEPSHandsOffDetection)
        self.register_dispatch.register(EgoAPI.Sys_SetESPState, self.ego_Sys_SetESPState)
        self.register_dispatch.register(EgoAPI.Sys_SetHandsOnCapacitiveSteeringDetection, self.ego_Sys_SetHandsOnCapacitiveSteeringDetection)
        self.register_dispatch.register(EgoAPI.Sys_SetIgnitionState, self.ego_Sys_SetIgnitionState)
        self.register_dispatch.register(EgoAPI.Sys_SetProductionMode, self.ego_Sys_SetProductionMode)
        self.register_dispatch.register(EgoAPI.Sys_SetSnowChainMode, self.ego_Sys_SetSnowChainMode)
        self.register_dispatch.register(EgoAPI.Sys_SetTireFriction, self.ego_Sys_SetTireFriction)
        self.register_dispatch.register(EgoAPI.Sys_SetTireState, self.ego_Sys_SetTireState)
        self.register_dispatch.register(EgoAPI.Sys_SetTrailerPlugState, self.ego_Sys_SetTrailerPlugState)
        self.register_dispatch.register(EgoAPI.Sys_SetTransportMode, self.ego_Sys_SetTransportMode)
        self.register_dispatch.register(OtherAPI.TBA_RunDiagnosticService, self.ego_TBA_RunDiagnosticService)
        self.register_dispatch.register(OtherAPI.TBA_WriteEvaluationEvent, self.ego_TBA_WriteEvaluationEvent)


    def check_api_dispatch_function(self, all_ego_events):
        for statekey, statevalue in self.states_analysis.items():
            ego_actions = statevalue.get("EgoActions", [])
            if not ego_actions:
                continue
            for egoaction in ego_actions:
                action = egoaction.get("Action")

                if action:
                    try:
                        self.register_dispatch.dispatch(action, all_ego_events,statekey)
                    except KeyError:
                        # Handle specific exception if the action is not found in the dispatcher
                        continue
                    except Exception as e:
                        # Log the exception if needed
                        print(f"Error dispatching action {action}: {e}")
                        continue

    flag = 0
    error_name = None

    def initial_acceleration(self, all_ego_events,state_key):
        if EgoScnearioActs.flag == 0:
        # initial Acceleration act
            self.EGO_algo_acts.ego_accelration_act(all_ego_events,state_key)

    def prepare_vehicle(self,all_ego_events,state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.prepare_vehicle(all_ego_events,state_key)

    #
    def ego_E_Landmark(self,all_ego_events,state_key):
        if EgoScnearioActs.flag == 0:
            result = self.EGO_algo_acts.E_landmark(all_ego_events, state_key)
            if result == "Stop":
                EgoScnearioActs.flag = 1
                EgoScnearioActs.error_name = "E_Landmark"



    def e_time(self,all_ego_events,state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.e_time(all_ego_events,state_key)
            # if self.EGO_algo_acts.e_time(all_ego_events, state_key) == "Stop":
            #     EgoScnearioActs.flag = 1
            # else:
            #     self.EGO_algo_acts.e_time(all_ego_events, state_key)

    def throttle_acts(self, all_ego_events,state_key):
        # Ego Throttle act
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_throttle_act(all_ego_events,state_key)

    def brake_acts(self, all_ego_events,state_key):
        # Ego brake act
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_brake_act(all_ego_events,state_key)

    def dri_setLateralDisplacement(self, all_ego_events,state_key):
        # dri_setLateralDisplacement
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Dri_SetLateralDisplacement(all_ego_events,state_key)

    def ego_E_DistanceTimeBased(self, all_ego_events,state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_E_DistanceTimeBased(all_ego_events,state_key)

    def dri_setLateralReference(self,all_ego_events,state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Dri_SetLateralReference(all_ego_events,state_key)

    def ego_ObjectDistanceLaneBased(self, all_ego_events,state_key):
        # E_ObjectDistanceLaneBased
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_E_ObjectDistanceLaneBased(all_ego_events,state_key)

    def ego_ObjectCollision(self, all_ego_events,state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_E_ObjectCollision(all_ego_events,state_key)

    def ego_dri_SwitchGear(self, all_ego_events,state_key):
        # Dri_SwitchGear
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Dri_SwitchGear(all_ego_events,state_key)

    def ego_dri_SetVehicleDoor(self, all_ego_events,state_key):
        # Dri_SetVehicleDoor
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Dri_SetVehicleDoor(all_ego_events,state_key)

    def ego_dri_SetParkingBrake(self, all_ego_events,state_key):
        # Dri_SetParkingBrake
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Dri_SetParkingBrake(all_ego_events,state_key)

    def ego_dri_SetSteeringWheelAngle(self, all_ego_events,state_key):
        # Dri_SetSteeringWheelAngle
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Dri_SetSteeringWheelAngle(all_ego_events,state_key)

    def ego_SetTrafficLightState(self, all_ego_events,state_key):
        # Env_SetTrafficLightState
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Env_SetTrafficLightState(all_ego_events,state_key)

    def E_sysvehiclevelocity(self, all_ego_events,state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_E_SysVehicleVelocity(all_ego_events,state_key)

    def dri_setIndicatorState(self,all_ego_events,state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.Dri_SetIndicatorState(all_ego_events,state_key)

    def ego_Eobjectcollision(self,all_ego_events,state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_E_ObjectCollision(all_ego_events,state_key)

    def ego_E_timetocollision(self,all_ego_events,state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_E_Timetocollision(all_ego_events,state_key)

    def ego_ethernet_setsignal(self,all_ego_events,state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_ethernet_setsignal(all_ego_events,state_key)

    def ego_ethernet_invalidateE2E(self,all_ego_events,state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_ethernet_invalidateE2E(all_ego_events, state_key)

    def ego_ethernet_setsignalinvalid(self,all_ego_events,state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_ethernet_setsignalinvalid(all_ego_events, state_key)

    def ego_ethernet_suspendPDUTriggering(self,all_ego_events,state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_ethernet_suspendPDUTriggering(all_ego_events, state_key)

    def ego_dri_AcknowledgeCPMToastMessage(self,all_ego_events,state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_dri_AcknowledgeCPMToastMessage(all_ego_events, state_key)

    def ego_Dri_ChangeACCSpeed(self,all_ego_events,state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Dri_ChangeACCSpeed(all_ego_events, state_key)

    def ego_Dri_ChangeVSLSpeed(self,all_ego_events,state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Dri_ChangeVSLSpeed(all_ego_events, state_key)

    def ego_Dri_ConfigureCollisionAvoidanceFunction(self,all_ego_events,state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Dri_ConfigureCollisionAvoidanceFunction(all_ego_events, state_key)

    def ego_Dri_ConfigureDrivingFunction(self,all_ego_events,state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Dri_ConfigureDrivingFunction(all_ego_events, state_key)

    def ego_Dri_ConfigureParkingFunction(self,all_ego_events,state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Dri_ConfigureParkingFunction(all_ego_events, state_key)

    def ego_Dri_PlaceHandCloseToDoorHandle(self,all_ego_events,state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Dri_PlaceHandCloseToDoorHandle(all_ego_events, state_key)

    def ego_Dri_PressAPASoftKey(self,all_ego_events,state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Dri_PressAPASoftKey(all_ego_events, state_key)

    def ego_Dri_PressRMASoftKey(self,all_ego_events,state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Dri_PressRMASoftKey(all_ego_events, state_key)

    def ego_Dri_PressSteeringWheelButton(self,all_ego_events,state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Dri_PressSteeringWheelButton(all_ego_events, state_key)

    def ego_Dri_SetACCDistance(self,all_ego_events,state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Dri_SetACCDistance(all_ego_events, state_key)

    def ego_Dri_SetBeltState(self,all_ego_events,state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Dri_SetBeltState(all_ego_events, state_key)

    def ego_Dri_SetCarwashMode(self,all_ego_events,state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Dri_SetCarwashMode(all_ego_events, state_key)

    def ego_Dri_SetDrivingProgram(self,all_ego_events,state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Dri_SetDrivingProgram(all_ego_events, state_key)

    def ego_Dri_SetElectricalTrailerHitchPosition(self,all_ego_events,state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Dri_SetElectricalTrailerHitchPosition(all_ego_events, state_key)

    def ego_Dri_SetESPMode(self,all_ego_events,state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Dri_SetESPMode(all_ego_events, state_key)

    def ego_Dri_SetHazardWarningLights(self,all_ego_events,state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Dri_SetHazardWarningLights(all_ego_events, state_key)

    def ego_Dri_SetLockState(self,all_ego_events,state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Dri_SetLockState(all_ego_events, state_key)

    def ego_Dri_SetMirrorPosition(self, all_ego_events, state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Dri_SetMirrorPosition(all_ego_events, state_key)

    def ego_Dri_SetParkDisplayView(self, all_ego_events, state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Dri_SetParkDisplayView(all_ego_events, state_key)

    def ego_Dri_SetParkSwitchState(self, all_ego_events, state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Dri_SetParkSwitchState(all_ego_events, state_key)

    def ego_Dri_SetVehicleSpeedUnit(self, all_ego_events, state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Dri_SetVehicleSpeedUnit(all_ego_events, state_key)

    def ego_Dri_SetWinterTireSpeedLimiter(self, all_ego_events, state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Dri_SetWinterTireSpeedLimiter(all_ego_events, state_key)

    def ego_Dri_SwitchACCVSLMode(self, all_ego_events, state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Dri_SwitchACCVSLMode(all_ego_events, state_key)

    def ego_Dri_SwitchToACCDriving(self, all_ego_events, state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Dri_SwitchToACCDriving(all_ego_events, state_key)

    def ego_Dri_SwitchToVSLDriving(self, all_ego_events, state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Dri_SwitchToVSLDriving(all_ego_events, state_key)

    def ego_Dri_TakeHandsOff(self, all_ego_events, state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Dri_TakeHandsOff(all_ego_events, state_key)

    def ego_Sys_ReleaseAcceleratorPedalRequest(self, all_ego_events, state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Sys_ReleaseAcceleratorPedalRequest(all_ego_events, state_key)

    def ego_Sys_SetADASISv2Attribute(self, all_ego_events, state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Sys_SetADASISv2Attribute(all_ego_events, state_key)

    def ego_Sys_SetAttentionAssist(self, all_ego_events, state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Sys_SetAttentionAssist(all_ego_events, state_key)

    def ego_Sys_SetECOAssistRequest(self, all_ego_events, state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Sys_SetECOAssistRequest(all_ego_events, state_key)

    def ego_Sys_SetELVIRARecommendation(self, all_ego_events, state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Sys_SetELVIRARecommendation(all_ego_events, state_key)

    def ego_Sys_SetEPSHandsOffDetection(self, all_ego_events, state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Sys_SetEPSHandsOffDetection(all_ego_events, state_key)

    def ego_Sys_SetESPState(self, all_ego_events, state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Sys_SetESPState(all_ego_events, state_key)

    def ego_Sys_SetHandsOnCapacitiveSteeringDetection(self, all_ego_events, state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Sys_SetHandsOnCapacitiveSteeringDetection(all_ego_events, state_key)

    def ego_Sys_SetIgnitionState(self, all_ego_events, state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Sys_SetIgnitionState(all_ego_events, state_key)

    def ego_Sys_SetProductionMode(self, all_ego_events, state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Sys_SetProductionMode(all_ego_events, state_key)

    def ego_Sys_SetSnowChainMode(self, all_ego_events, state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Sys_SetSnowChainMode(all_ego_events, state_key)

    def ego_Sys_SetTireFriction(self, all_ego_events, state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Sys_SetTireFriction(all_ego_events, state_key)

    def ego_Sys_SetTireState(self, all_ego_events, state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Sys_SetTireState(all_ego_events, state_key)

    def ego_Sys_SetTrailerPlugState(self, all_ego_events, state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Sys_SetTrailerPlugState(all_ego_events, state_key)

    def ego_Sys_SetTransportMode(self, all_ego_events, state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_Sys_SetTransportMode(all_ego_events, state_key)

    def ego_TBA_RunDiagnosticService(self, all_ego_events, state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_TBA_RunDiagnosticService(all_ego_events, state_key)

    def ego_TBA_WriteEvaluationEvent(self, all_ego_events, state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.ego_TBA_WriteEvaluationEvent(all_ego_events, state_key)

    def E_PrepareVehicle(self, all_ego_events, state_key):
        if EgoScnearioActs.flag == 0:
            self.EGO_algo_acts.E_PrepareVehicle(all_ego_events, state_key)