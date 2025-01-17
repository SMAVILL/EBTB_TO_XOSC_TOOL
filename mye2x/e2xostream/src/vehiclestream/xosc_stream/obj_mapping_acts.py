import copy
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
from e2xostream.src.vehiclestream.xosc_stream.ego_mapping_acts import EgoScnearioActs
from e2xostream.stk.fun_register_dispatch import FunctionRegisterDispatcher

MAIN_PATH = os.path.abspath(os.path.join(__file__, "..", "..", "..", ".."))
CURRENT_WORKING_FILE_DIRECTORY = os.path.abspath(os.path.join(__file__))
CURRENT_WORKING_DIRECTORY = os.path.abspath(os.path.join(__file__, ".."))

if MAIN_PATH not in sys.path:
    sys.path.append(MAIN_PATH)

dir_path = os.path.dirname(os.path.realpath(__file__))
for root, dirs, files in os.walk(dir_path):
    sys.path.append(root)


class ObjScnearioActs:
    def __init__(self, egoname, states_analysis, paramlist_analysis, state_events, param_events, esmini_path):
        self.states_analysis = states_analysis
        self.paramlist_analysis = paramlist_analysis
        self.state_events = state_events
        self.param_events = param_events
        self.esmini_path = esmini_path
        self.egoname = egoname
        self.open_scenario_version = 0

        self.all_target_events = []
        self.target_name = None
        self.VehicleControls = vehiclecontrol()
        self.Data_Controls = datacontrol()
        self.VehicleDefines = VehicleScenario()

        self.Obj_algo_acts = obj_acts.Obj_Acts(egoname, states_analysis, paramlist_analysis,
                                               state_events, param_events, esmini_path)

        self.register_dispatch = FunctionRegisterDispatcher()



        # Register the functions
        # dispatcher.register('a', func_a)
        # Dispatch the function
        # dispatcher.dispatch('a', 'Hello', 'World')
        # dispatcher.dispatch('b', 'Test', kwarg1='Optional')
        # dispatcher.dispatch('c', 1, 2, 3)


    def __call__(self, all_target_events,target_name):
        self.all_target_events = all_target_events
        self.target_name = target_name
        self.maneuver_api_mapping()
        self.check_api_dispatch_function(self.all_target_events,target_name)


    def maneuver_api_mapping(self):

        # self.register_dispatch.register(ObjAPI.Obj_Initialize,
        #                                 self.initial_acceleration_acts)

        self.register_dispatch.register(ObjAPI.Obj_SetLongitudinalSpeed,
                                        self.obj_setlongitudinalspeed)

        self.register_dispatch.register(ObjAPI.Obj_ChangeLane,
                                        self.obj_changelane)
        self.register_dispatch.register(ObjAPI.Obj_SetLateralDisplacement,
                                        self.obj_setlateraldisplacement)
        self.register_dispatch.register(OtherAPI.E_ObjectDistanceLaneBased,self.obj_distance_lanebased)
        self.register_dispatch.register(ObjAPI.Obj_SetLateralReference,self.obj_setlateralref)
        self.register_dispatch.register(ObjAPI.Obj_SetLongitudinalRelativePosition,self.Obj_SetLongitudinalRelativePosition)
        self.register_dispatch.register(ObjAPI.Obj_Deactivate,self.obj_deactivate)


    def check_api_dispatch_function(self,all_target_events,target_name):
        for statekey, statevalue in self.states_analysis.items():
            obj_actions = statevalue.get("ObjectActions", {})
            for k_objID, value_Objv in obj_actions.items():
                for action_dict in value_Objv:
                    tar_name = (
                            action_dict.get("Parameters")[0].get("ObjectId")
                            or action_dict.get("Parameters")[0].get("ObjectID")
                    )
                    if target_name == tar_name:
                        action = action_dict.get("Action")


                        if action:
                            try:


                            # Dispatch the action if it's found
                                self.register_dispatch.dispatch(action,all_target_events,statekey,target_name)

                            except KeyError:
                            # Handle specific exception if the action is not found in the dispatcher
                                continue
                            except Exception as e:
                                print(f"Error dispatching action {action}: {e}")
                                continue



    def initial_acceleration_acts(self, all_target_events,state_key,target_name):
        self.Obj_algo_acts.Obj_Accelration_act(all_target_events,state_key,target_name)

    def obj_changelane(self,all_target_events,state_key,target_name):
        self.Obj_algo_acts.obj_changelane(all_target_events,state_key,target_name)

    def obj_setlateraldisplacement(self, all_target_events,state_key,target_name):
        self.Obj_algo_acts.obj_setlateraldisplacement(all_target_events,state_key,target_name)

    def obj_setlongitudinalspeed(self,all_target_events,state_key,target_name):
        self.Obj_algo_acts.obj_set_longitudinal_speed(all_target_events,state_key,target_name)

    def obj_distance_lanebased(self,all_target_events,state_key,target_name):
        self.Obj_algo_acts.obj_E_ObjectDistanceLaneBased(all_target_events,state_key,target_name)

    def obj_E_distancetimebased(self,all_target_events,state_key,target_name):
        self.Obj_algo_acts.obj_E_DistanceTimeBased(all_target_events,state_key,target_name)

    def obj_setlateralref(self,all_target_events,state_key,target_name):
        self.Obj_algo_acts.obj_setlateralref(all_target_events,state_key,target_name)

    def Obj_SetLongitudinalRelativePosition(self,all_target_events,state_key,target_name):
        self.Obj_algo_acts.Obj_SetLongitudinalRelativePosition(all_target_events,state_key,target_name)

    def obj_deactivate(self,all_target_events,state_key,target_name):
        self.Obj_algo_acts.obj_deactivate(all_target_events,state_key,target_name)