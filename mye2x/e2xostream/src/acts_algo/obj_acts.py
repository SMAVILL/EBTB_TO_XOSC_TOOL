import os
import sys
from e2xostream.src.acts_algo import shared_data

from e2xostream.src.scenario_generator.basescenario import BaseScenario
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

MAIN_PATH = os.path.abspath(os.path.join(__file__, "..", "..", "..", ".."))
CURRENT_WORKING_FILE_DIRECTORY = os.path.abspath(os.path.join(__file__))
CURRENT_WORKING_DIRECTORY = os.path.abspath(os.path.join(__file__, ".."))

if MAIN_PATH not in sys.path:
    sys.path.append(MAIN_PATH)

dir_path = os.path.dirname(os.path.realpath(__file__))
for root, dirs, files in os.walk(dir_path):
    sys.path.append(root)


class Obj_Acts:
    def __init__(self, egoname, states_analysis, paramlist_analysis, state_events, param_events, esmini_path):
        self.states_analysis = states_analysis
        self.paramlist_analysis = paramlist_analysis
        self.state_events = state_events
        self.param_events = param_events
        self.target_name = None
        self.esmini_path = esmini_path
        self.egoname = egoname
        self.open_scenario_version = 0
        self.VehicleControls = vehiclecontrol()
        self.Data_Controls = datacontrol()
        self.VehicleDefines = VehicleScenario()
        self.basescenario = BaseScenario()

        self.processed_flags = {}  # Define it as an instance attribute
        self.lane = 0
        shared_data.event_counter_obj = 1
        self.last_index = {}




    def Obj_Accelration_act(self, all_target_events,state_key,target_name):
        pass


    def obj_changelane(self, all_target_events,state_key,target_name):
        try:
            event_name = f"event{shared_data.event_counter_obj}"
            action_name = f"{target_name}1:event{shared_data.event_counter_obj}"

            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
                element_name=f"{target_name}:event{shared_data.event_counter_obj - 1}", delay=0)


            for state_id, state_info in self.states_analysis.items():
                if 'ObjectActions' in state_info and state_info['ObjectActions']:
                    if target_name in state_info['ObjectActions']:
                        obj1_actions = state_info['ObjectActions'][target_name]
                        for action in obj1_actions:
                            if action['Action'] == 'Obj_ChangeLane':
                                direction = action['Parameters'][0]['Direction']
                                value_of_dist = action['Parameters'][0]['TransitionDistance']


            for k, v in self.states_analysis.items():
                for obj_id, actions in v.get('ObjectActions', {}).items():
                    for action in actions:
                        if action.get('Action') == ObjAPI.Obj_Initialize:
                            for param in action.get('Parameters', []):
                                if param.get('ObjectId') == target_name:
                                    present_lane = param.get('LaneSelection')

            start_action = self.VehicleDefines.create_obj_lanechange_action(target_name, direction, present_lane,value_of_dist,
                                                                            state_data=self.states_analysis,
                                                                            param_data=self.paramlist_analysis)
            target_next_event = self.VehicleDefines.define_target_action_event(start_trig=start_trig,
                                                                               start_action=start_action,event_name=event_name,action_name=action_name)

            all_target_events.append(target_next_event)
            shared_data.event_counter_obj += 1
        except Exception as e:
            print(f"Error: {e}")

    def obj_setlateraldisplacement(self, all_target_events,state_key,target_name):
        event_name = f"event{shared_data.event_counter_obj}"
        action_name = f"{target_name}:event{shared_data.event_counter_obj}"


        for k, v in self.states_analysis.items():
            for kv, vv in v["ObjectActions"].items():
                if vv[0]['Action'] == "Obj_SetLateralReference":
                    pass
                elif vv[0]['Action'] == "Obj_SetLateralDisplacement":
                    dispvalue = vv[0]['Parameters'][0]['TargetDisplacement']

                    start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
                        element_name=f"{target_name}:event{shared_data.event_counter_obj - 1}",delay=0)

                    start_action = self.VehicleDefines.create_obj_lateral_distance_action(value=dispvalue,
                                                                                          entity=target_name,
                                                                                          state_data=self.states_analysis)
                    target_next_event = self.VehicleDefines.define_target_action_event(start_trig=start_trig,
                                                                                       start_action=start_action,event_name=event_name,action_name=action_name)
                    all_target_events.append(target_next_event)
                    shared_data.event_counter_obj += 1


    def obj_set_longitudinal_speed(self, all_target_events,state_key, target_name):

        event_name = f"event{shared_data.event_counter_obj}"
        action_name = f"{target_name}:event{shared_data.event_counter_obj}"

        if shared_data.event_counter_obj == 1:
            #start_trig = self.VehicleDefines.create_target_event(value=15)
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
                element_name=f"SimOneDriver:event{shared_data.count-1}", delay=0)

        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
                element_name=f"{target_name}:event{shared_data.event_counter_obj - 1}", delay=0)

        start_action = self.VehicleDefines.obj_acceleration_actions(target_name,
                                                                    state_data=self.states_analysis,
                                                                    param_data=self.paramlist_analysis,dict=self.last_index)

        target_next_event = self.VehicleDefines.define_target_action_event(start_trig=start_trig,
                                                                           start_action=start_action,
                                                                           event_name=event_name,
                                                                           action_name=action_name)

        all_target_events.append(target_next_event)
        shared_data.event_counter_obj += 1