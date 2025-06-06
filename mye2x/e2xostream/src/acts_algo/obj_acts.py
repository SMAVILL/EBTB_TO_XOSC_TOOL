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
        self.last_processed_obj = None
        self.last_processed_action_index_obj = -1

        self.processed_flags = {}  # Define it as an instance attribute
        self.lane = 0
        shared_data.event_counter_obj = 1
        self.last_index = {}
        self.last_index_disp = {}
        self.last_processed_action = -1
        self.last_processed = None
        self.demo_index = {}

    def obj_changelane(self, all_target_events,state_key,target_name):
        """
        Obj_ChangeLane - obj_changelane API
        Args:
            all_target_events
            state_key
            target_name
        """
        try:
            for api in shared_data.res[state_key]:
                if api['api_name'] == "Obj_ChangeLane":
                    event_count = api['event_count']
                    action_count = api['action_count']
                    event_name = f"event{event_count}"
                    action_name = f"{target_name}:action{action_count}"

            state_key = int(state_key)

            if event_count == 1:
                start_trig = self.VehicleDefines.create_ego_event(value=10)
            else:
                val = shared_data.state_e_mapping.get(str(state_key - 1), (None, None))[1]
                target = shared_data.state_e_mapping.get(str(state_key - 1), (None, None))[2]
                start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
                    element_name=f"{target}:action{val}", delay=0)


            direction,value_of_dist = EBTB_API_data.obj_change_lane_details(self.states_analysis,target_name)



            present_lane = shared_data.obj_lane_init[target_name]


            start_action,upd_lane = self.VehicleDefines.create_obj_lanechange_action(target_name, direction, present_lane,value_of_dist,
                                                                            state_data=self.states_analysis,
                                                                            param_data=self.paramlist_analysis)
            shared_data.latest_lane_ego_ref = upd_lane
            target_next_event = self.VehicleDefines.define_target_action_event(start_trig=start_trig,
                                                                               start_action=start_action,event_name=event_name,action_name=action_name)

            all_target_events.append(target_next_event)
            shared_data.event_counter_obj += 1
        except Exception as e:
            print(f"Error: {e}")


    def obj_setlateralref(self,all_target_events,state_key,target_name):
        """
        Obj_SetLateralReference - obj_setlateralref API
        Args:
            all_target_events
            state_key
            target_name
        """
        for api in shared_data.res[state_key]:
            if api['api_name'] == "Obj_SetLateralReference":
                event_count = api['event_count']
                action_count = api['action_count']
                event_name = f"event{event_count}"
                action_name = f"{target_name}:action{action_count}"

        dispvalue = EBTB_API_data.obj_lateral_disp(self.states_analysis, target_name,last_index_disp=self.last_index_disp)
        abs_or_rel,entity = EBTB_API_data.obj_lateral_ref(self.states_analysis,target_name)

        state_key = int(state_key)

        if event_count == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            val = shared_data.state_e_mapping.get(str(state_key - 1), (None, None))[1]
            target = shared_data.state_e_mapping.get(str(state_key - 1), (None, None))[2]
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
                element_name=f"{target}:action{val}", delay=0)

        start_action = self.VehicleDefines.create_obj_lateral_distance_action(
            value=dispvalue,target_name=target_name,
            entity=entity,abs_or_rel=abs_or_rel,
            state_data=self.states_analysis
        )

        target_next_event = self.VehicleDefines.define_target_action_event(start_trig=start_trig,
                                                                           start_action=start_action,
                                                                           event_name=event_name,
                                                                           action_name=action_name)
        all_target_events.append(target_next_event)
        shared_data.event_counter_obj += 1

    def obj_setlateraldisplacement(self, all_target_events, state_key, target_name):
        """
        Obj_SetLateralDisplacement - obj_setlateraldisplacement API
        Args:
            all_target_events
            state_key
            target_name
        """
        # Check if Obj_SetLateralReference exists in the same state_key
        if any(api['api_name'] == "Obj_SetLateralReference" for api in shared_data.res[state_key]):
            # Skip execution if Obj_SetLateralReference is found
            return

        # Proceed with the current implementation
        for api in shared_data.res[state_key]:
            if api['api_name'] == "Obj_SetLateralDisplacement":
                event_count = api['event_count']
                action_count = api['action_count']
                event_name = f"event{event_count}"
                action_name = f"{target_name}:action{action_count}"

        dispvalue = EBTB_API_data.obj_lateral_disp(self.states_analysis, target_name,last_index_disp=self.last_index_disp)
        state_key = int(state_key)

        if event_count == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            val = shared_data.state_e_mapping.get(str(state_key - 1), (None, None))[1]
            target = shared_data.state_e_mapping.get(str(state_key - 1), (None, None))[2]
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
                element_name=f"{target}:action{val}", delay=0)

        start_action = self.VehicleDefines.create_obj_lateral_distance_action(
            value=dispvalue,target_name=target_name,
            entity=None,abs_or_rel="AbsSysLane",
            state_data=self.states_analysis
        )
        target_next_event = self.VehicleDefines.define_target_action_event(
            start_trig=start_trig,
            start_action=start_action,
            event_name=event_name,
            action_name=action_name
        )
        all_target_events.append(target_next_event)
        shared_data.event_counter_obj += 1

    def obj_set_longitudinal_speed(self, all_target_events,state_key, target_name):
        """
        Obj_SetLongitudinalSpeed - speed action API
        Args:
            all_target_events
            state_key
            target_name
        """
        for api in shared_data.res[state_key]:
            if api['api_name'] == "Obj_SetLongitudinalSpeed":
                event_count = api['event_count']
                action_count = api['action_count']
                event_name = f"event{event_count}"
                action_name = f"{target_name}:action{action_count}"


        state_key = int(state_key)

        if event_count == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            val = shared_data.state_e_mapping.get(str(state_key - 1), (None, None))[1]
            target = shared_data.state_e_mapping.get(str(state_key - 1), (None, None))[2]
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
                element_name=f"{target}:action{val}", delay=0)


        start_action = self.VehicleDefines.obj_acceleration_actions(target_name,
                                                                    state_data=self.states_analysis,
                                                                    param_data=self.paramlist_analysis,dict=self.last_index)

        target_next_event = self.VehicleDefines.define_target_action_event(start_trig=start_trig,
                                                                           start_action=start_action,
                                                                           event_name=event_name,
                                                                           action_name=action_name)

        all_target_events.append(target_next_event)
        shared_data.event_counter_obj += 1

    def Obj_SetLongitudinalRelativePosition(self,all_target_events, state_key, target_name):
        """
        Obj_SetLongitudinalRelativePosition - Obj_SetLongitudinalRelativePosition API
        Args:
            all_target_events
            state_key
            target_name
        """
        for api in shared_data.res[state_key]:
            if api['api_name'] == "Obj_SetLongitudinalRelativePosition":
                event_count = api['event_count']
                action_count = api['action_count']
                event_name = f"event{event_count}"
                action_name = f"{target_name}:action{action_count}"

        state_key = int(state_key)

        entity,distance = EBTB_API_data.obj_set_lateral_relative(states_analysis=self.states_analysis,target_name=target_name)

        if event_count == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            val = shared_data.state_e_mapping.get(str(state_key - 1), (None, None))[1]
            target = shared_data.state_e_mapping.get(str(state_key - 1), (None, None))[2]
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
                element_name=f"{target}:action{val}", delay=0)

        start_action = self.VehicleDefines.obj_relative_position(target_name,
                                                                    state_data=self.states_analysis,
                                                                    entity=entity,distance=distance)
        target_next_event = self.VehicleDefines.define_target_action_event(start_trig=start_trig,
                                                                           start_action=start_action,
                                                                           event_name=event_name,
                                                                           action_name=action_name)

        all_target_events.append(target_next_event)
        shared_data.event_counter_obj += 1

    def obj_setlateral_relative_position(self, all_target_events, state_key, target_name):
        """
        Obj_SetLateralRelativePosition - obj_setlateral_relative_position API
        Args:
            all_target_events
            state_key
            target_name
        """
        for api in shared_data.res[state_key]:
            if api['api_name'] == "Obj_SetLateralRelativePosition":
                event_count = api['event_count']
                action_count = api['action_count']
                event_name = f"event{event_count}"
                action_name = f"{target_name}:action{action_count}"

        state_key = int(state_key)
        entity, displacement,units = EBTB_API_data.obj_set_lateral_relative_position(states_analysis=self.states_analysis,target_name=target_name)

        if event_count == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            val = shared_data.state_e_mapping.get(str(state_key - 1), (None, None))[1]
            target = shared_data.state_e_mapping.get(str(state_key - 1), (None, None))[2]
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
                element_name=f"{target}:action{val}", delay=0)

        start_action = self.VehicleDefines.create_obj_lateral_rel_position(target_name,units,
                                                                    entity,displacement)

        target_next_event = self.VehicleDefines.define_target_action_event(start_trig=start_trig,
                                                                           start_action=start_action,
                                                                           event_name=event_name,
                                                                           action_name=action_name)

        all_target_events.append(target_next_event)
        shared_data.event_counter_obj += 1

    def obj_deactivate(self, all_target_events, state_key, target_name):
        """
        Obj_Deactivate - obj_deactivate API
        Args:
            all_target_events
            state_key
            target_name
        """
        for api in shared_data.res[state_key]:
            if api['api_name'] == "Obj_Deactivate":
                event_count = api['event_count']
                action_count = api['action_count']
                event_name = f"event{event_count}"
                action_name = f"{target_name}:action{action_count}"

        state_key = int(state_key)

        if event_count == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            val = shared_data.state_e_mapping.get(str(state_key - 1), (None, None))[1]
            target = shared_data.state_e_mapping.get(str(state_key - 1), (None, None))[2]
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
                element_name=f"{target}:action{val}", delay=0)

        start_action = self.VehicleDefines.create_custom_command_action('{"ActorStateAction": {"enabled": true, "name": "DisabledVehicle"}}')

        target_next_event = self.VehicleDefines.define_target_action_event(start_trig=start_trig,
                                                                           start_action=start_action,
                                                                           event_name=event_name,
                                                                           action_name=action_name)

        all_target_events.append(target_next_event)
        shared_data.event_counter_obj += 1

    def obj_E_ObjectDistanceLaneBased(self, all_target_events, state_key, target_name):
        """
        E_ObjectDistanceLaneBased - obj_E_ObjectDistanceLaneBased API
        Args:
            all_target_events
            state_key
            target_name
        """
        for api in shared_data.res[state_key]:
            if api['api_name'] == "E_ObjectDistanceLaneBased":
                event_count = api['event_count']
                action_count = api['action_count']
                event_name = f"event{event_count}"
                action_name = f"{target_name}:action{action_count}"

        keys = list(self.states_analysis.keys())
        start_key_index = keys.index(self.last_processed) + 1 if self.last_processed else 0

        for key in keys[start_key_index:]:
            object_entries = self.states_analysis[key].get('ObjectActions', {})

            # Ensure dictionary structure for object actions
            for obj, actions in object_entries.items():
                start_action_index = self.last_processed_action + 1 if self.last_processed == key else 0

                for idx in range(start_action_index, len(actions)):
                    action_info = actions[idx]
                    action = action_info.get('Action', None)

                    if action == "E_ObjectDistanceLaneBased":
                        parameters = action_info.get('Parameters', [])
                        distance_value = parameters[0].get("Distance")
                        relational_operator = parameters[0].get("RelationalOperator")
                        reference_object = parameters[0].get("ReferenceObject")
                        object_id = parameters[0].get("ObjectID")

                        shared_data.state_e_mapping[state_key] = ("E_ObjectDistanceLaneBased", action_count, target_name)

                        # Process the action
                        if event_count == 1:
                            start_trig = self.VehicleDefines.create_target_event(value=10)
                        else:
                            start_trig = self.VehicleDefines.create_relative_distance_condition_trigger(
                                    distance_value, relational_operator, reference_object, object_id
                                )

                        start_action = self.VehicleDefines.create_custom_command_action(
                                "Signal add:E_ObjectDistanceLaneBased")

                        all_target_events.append(
                                self.VehicleDefines.define_target_action_event(
                                    start_trig=start_trig,
                                    start_action=start_action,
                                    event_name=event_name,
                                    action_name=action_name))

                            # Update tracking variables
                        shared_data.event_counter_obj += 1
                        self.last_processed = key
                        self.last_processed_action = idx
                        return

            # Reset tracking variables if all actions are processed
        self.last_processed = None
        self.last_processed_action = -1
        return


