import os
import sys
from e2xostream.src.acts_algo import shared_data
from e2xostream.stk.scenariogeneration import xosc, prettyprint, ScenarioGenerator
from e2xostream.stk.vehicledynamics.VehicleControl import VehicleControlMovements as vehiclecontrol
from e2xostream.stk.vehicledynamics.DataControl import DataControls as datacontrol
from e2xostream.stk.vehicledynamics.VehicleScenarioSetup import VehicleScenario
from e2xostream import xlmrmaps,xodrmaps
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


class Ego_Acts:
    def __init__(self, egoname, states_analysis, paramlist_analysis, state_events, param_events, esmini_path):
        self.states_analysis = states_analysis
        self.paramlist_analysis = paramlist_analysis
        self.last_processed_key = None  # Track the last key processed
        self.last_processed_action_index = -1
        # self.last_processed_key1 = None
        # self.last_processed_action_index1 = -1
        self.last_processed_key2 = None
        self.last_processed_action_index2 = -1
        self.last_processed_key3 = None
        self.last_processed_action_index3 = -1
        self.last_processed_key4 = None
        self.last_processed_action_index4 = -1
        self.last_processed_key5 = None
        self.last_processed_action_index5 = -1
        self.last_processed_key6 = None
        self.last_processed_action_index6 = -1
        self.last_processed_key7 = None
        self.last_processed_action_index7 = -1
        self.states_analysis = states_analysis
        self.paramlist_analysis = paramlist_analysis
        self.state_events = state_events
        self.param_events = param_events
        self.esmini_path = esmini_path
        self.egoname = egoname
        self.open_scenario_version = 0
        self.value_throttle=0
        self.VehicleControls = vehiclecontrol()
        self.Data_Controls = datacontrol()
        self.VehicleDefines = VehicleScenario()

        self.TBA_value = EBTB_API_data.get_TBA_key_value(states_analysis=self.states_analysis)
        self.throttle = EBTB_API_data.get_vehicle_throttle_info(TBA_eval_key=self.TBA_value,
                                                                states_analysis=self.states_analysis)
        self.brake = EBTB_API_data.get_vehicle_braking_info(TBA_eval_key=self.TBA_value,
                                                            states_analysis=self.states_analysis)

        shared_data.event_counter = 1
        self.last_index_ego = {}
        self.ego_brake_index = {}
        self.ego_gear_index = {}
        self.ego_throttle_index = {}
        self.ego_pb_index = {}
        self.ego_sw_index = {}





    def prepare_vehicle(self,all_ego_events,state_key):
        pass

    def e_time(self,all_ego_events,state_key):
        keys = list(self.states_analysis.keys())
        start_key_index = keys.index(self.last_processed_key5) + 1 if self.last_processed_key5 else 0

        for key in keys[start_key_index:]:
            ego_actions = self.states_analysis[key].get('EgoActions', [])

            # Loop through `ego_actions`, starting from the next unprocessed action index
            start_action_index = self.last_processed_action_index5 + 1 if self.last_processed_key5 == key else 0
            for i in range(start_action_index, len(ego_actions)):
                action = ego_actions[i]
                if action.get('Action') == "E_Time":
                    # Extract operator and velocity values from the parameters
                    parameters = action.get('Parameters', [])
                    delay = parameters[0].get("StateTime",0)
                    delay1 = parameters[0].get("SimTime",0)


                    event_name = f"event{shared_data.event_counter}"
                    action_name = f"SimOneDriver:event{shared_data.event_counter}"

                    if shared_data.event_counter == 1:
                        start_trig = self.VehicleDefines.create_ego_event(value=10)
                    else:

                        if delay:
                            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
                                element_name=f"SimOneDriver:event{shared_data.event_counter - 1}", delay=delay)
                        elif delay1:
                            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
                                element_name=f"SimOneDriver:event{shared_data.event_counter - 1}", delay=delay1)
                        else:
                            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
                                element_name=f"SimOneDriver:event{shared_data.event_counter - 1}", delay=0)

                    start_action = self.VehicleDefines.create_custom_command_action("Signal add:E_Time")
                    all_ego_events.append(
                        self.VehicleDefines.define_ego_action_event(start_trig=start_trig,
                                                                    start_action=start_action,
                                                                    event_name=event_name, action_name=action_name))
                    shared_data.event_counter += 1
                    self.last_processed_key5 = key  # Update last processed key
                    self.last_processed_action_index5 = i  # Update last processed action index
                    return  # Exit after processing one action

                    # If all actions are processed, reset tracking variables
            self.last_processed_key5 = None
            self.last_processed_action_index5 = -1





    def ego_accelration_act(self, all_ego_events,state_key):

        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"

        shared_data.count = shared_data.event_counter

        if shared_data.event_counter == 1:
            self.start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            self.start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)

        start_action = self.VehicleDefines.ego_acceleration_actions(
                                                                    state_data=self.states_analysis,
                                                                    param_data=self.paramlist_analysis,dict=self.last_index_ego)
        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=self.start_trig,start_action=start_action,event_name=event_name,action_name=action_name))

        shared_data.event_counter += 1


    def ego_E_SysVehicleVelocity(self, all_ego_events,state_key):
        # Loop through each key in `states_analysis`, starting from the last processed key
        keys = list(self.states_analysis.keys())
        start_key_index = keys.index(self.last_processed_key) + 1 if self.last_processed_key else 0

        for key in keys[start_key_index:]:
            ego_actions = self.states_analysis[key].get('EgoActions', [])

            # Loop through `ego_actions`, starting from the next unprocessed action index
            start_action_index = self.last_processed_action_index + 1 if self.last_processed_key == key else 0
            for i in range(start_action_index, len(ego_actions)):
                action = ego_actions[i]
                if action.get('Action') == "E_SysVehicleVelocity":
                    # Extract operator and velocity values from the parameters
                    parameters = action.get('Parameters', [])
                    operator = parameters[0].get("Operator","equalTo")
                    velocity_value = parameters[0].get("Velocity",0)
                    velocity = float(velocity_value)*(5/18)


                    # Define event and action names
                    event_name = f"event{shared_data.event_counter}"
                    action_name = f"SimOneDriver:event{shared_data.event_counter}"

                    # Create trigger and action
                    if shared_data.event_counter == 1:
                        self.start_trig_sysveh = self.VehicleDefines.create_ego_event(value=10)
                    else:
                        self.start_trig_sysveh = self.VehicleDefines.create_speed_condition_trigger(
                        operator, self.egoname, speed=velocity)

                    start_action = self.VehicleDefines.create_custom_command_action("Signal add:E_SysVehicleVelocity")

                    # Append event to all_ego_events and increment event counter
                    all_ego_events.append(
                        self.VehicleDefines.define_ego_action_event(
                            start_trig=self.start_trig_sysveh,
                            start_action=start_action,
                            event_name=event_name,
                            action_name=action_name
                        )
                    )


                    shared_data.event_counter += 1
                    self.last_processed_key = key  # Update last processed key
                    self.last_processed_action_index = i  # Update last processed action index
                    return  # Exit after processing one action

        # If all actions are processed, reset tracking variables
        self.last_processed_key = None
        self.last_processed_action_index = -1

    def ego_throttle_act(self, all_ego_events,state_key):

        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"

        value_throttle = EBTB_API_data.ego_throttle(self.ego_throttle_index,self.states_analysis)
        if value_throttle == "Hold":
            value_throttle = 5

        if shared_data.event_counter == 1:
            self.start_trig_throttle = self.VehicleDefines.create_ego_event(value=10)

        else:
            self.start_trig_throttle = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
                element_name=f"SimOneDriver:event{shared_data.event_counter - 1}", delay=0)

        start_action = self.VehicleDefines.create_controller_override_action(throttle=value_throttle,
                                                                         brake=0, clutch=0, parkingbrake=0,
                                                                         steeringwheel=0, gear=0)

        all_ego_events.append(self.VehicleDefines.define_ego_action_event(start_trig=self.start_trig_throttle,
                                                                      start_action=start_action, event_name=event_name,
                                                                      action_name=action_name))
        shared_data.event_counter += 1

    def ego_brake_act(self, all_ego_events,state_key):

        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"

        value_break = EBTB_API_data.ego_brake(self.ego_brake_index, self.states_analysis)

        if shared_data.event_counter == 1:
            self.start_trig_brake = self.VehicleDefines.create_ego_event(value=10)

        else:
            self.start_trig_brake = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
                element_name=f"SimOneDriver:event{shared_data.event_counter - 1}", delay=0)


        start_action = self.VehicleDefines.create_controller_override_action(throttle=0,
                                                                         brake=value_break,
                                                                         clutch=0, parkingbrake=0,
                                                                         steeringwheel=0, gear=0)

        all_ego_events.append(self.VehicleDefines.define_ego_action_event(start_trig=self.start_trig_brake,
                                                                      start_action=start_action,event_name=event_name,action_name=action_name))
        shared_data.event_counter +=1

    def E_landmark(self,all_ego_events,state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"

        envp_lane_selection = EBTB_API_data.get_lane_selection_ego(paramlist_analysis=self.paramlist_analysis)

        landmark_type, landmark_offset = EBTB_API_data.xlmr_mapping_landmark(states_analysis=self.states_analysis,
                                                                             paramlist_analysis=self.paramlist_analysis)

        if landmark_type is None and landmark_offset is None:
            return "Stop"


        lane_mapping = {"Right1":-1,"Right2":-2,"Right3" : -3,"Right4" : -4,"Right5" : -5,"Right6" : -6,
         "Left1" : 1,"Left2" : 2,"Left3":3,"Left4":4,"Left5":5,"Left6":6}
        envp_lane_selection1 = lane_mapping.get(envp_lane_selection)

        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_reach_position_condition_trigger(landmark_type, envp_lane_selection1,
                                                                                     landmark_offset)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:E_Landmark")
        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1



    def ego_Dri_SetLateralDisplacement(self, all_ego_events,state_key):

        keys = list(self.states_analysis.keys())
        start_key_index = keys.index(self.last_processed_key7) + 1 if self.last_processed_key7 else 0

        for key in keys[start_key_index:]:
            ego_actions = self.states_analysis[key].get('EgoActions', [])
            start_action_index = self.last_processed_action_index7 + 1 if self.last_processed_key7 == key else 0
            for i in range(start_action_index, len(ego_actions)):
                action = ego_actions[i]
                if action.get('Action') == "Dri_SetLateralDisplacement":
                    parameters = action.get('Parameters', [])
                    dispvalue = parameters[0]["TargetDisplacement"]

                    event_name = f"event{shared_data.event_counter}"
                    action_name = f"SimOneDriver:event{shared_data.event_counter}"

                    if shared_data.event_counter == 1:
                        start_trig = self.VehicleDefines.create_ego_event(value=10)
                    else:
                        start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
                        element_name=f"SimOneDriver:event{shared_data.event_counter - 1}", delay=0)
                    start_action = self.VehicleDefines.create_lateral_distance_action(value=dispvalue)
                    all_ego_events.append(
                        self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                                    event_name=event_name, action_name=action_name))

                    shared_data.event_counter += 1
                    self.last_processed_key7 = key  # Update last processed key
                    self.last_processed_action_index7 = i  # Update last processed action index
                    return  # Exit after processing one action

        self.last_processed_key7 = None
        self.last_processed_action_index7 = -1



    def ego_Dri_SetLateralReference(self, all_ego_events,state_key):

        keys = list(self.states_analysis.keys())
        start_key_index = keys.index(self.last_processed_key6) + 1 if self.last_processed_key6 else 0

        for key in keys[start_key_index:]:
            ego_actions = self.states_analysis[key].get('EgoActions', [])

            # Loop through `ego_actions`, starting from the next unprocessed action index
            start_action_index = self.last_processed_action_index6 + 1 if self.last_processed_key6 == key else 0
            for i in range(start_action_index, len(ego_actions)):
                action = ego_actions[i]
                if action.get('Action') == "Dri_SetLateralReference":
                    parameters = action.get('Parameters', [])
                    lane_value_str = parameters[0]["LaneSelection"]

                    event_name = f"event{shared_data.event_counter}"
                    action_name = f"SimOneDriver:event{shared_data.event_counter}"

                    try:
                        lane = self.VehicleControls.split_string(lane_value_str)
                        if lane[0] == "Right":
                            lane_value = self.VehicleControls.right_position(lane[1])
                        elif lane[0] == "Left":
                            lane_value = self.VehicleControls.left_position(lane[1])

                        if shared_data.event_counter == 1:
                            start_trig = self.VehicleDefines.create_ego_event(value=10)
                        else:
                            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
                            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}", delay=0)

                        start_action = self.VehicleDefines.create_lateral_reference_action(lane=lane_value)
                        all_ego_events.append(self.VehicleDefines.define_ego_action_event(start_trig=start_trig,
                                                                                          start_action=start_action,
                                                                                          event_name=event_name,
                                                                                          action_name=action_name))

                        shared_data.event_counter += 1
                        self.last_processed_key = key  # Update last processed key
                        self.last_processed_action_index = i  # Update last processed action index
                        return  # Exit after processing one action


                    except:
                        ego_lane = EBTB_API_data.get_lane_selection_ego(paramlist_analysis=self.paramlist_analysis)

                        ego = self.VehicleControls.split_string(ego_lane)
                        if ego[0] == "Right":
                            lane_split_ego = self.VehicleControls.right_position(ego[1])
                        elif ego[0] == "Left":
                            lane_split_ego = self.VehicleControls.left_position(ego[1])

                        lane_value = self.VehicleDefines.lane_set_lateral_ref(ego_lane, lane_value_str, lane_split_ego)

                        if shared_data.event_counter == 1:
                            start_trig = self.VehicleDefines.create_ego_event(value=10)
                        else:
                            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
                                            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}", delay=0)
                        start_action = self.VehicleDefines.create_lateral_reference_action(lane=lane_value)
                        all_ego_events.append(self.VehicleDefines.define_ego_action_event(start_trig=start_trig,
                                                                            start_action=start_action,
                                                                            event_name=event_name, action_name=action_name))
                        shared_data.event_counter += 1
                        self.last_processed_key6 = key
                        self.last_processed_action_index6 = i
                        return

        self.last_processed_key = None
        self.last_processed_action_index = -1



    # def ego_E_ObjectDistanceLaneBased(self, all_ego_events,state_key):
    #
    #     event_name = f"event{shared_data.event_counter}"
    #     action_name = f"SimOneDriver:event{shared_data.event_counter}"
    #
    #     keys = list(self.states_analysis.keys())
    #     start_key_index = keys.index(self.last_processed_key1) + 1 if self.last_processed_key1 else 0
    #
    #     for key in keys[start_key_index:]:
    #         ego_actions = self.states_analysis[key].get('EgoActions', [])
    #
    #         # Loop through `ego_actions`, starting from the next unprocessed action index
    #         start_action_index = self.last_processed_action_index1 + 1 if self.last_processed_key1 == key else 0
    #         for i in range(start_action_index, len(ego_actions)):
    #             action = ego_actions[i]
    #             if action.get('Action') == "E_ObjectDistanceLaneBased":
    #                 parameters = action.get('Parameters',[])
    #                 distance_value = parameters[0].get("Distance")
    #                 relational_operator = parameters[0].get("RelationalOperator")
    #                 reference_object = parameters[0].get("ReferenceObject")
    #                 object_id = parameters[0].get("ObjectID")
    #
    #                 if shared_data.event_counter == 1:
    #                     start_trig = self.VehicleDefines.create_ego_event(value=10)
    #                 else:
    #                     start_trig = self.VehicleDefines.create_relative_distance_condition_trigger(
    #                     distance_value, relational_operator, reference_object, object_id)
    #
    #                 start_action = self.VehicleDefines.create_custom_command_action("Signal add:E_ObjectDistanceLaneBased")
    #
    #                 all_ego_events.append(
    #                     self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,event_name=event_name,action_name=action_name))
    #                 shared_data.event_counter += 1
    #                 self.last_processed_key1 = key
    #                 self.last_processed_action_index1 = i
    #                 return
    #     self.last_processed_key1 = None
    #     self.last_processed_action_index1 = -1


    def ego_E_ObjectCollision(self, all_ego_events,state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"

        keys = list(self.states_analysis.keys())
        start_key_index = keys.index(self.last_processed_key3) + 1 if self.last_processed_key3 else 0

        for key in keys[start_key_index:]:
            ego_actions = self.states_analysis[key].get('EgoActions', [])

            # Loop through `ego_actions`, starting from the next unprocessed action index
            start_action_index = self.last_processed_action_index3 + 1 if self.last_processed_key3 == key else 0
            for i in range(start_action_index, len(ego_actions)):
                action = ego_actions[i]
                if action.get('Action') == "E_ObjectCollision":
                    parameters = action.get('Parameters',[])
                    obj_name = parameters[0].get("ObjectID")

                    if shared_data.event_counter == 1:
                        start_trig = self.VehicleDefines.create_ego_event(value=10)
                    else:
                        start_trig = self.VehicleDefines.create_collision_condition_trigger(entity_name="SimOneDriver",target_entity=obj_name)
                    start_action = self.VehicleDefines.create_custom_command_action("Signal add:E_obj_collision")

                    all_ego_events.append(
                        self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                                    event_name=event_name, action_name=action_name))
                    shared_data.event_counter += 1
                    self.last_processed_key3 = key
                    self.last_processed_action_index3 = i
                    return
        self.last_processed_key3 = None
        self.last_processed_action_index3 = -1


    def ego_Dri_SwitchGear(self, all_ego_events,state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"

        gear_value = EBTB_API_data.ego_switchgear(self.ego_gear_index,self.states_analysis)

        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        if gear_value == "D":
            start_action = self.VehicleDefines.create_controller_override_action(throttle=0,
                                                                                 brake=0, clutch=0, parkingbrake=0,
                                                                                 steeringwheel=0, gear=1)
        elif gear_value == "R":
            start_action = self.VehicleDefines.create_controller_override_action(throttle=0,
                                                                                 brake=0, clutch=0, parkingbrake=0,
                                                                                 steeringwheel=0, gear=2)
        elif gear_value == "N":
            start_action = self.VehicleDefines.create_controller_override_action(throttle=0,
                                                                                 brake=0, clutch=0, parkingbrake=0,
                                                                                 steeringwheel=0, gear=0)
        elif gear_value == "P":
            start_action = self.VehicleDefines.create_controller_override_action(throttle=0,
                                                                                 brake=0, clutch=0, parkingbrake=0,
                                                                                 steeringwheel=0, gear=3)
        all_ego_events.append(self.VehicleDefines.define_ego_action_event(start_trig=start_trig,
                                                                          start_action=start_action,event_name=event_name,action_name=action_name))
        shared_data.event_counter += 1

    def ego_ethernet_setsignal(self,all_ego_events,state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Ethernet_Set_Signal")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_ethernet_invalidateE2E(self,all_ego_events,state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Ethernet_InvalidateE2EProtection")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_ethernet_setsignalinvalid(self,all_ego_events,state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Ethernet_SetSignalInvalid")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_ethernet_suspendPDUTriggering(self,all_ego_events,state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Ethernet_SuspendPDUTriggering")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_dri_AcknowledgeCPMToastMessage(self,all_ego_events,state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Dri_AcknowledgeCPMToastMessage")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Dri_ChangeACCSpeed(self,all_ego_events,state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Dri_ChangeACCSpeed")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Dri_ChangeVSLSpeed(self,all_ego_events,state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Dri_ChangeVSLSpeed")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Dri_ConfigureCollisionAvoidanceFunction(self,all_ego_events,state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Dri_ConfigureCollisionAvoidanceFunction")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Dri_ConfigureDrivingFunction(self,all_ego_events,state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Dri_ConfigureDrivingFunction")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Dri_ConfigureParkingFunction(self,all_ego_events,state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Dri_ConfigureParkingFunction")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Dri_PlaceHandCloseToDoorHandle(self,all_ego_events,state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Dri_PlaceHandCloseToDoorHandle")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Dri_PressAPASoftKey(self,all_ego_events,state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Dri_PressAPASoftKey")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Dri_PressRMASoftKey(self,all_ego_events,state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Dri_PressRMASoftKey")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Dri_PressSteeringWheelButton(self,all_ego_events,state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Dri_PressSteeringWheelButton")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Dri_SetACCDistance(self,all_ego_events,state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Dri_SetACCDistance")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Dri_SetBeltState(self,all_ego_events,state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Dri_SetBeltState")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Dri_SetCarwashMode(self,all_ego_events,state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Dri_SetCarwashMode")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Dri_SetDrivingProgram(self,all_ego_events,state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Dri_SetDrivingProgram")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Dri_SetElectricalTrailerHitchPosition(self,all_ego_events,state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Dri_SetElectricalTrailerHitchPosition")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Dri_SetESPMode(self,all_ego_events,state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Dri_SetESPMode")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Dri_SetHazardWarningLights(self,all_ego_events,state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Dri_SetHazardWarningLights")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Dri_SetLockState(self,all_ego_events,state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Dri_SetLockState")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Dri_SetMirrorPosition(self, all_ego_events, state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Dri_SetMirrorPosition")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Dri_SetParkDisplayView(self, all_ego_events, state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Dri_SetParkDisplayView")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Dri_SetParkSwitchState(self, all_ego_events, state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Dri_SetParkSwitchState")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Dri_SetVehicleSpeedUnit(self, all_ego_events, state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Dri_SetVehicleSpeedUnit")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Dri_SetWinterTireSpeedLimiter(self, all_ego_events, state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Dri_SetWinterTireSpeedLimiter")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Dri_SwitchACCVSLMode(self, all_ego_events, state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Dri_SwitchACCVSLMode")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Dri_SwitchToACCDriving(self, all_ego_events, state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Dri_SwitchToACCDriving")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Dri_SwitchToVSLDriving(self, all_ego_events, state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Dri_SwitchToVSLDriving")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Dri_TakeHandsOff(self, all_ego_events, state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Dri_TakeHandsOff")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Sys_ReleaseAcceleratorPedalRequest(self, all_ego_events, state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Sys_ReleaseAcceleratorPedalRequest")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Sys_SetADASISv2Attribute(self, all_ego_events, state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Sys_SetADASISv2Attribute")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Sys_SetAttentionAssist(self, all_ego_events, state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Sys_SetAttentionAssist")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Sys_SetECOAssistRequest(self, all_ego_events, state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Sys_SetECOAssistRequest")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Sys_SetELVIRARecommendation(self, all_ego_events, state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Sys_SetELVIRARecommendation")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Sys_SetEPSHandsOffDetection(self, all_ego_events, state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Sys_SetEPSHandsOffDetection")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Sys_SetESPState(self, all_ego_events, state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Sys_SetESPState")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Sys_SetHandsOnCapacitiveSteeringDetection(self, all_ego_events, state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Sys_SetHandsOnCapacitiveSteeringDetection")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1
    def ego_Sys_SetIgnitionState(self, all_ego_events, state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Sys_SetIgnitionState")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Sys_SetProductionMode(self, all_ego_events, state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Sys_SetProductionMode")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Sys_SetSnowChainMode(self, all_ego_events, state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Sys_SetSnowChainMode")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Sys_SetTireFriction(self, all_ego_events, state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Sys_SetTireFriction")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Sys_SetTireState(self, all_ego_events, state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Sys_SetTireState")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Sys_SetTrailerPlugState(self, all_ego_events, state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Sys_SetTrailerPlugState")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Sys_SetTransportMode(self, all_ego_events, state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Sys_SetTransportMode")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_TBA_RunDiagnosticService(self, all_ego_events, state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"
        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)
        start_action = self.VehicleDefines.create_custom_command_action("Signal add:TBA_RunDiagnosticService")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_TBA_WriteEvaluationEvent(self, all_ego_events, state_key):

        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"

        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=15)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
            element_name=f"SimOneDriver:event{shared_data.event_counter - 1}")

        start_action = self.VehicleDefines.create_custom_command_action("Signal add:TBA_WriteEvaluationEvent")

        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                        event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def E_PrepareVehicle(self, all_ego_events, state_key):
        pass

    def ego_E_DistanceTimeBased(self, all_ego_events,state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"

        keys = list(self.states_analysis.keys())
        start_key_index = keys.index(self.last_processed_key2) + 1 if self.last_processed_key2 else 0

        for key in keys[start_key_index:]:
            ego_actions = self.states_analysis[key].get('EgoActions', [])

            # Loop through `ego_actions`, starting from the next unprocessed action index
            start_action_index = self.last_processed_action_index2 + 1 if self.last_processed_key2 == key else 0
            for i in range(start_action_index, len(ego_actions)):
                action = ego_actions[i]
                if action.get('Action') == "E_DistanceTimeBased":

                    parameters = action.get('Parameters',[])
                    dist_value = parameters[0].get("Offset")
                    relational_operator = parameters[0].get("RelationalOperator")
                    reference_object = parameters[0].get("ReferenceObject")
                    object = parameters[0].get("Object")

                    if object == "SysVehicle":
                        if shared_data.event_counter == 1:
                            start_trig = self.VehicleDefines.create_ego_event(value=10)
                        else:
                            start_trig = self.VehicleDefines.create_time_headway_condition_trigger(
                            dist_value, relational_operator, object, reference_object)

                        start_action = self.VehicleDefines.create_custom_command_action("Signal add:E_DistanceTimeBased")

                        all_ego_events.append(
                            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                                        event_name=event_name, action_name=action_name))
                        shared_data.event_counter += 1
                        self.last_processed_key2 = key
                        self.last_processed_action_index2 = i
                        return

            self.last_processed_key2 = None
            self.last_processed_action_index2 = -1


    def ego_Dri_SetParkingBrake(self, all_ego_events,state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"


        brake_value = EBTB_API_data.ego_parkingbrake(self.ego_pb_index,self.states_analysis)

        if shared_data.event_counter == 1:
            start_trig_parkingbrake = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig_parkingbrake = self.VehicleDefines.create_storyboard_element_state_condition_trigger(element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)

        if brake_value == "Engage":
            start_action = self.VehicleDefines.create_controller_override_action(throttle=0,
                                                                                 brake=0, clutch=0, parkingbrake=1,
                                                                                 steeringwheel=0, gear=0)
        elif brake_value == "Release":
            start_action = self.VehicleDefines.create_controller_override_action(throttle=0,
                                                                                 brake=0, clutch=0, parkingbrake=0,
                                                                                 steeringwheel=0, gear=0)



        all_ego_events.append(self.VehicleDefines.define_ego_action_event(start_trig=start_trig_parkingbrake,
                                                                          start_action=start_action,event_name=event_name,action_name=action_name))
        shared_data.event_counter += 1

    def ego_Dri_SetSteeringWheelAngle(self, all_ego_events,state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"

        wheel_angle_value = EBTB_API_data.ego_steeringwheel_angle(self.ego_sw_index,self.states_analysis)

        if shared_data.event_counter == 1:
            start_trig_steeringwheel = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig_steeringwheel = self.VehicleDefines.create_storyboard_element_state_condition_trigger(element_name=f"SimOneDriver:event{shared_data.event_counter - 1}",delay=0)

        start_action = self.VehicleDefines.create_controller_override_action(throttle=0,
                                                                             brake=0, clutch=0, parkingbrake=0,
                                                                             steeringwheel=wheel_angle_value, gear=0)
        all_ego_events.append(self.VehicleDefines.define_ego_action_event(start_trig=start_trig_steeringwheel ,
                                                                          start_action=start_action,event_name=event_name,action_name=action_name))
        shared_data.event_counter += 1

    def ego_Env_SetTrafficLightState(self, all_ego_events,state_key):
        pass

    def ego_E_Timetocollision(self,all_ego_events,state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"

        keys = list(self.states_analysis.keys())
        start_key_index = keys.index(self.last_processed_key4) + 1 if self.last_processed_key4 else 0

        for key in keys[start_key_index:]:
            ego_actions = self.states_analysis[key].get('EgoActions', [])

            # Loop through `ego_actions`, starting from the next unprocessed action index
            start_action_index = self.last_processed_action_index4 + 1 if self.last_processed_key4 == key else 0
            for i in range(start_action_index, len(ego_actions)):
                action = ego_actions[i]
                if action.get('Action') == "E_TimeToCollision":
                    parameters = action.get('Parameters', [])
                    obj_id = parameters[0].get("ObjectID")
                    operator = parameters[0].get("Operator","equalTo")
                    reference_time = parameters[0].get("ReferenceTime",0.0)

                    if shared_data.event_counter == 1:
                        start_trig = self.VehicleDefines.create_ego_event(value=10)
                    else:
                        start_trig = self.VehicleDefines.create_time_to_collision_condition_trigger(reference_time,operator,obj_id)

                    start_action = self.VehicleDefines.create_custom_command_action("Signal add:E_TimetoCollision")

                    all_ego_events.append(
                        self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,
                                                                    event_name=event_name, action_name=action_name))
                    shared_data.event_counter += 1
                    self.last_processed_key4 = key
                    self.last_processed_action_index4 = i
                    return
        self.last_processed_key4 = None
        self.last_processed_action_index4 = -1

    def Dri_SetIndicatorState(self,all_ego_events,state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"

        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
                element_name=f"SimOneDriver:event{shared_data.event_counter - 1}", delay=0)

        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Dri_SetIndicatorState")
        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1

    def ego_Dri_SetVehicleDoor(self, all_ego_events,state_key):
        event_name = f"event{shared_data.event_counter}"
        action_name = f"SimOneDriver:event{shared_data.event_counter}"

        if shared_data.event_counter == 1:
            start_trig = self.VehicleDefines.create_ego_event(value=10)
        else:
            start_trig = self.VehicleDefines.create_storyboard_element_state_condition_trigger(
                element_name=f"SimOneDriver:event{shared_data.event_counter - 1}", delay=0)

        start_action = self.VehicleDefines.create_custom_command_action("Signal add:Dri_SetVehicleDoor")
        all_ego_events.append(
            self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action,event_name=event_name, action_name=action_name))
        shared_data.event_counter += 1