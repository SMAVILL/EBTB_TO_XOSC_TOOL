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
from e2xostream.src.acts_algo import shared_data


class BaseScenario:
    def __init__(self):

        self.VehicleControls = vehiclecontrol()
        self.Data_Controls = datacontrol()
        self.VehicleDefines = VehicleScenario()
        road_len = None
        x_value = None
        envp_lane_selection = None
        shared_data.obj_lane_init = {}

    def trigger_condition(self, targetname):
        """
        Trigger condition creation
        Parameters
        ----------
        targetname

        Returns
        -------

        """
        trigcond = xosc.StandStillCondition(0.5)
        standstill_trigger = xosc.EntityTrigger(
            "standstill trigger", 0.1, xosc.ConditionEdge.none, trigcond, targetname
        )

        return standstill_trigger

    def integrate_new_actions(self, init, egoname):
        """
        Integrate new actions
        Returns
        -------

        """
        # Create a controller override action
        override_action = self.VehicleDefines.create_controller_override_action()

        # Assuming you want to apply this action at the start, add it to the init actions for the ego
        init.add_init_action(egoname, override_action)

        # Create a time-to-collision trigger and integrate it
        # This example assumes the target entity reference and other details are known and static.
        # You might want to adapt these values based on your scenario's requirements.
        condition_trigger = self.VehicleDefines.add_time_to_collision_start_trigger(
            target_entity_ref="4bdc6cf0-d460-11ee-bd54-59f53ea26ff9",
            triggering_entity=egoname,
            value=4,  # Time to collision value
            rule="lessThan"
        )
        return condition_trigger

    def ego_startevent_maneuver_group(self, total_events, egoname):
        """
        Create Ego Manuever group
        Parameters
        ----------
        total_events

        Returns
        -------

        """
        ego_man = xosc.Maneuver("EgoManeuver")
        for te in total_events:
            ego_man.add_event(te)
        # ego_man.add_event(ego_event)
        # ego_man.add_event(egothrottle_event)

        ego_mangr = xosc.ManeuverGroup("Ego")
        ego_mangr.add_actor(egoname)
        ego_mangr.add_maneuver(ego_man)

        return ego_mangr

    def target_startevent_maneuver_group(self, target_events, target_name="Obj1"):
        """
        Create Target Manuver group
        Parameters
        ----------
        target_start_event
        target_event
        target_name

        Returns
        -------

        """
        target_man = xosc.Maneuver("targetManeuver")

        for tgt_evnt in target_events:
            target_man.add_event(tgt_evnt)
        # target_man.add_event(target_event)

        target_mangr = xosc.ManeuverGroup(target_name)
        target_mangr.add_actor(target_name)
        target_mangr.add_maneuver(target_man)

        return target_mangr

    def define_ego_entities(self, properties, vehicle_entities, width=2.0, length=5.0, egoname="Ego"):
        """
        Define the Ego entities
        Parameters
        ----------
        properties
        width
        length

        Returns
        -------

        """
        # properties["model_id"] = "0"
        properties = {"Ego": properties}

        ego_veh = self.VehicleDefines.ego_vehicle_and_entities(egoname=egoname,
                                                               model=properties['Ego']["model"],
                                                               properties=properties["Ego"],
                                                               VehicleWidth=float(width),
                                                               VehicleLength=float(length))

        vehicle_entities.add_scenario_object(egoname, ego_veh)

    def define_target_entities(self, properties, paramlist_analysis, vehicle_entities):
        # print("yes7",properties)
        """
        Define target entities
        Parameters
        ----------
        properties

        Returns
        -------

        """
        obj_entities, obj_list = EBTB_API_data.get_obj_entities(paramlist_analysis=paramlist_analysis)

        for obj, value in obj_entities.items():
            property_car = properties.get('car')
            property_truck = properties.get('Truck')
            property_motorbike = properties.get('twowheelers')
            proprty_bicycle = properties.get('bicycle')
            proprty_pedestrian = properties.get('human')
            property_roadside = properties.get('obstacle')
            property_traffic = properties.get('pole')

            if obj_entities[obj][3] == 'car':
                if obj == 'Obj1':
                    properties1 = property_car.get('Obj1')

                    target_obj = self.VehicleDefines.target_vehicle_and_entities(target_name=obj, model="ENCAP_GVT01",
                                                                                 default_cat=False,
                                                                                 entity_type='Vehicle',
                                                                                 properties=properties1,
                                                                                 vehicle_category=value[3],
                                                                                 width=float(value[0]),
                                                                                 length=float(value[1]),
                                                                                 height=float(value[2]))
                if obj == 'Obj2':
                    properties2 = property_car.get('Obj2')
                    target_obj = self.VehicleDefines.target_vehicle_and_entities(target_name=obj,
                                                                                 model="Chevrolet Camaro",
                                                                                 default_cat=False,
                                                                                 entity_type='Vehicle',
                                                                                 properties=properties2,
                                                                                 vehicle_category=value[3],
                                                                                 width=float(value[0]),
                                                                                 length=float(value[1]),
                                                                                 height=float(value[2]))

                if obj == 'Obj3':
                    properties3 = property_car.get('Obj3')
                    target_obj = self.VehicleDefines.target_vehicle_and_entities(target_name=obj, model="Audi TT",
                                                                                 default_cat=False,
                                                                                 entity_type='Vehicle',
                                                                                 properties=properties3,
                                                                                 vehicle_category=value[3],
                                                                                 width=float(value[0]),
                                                                                 length=float(value[1]),
                                                                                 height=float(value[2]))
                if obj == 'Obj4':
                    properties4 = property_car.get('Obj4')
                    target_obj = self.VehicleDefines.target_vehicle_and_entities(target_name=obj,
                                                                                 model="Chevrolet Camaro",
                                                                                 default_cat=False,
                                                                                 entity_type='Vehicle',
                                                                                 properties=properties4,
                                                                                 vehicle_category=value[3],
                                                                                 width=float(value[0]),
                                                                                 length=float(value[1]),
                                                                                 height=float(value[2]))

                if obj == 'Obj5':
                    properties5 = property_car.get('Obj5')
                    target_obj = self.VehicleDefines.target_vehicle_and_entities(target_name=obj,
                                                                                 model="Bentley Continental GT",
                                                                                 default_cat=False,
                                                                                 entity_type='Vehicle',
                                                                                 properties=properties5,
                                                                                 vehicle_category=value[3],
                                                                                 width=float(value[0]),
                                                                                 length=float(value[1]),
                                                                                 height=float(value[2]))

                if obj == 'Obj6':
                    properties6 = property_car.get('Obj6')
                    target_obj = self.VehicleDefines.target_vehicle_and_entities(target_name=obj, model="Ferrari 458",
                                                                                 default_cat=False,
                                                                                 entity_type='Vehicle',
                                                                                 properties=properties6,
                                                                                 vehicle_category=value[3],
                                                                                 width=float(value[0]),
                                                                                 length=float(value[1]),
                                                                                 height=float(value[2]))

            elif obj_entities[obj][3] == 'bicycle':
                if obj == 'Obj1':
                    prop1 = proprty_bicycle.get('Obj1')
                    target_obj = self.VehicleDefines.target_vehicle_and_entities(target_name=obj, model="Bike01",
                                                                             default_cat=False, entity_type='Vehicle',
                                                                             properties=prop1,
                                                                             vehicle_category=value[3],
                                                                             width=float(value[0]),
                                                                             length=float(value[1]),
                                                                             height=float(value[2]))
                if obj == 'Obj2':
                    prop2 = proprty_bicycle.get('Obj2')
                    target_obj = self.VehicleDefines.target_vehicle_and_entities(target_name=obj, model="Bike05",
                                                                             default_cat=False, entity_type='Vehicle',
                                                                             properties=prop2,
                                                                             vehicle_category=value[3],
                                                                             width=float(value[0]),
                                                                             length=float(value[1]),
                                                                             height=float(value[2]))
                if obj == 'Obj3':
                    prop3 = proprty_bicycle.get('Obj3')
                    target_obj = self.VehicleDefines.target_vehicle_and_entities(target_name=obj, model="Bike02",
                                                                             default_cat=False, entity_type='Vehicle',
                                                                             properties=prop3,
                                                                             vehicle_category=value[3],
                                                                             width=float(value[0]),
                                                                             length=float(value[1]),
                                                                             height=float(value[2]))
                if obj == 'Obj4':
                    prop4 = proprty_bicycle.get('Obj4')
                    target_obj = self.VehicleDefines.target_vehicle_and_entities(target_name=obj, model="Bike03",
                                                                             default_cat=False, entity_type='Vehicle',
                                                                             properties=prop4,
                                                                             vehicle_category=value[3],
                                                                             width=float(value[0]),
                                                                             length=float(value[1]),
                                                                             height=float(value[2]))
                if obj == 'Obj5':
                    prop5 = proprty_bicycle.get('Obj5')
                    target_obj = self.VehicleDefines.target_vehicle_and_entities(target_name=obj, model="Bike06",
                                                                             default_cat=False, entity_type='Vehicle',
                                                                             properties=prop5,
                                                                             vehicle_category=value[3],
                                                                             width=float(value[0]),
                                                                             length=float(value[1]),
                                                                             height=float(value[2]))
                # vehicle_entities.add_scenario_object(obj, target_obj)

            elif obj_entities[obj][3] == 'pole':
                if obj =='Obj1':
                    pro1 = property_traffic.get('Obj1')
                    target_obj = self.VehicleDefines.target_vehicle_and_entities(target_name=obj, model="Defaced Sign 02",
                                                                             default_cat=False,
                                                                             entity_type='MiscObject',
                                                                             properties=pro1,
                                                                             vehicle_category=value[3],
                                                                             width=float(value[0]),
                                                                             length=float(value[1]),
                                                                             height=float(value[2]))
                if obj =='Obj2':
                    pro2 = property_traffic.get('Obj2')
                    target_obj = self.VehicleDefines.target_vehicle_and_entities(target_name=obj, model="Defaced Sign 03",
                                                                             default_cat=False,
                                                                             entity_type='MiscObject',
                                                                             properties=pro2,
                                                                             vehicle_category=value[3],
                                                                             width=float(value[0]),
                                                                             length=float(value[1]),
                                                                             height=float(value[2]))
                if obj =='Obj3':
                    pro3 = property_traffic.get('Obj3')
                    target_obj = self.VehicleDefines.target_vehicle_and_entities(target_name=obj, model="Defaced Sign 09",
                                                                             default_cat=False,
                                                                             entity_type='MiscObject',
                                                                             properties=pro3,
                                                                             vehicle_category=value[3],
                                                                             width=float(value[0]),
                                                                             length=float(value[1]),
                                                                             height=float(value[2]))

            elif obj_entities[obj][3] == 'obstacle':
                if obj =='Obj1':
                    proper1 = property_roadside.get('Obj1')
                    target_obj = self.VehicleDefines.target_vehicle_and_entities(target_name=obj, model="Traffic Cone 01",
                                                                             default_cat=False,
                                                                             entity_type='MiscObject',
                                                                             properties=proper1,
                                                                             vehicle_category=value[3],
                                                                             width=float(value[0]),
                                                                             length=float(value[1]),
                                                                             height=float(value[2]))
                if obj =='Obj2':
                    proper2 = property_roadside.get('Obj2')
                    target_obj = self.VehicleDefines.target_vehicle_and_entities(target_name=obj, model="Traffic Cone 02",
                                                                             default_cat=False,
                                                                             entity_type='MiscObject',
                                                                             properties=proper2,
                                                                             vehicle_category=value[3],
                                                                             width=float(value[0]),
                                                                             length=float(value[1]),
                                                                             height=float(value[2]))
                if obj =='Obj3':
                    proper3 = property_roadside.get('Obj3')
                    target_obj = self.VehicleDefines.target_vehicle_and_entities(target_name=obj, model="Road Block 01",
                                                                             default_cat=False,
                                                                             entity_type='MiscObject',
                                                                             properties=proper3,
                                                                             vehicle_category=value[3],
                                                                             width=float(value[0]),
                                                                             length=float(value[1]),
                                                                             height=float(value[2]))
                if obj =='Obj4':
                    proper4 = property_roadside.get('Obj4')
                    target_obj = self.VehicleDefines.target_vehicle_and_entities(target_name=obj, model="Plastic Barrier 01",
                                                                             default_cat=False,
                                                                             entity_type='MiscObject',
                                                                             properties=proper4,
                                                                             vehicle_category=value[3],
                                                                             width=float(value[0]),
                                                                             length=float(value[1]),
                                                                             height=float(value[2]))
                if obj =='Obj5':
                    proper5 = property_roadside.get('Obj5')
                    target_obj = self.VehicleDefines.target_vehicle_and_entities(target_name=obj, model="Plastic Barrier 02",
                                                                             default_cat=False,
                                                                             entity_type='MiscObject',
                                                                             properties=proper5,
                                                                             vehicle_category=value[3],
                                                                             width=float(value[0]),
                                                                             length=float(value[1]),
                                                                             height=float(value[2]))
                if obj =='Obj6':
                    proper6 = property_roadside.get('Obj6')
                    target_obj = self.VehicleDefines.target_vehicle_and_entities(target_name=obj, model="Warning Triangle 01",
                                                                             default_cat=False,
                                                                             entity_type='MiscObject',
                                                                             properties=proper6,
                                                                             vehicle_category=value[3],
                                                                             width=float(value[0]),
                                                                             length=float(value[1]),
                                                                             height=float(value[2]))
                if obj =='Obj7':
                    proper7 = property_roadside.get('Obj7')
                    target_obj = self.VehicleDefines.target_vehicle_and_entities(target_name=obj, model="Road Block 02",
                                                                             default_cat=False,
                                                                             entity_type='MiscObject',
                                                                             properties=proper7,
                                                                             vehicle_category=value[3],
                                                                             width=float(value[0]),
                                                                             length=float(value[1]),
                                                                             height=float(value[2]))
                if obj =='Obj8':
                    proper8 = property_roadside.get('Obj8')
                    target_obj = self.VehicleDefines.target_vehicle_and_entities(target_name=obj, model="Barricade 02",
                                                                             default_cat=False,
                                                                             entity_type='MiscObject',
                                                                             properties=proper8,
                                                                             vehicle_category=value[3],
                                                                             width=float(value[0]),
                                                                             length=float(value[1]),
                                                                             height=float(value[2]))
            elif obj_entities[obj][3] == 'pedestrian':
                if obj =='Obj1':
                    prope1 = proprty_pedestrian.get('Obj1')
                    target_obj = self.VehicleDefines.target_vehicle_and_entities(target_name=obj, model="Ped00",
                                                                             default_cat=False,
                                                                             entity_type='Pedestrian',
                                                                             properties=prope1,
                                                                             vehicle_category=value[3],
                                                                             width=float(value[0]),
                                                                             length=float(value[1]),
                                                                             height=float(value[2]))
                if obj =='Obj2':
                    prope2 = proprty_pedestrian.get('Obj2')
                    target_obj = self.VehicleDefines.target_vehicle_and_entities(target_name=obj, model="Ped03",
                                                                             default_cat=False,
                                                                             entity_type='Pedestrian',
                                                                             properties=prope2,
                                                                             vehicle_category=value[3],
                                                                             width=float(value[0]),
                                                                             length=float(value[1]),
                                                                             height=float(value[2]))
                if obj =='Obj3':
                    prope3 = proprty_pedestrian.get('Obj3')
                    target_obj = self.VehicleDefines.target_vehicle_and_entities(target_name=obj, model="Ped08",
                                                                             default_cat=False,
                                                                             entity_type='Pedestrian',
                                                                             properties=prope3,
                                                                             vehicle_category=value[3],
                                                                             width=float(value[0]),
                                                                             length=float(value[1]),
                                                                             height=float(value[2]))
            elif obj_entities[obj][3] == 'truck':
                if obj == 'Obj1':
                    propers1 = property_truck.get('Obj1')
                    target_obj = self.VehicleDefines.target_vehicle_and_entities(target_name=obj, model="Van Truck 02",
                                                                                 default_cat=False,
                                                                                 entity_type='Vehicle',
                                                                                 properties=propers1,
                                                                                 vehicle_category=value[3],
                                                                                 width=float(value[0]),
                                                                                 length=float(value[1]),
                                                                                 height=float(value[2]))
                if obj == 'Obj2':
                    propers2 = property_truck.get('Obj2')
                    target_obj = self.VehicleDefines.target_vehicle_and_entities(target_name=obj, model="Van Truck 01",
                                                                                 default_cat=False,
                                                                                 entity_type='Vehicle',
                                                                                 properties=propers2,
                                                                                 vehicle_category=value[3],
                                                                                 width=float(value[0]),
                                                                                 length=float(value[1]),
                                                                                 height=float(value[2]))

                if obj == 'Obj3':
                    propers3 = property_truck.get('Obj3')
                    target_obj = self.VehicleDefines.target_vehicle_and_entities(target_name=obj, model="Truck 03",
                                                                                 default_cat=False,
                                                                                 entity_type='Vehicle',
                                                                                 properties=propers3,
                                                                                 vehicle_category=value[3],
                                                                                 width=float(value[0]),
                                                                                 length=float(value[1]),
                                                                                 height=float(value[2]))
            elif obj_entities[obj][3] == 'motorbike':
                if obj == 'Obj1':
                    propers1_bike = property_motorbike.get('Obj1')
                    target_obj = self.VehicleDefines.target_vehicle_and_entities(target_name=obj, model="Motorcycle03",
                                                                                 default_cat=False,
                                                                                 entity_type='Vehicle',
                                                                                 properties=propers1_bike,
                                                                                 vehicle_category=value[3],
                                                                                 width=float(value[0]),
                                                                                 length=float(value[1]),
                                                                                 height=float(value[2]))
                if obj == 'Obj2':
                    propers2_bike = property_motorbike.get('Obj2')
                    target_obj = self.VehicleDefines.target_vehicle_and_entities(target_name=obj, model="Motorcycle02",
                                                                                 default_cat=False,
                                                                                 entity_type='Vehicle',
                                                                                 properties=propers2_bike,
                                                                                 vehicle_category=value[3],
                                                                                 width=float(value[0]),
                                                                                 length=float(value[1]),
                                                                                 height=float(value[2]))

            vehicle_entities.add_scenario_object(obj, target_obj)

        return obj_entities, obj_list

    def Ego_initialize(self, init, step_time, paramlist_analysis):

        """
        Ego Initialize
        Returns
        -------

        """
        try:
            global road_len, x_value, envp_lane_selection,road_id
            landmark_start_value = EBTB_API_data.ego_landmark_start_init(paramlist_analysis=paramlist_analysis)
            landmark_start_value = float(landmark_start_value)
            envp_landmark_offset = EBTB_API_data.get_landmark_offset_ego(paramlist_analysis=paramlist_analysis)
            envp_lane_selection = EBTB_API_data.get_lane_selection_ego(paramlist_analysis=paramlist_analysis)
            road_len = EBTB_API_data.extract_lenthoflane(paramlist_analysis=paramlist_analysis)
            offset = EBTB_API_data.get_ego_initialise(paramlist_analysis=paramlist_analysis)

            envp_landmark_offset = float(envp_landmark_offset)

            if offset is not None:
                offset = float(offset)
            else:
                offset = 0

            if landmark_start_value:
                x_value = float(landmark_start_value + envp_landmark_offset)
            else:
                return "Stop"
                # if road_len is not None:
                #     x_value = float((road_len / 2) + envp_landmark_offset)
                # else:
                #     x_value = float(envp_landmark_offset)

            road_id,x_value = EBTB_API_data.ego_road_id(paramlist_analysis,x_value,landmark_start_value,xlmr_file=None)

            lane_selection_dict = {
                "Right1": -1, "Right2": -2, "Right3": -3, "Right4": -4, "Right5": -5, "Right6": -6,
                "Left1": 1, "Left2": 2, "Left3": 3, "Left4": 4, "Left5": 5, "Left6": 6
            }

            # Default value for unmatched lane selection
            y_value = lane_selection_dict.get(envp_lane_selection, -4.625)

            # Calling the ego_initialize method with the selected or default y value
            self.VehicleDefines.ego_initialize(init=init, step_time=step_time, road_id=road_id,
                                               y=y_value, x=x_value, offset=offset)

        except:
            print("err")

    def Target_initialize(self, init, step_time, target_name, states_analysis, paramlist_analysis, overlap_data=None):
        from e2xostream.src.vehiclestream.xosc_stream.XOSCScenarioDevelop import FuncScenario as funcscenario
        """
        Target initialize
        Returns
        -------

        """
        try:
            ref_axis = EBTB_API_data.get_ref_axis(states_analysis=states_analysis,target_name=target_name)

            if ref_axis == "AbsSysLane":
                global road_len

                obj_landmark_offset = EBTB_API_data.get_landmark_offset(states_analysis=states_analysis,
                                                                        target_name=target_name)

                obj_landmark_offset = float(obj_landmark_offset)

                landmark_start = EBTB_API_data.obj_landmark_start_init(states_analysis=states_analysis,
                                                                       target_name=target_name,
                                                                       paramlist_analysis=paramlist_analysis)

                lane_offset = EBTB_API_data.get_obj_initialise_ver1(states_analysis=states_analysis,
                                                                    target_name=target_name)

                obj_lane_selection = EBTB_API_data.get_lane_selection_object(states_analysis=states_analysis,
                                                                             target_name=target_name)

                if lane_offset is not None:
                    offset = float(lane_offset)
                else:
                    offset = 0

                if landmark_start:
                    x_val = float(landmark_start + obj_landmark_offset)
                else:
                    return "Stop"
                    # if road_len is not None:
                    #     x_val = float((road_len / 2) + obj_landmark_offset)
                    #
                    # else:
                    #     x_val = float(obj_landmark_offset)

                road_id1, x_val = EBTB_API_data.obj_road_id(states_analysis, paramlist_analysis, target_name, x_val,
                                                            landmark_start)
                lane_selection_map = {
                    "Right1": -1,
                    "Right2": -2,
                    "Right3": -3,
                    "Right4": -4,
                    "Right5": -5,
                    "Right6": -6,
                    "Left1": 1,
                    "Left2": 2,
                    "Left3": 3,
                    "Left4": 4,
                    "Left5": 5,
                    "Left6": 6
                }

                y_val = lane_selection_map.get(obj_lane_selection, -4.625)  # Default to -4.625 if no match

                self.VehicleDefines.target_initialize(init=init, step_time=step_time, road_id=road_id1,
                                                      targetname=target_name, x=x_val, y=y_val, offset=offset)
                shared_data.obj_lane_init[target_name] = obj_lane_selection

            if ref_axis == "RelSysLane":
                global x_value, envp_lane_selection, road_id
                obj_lane_selection = envp_lane_selection

                Longitudinal, Lateral, ref_obj = EBTB_API_data.get_obj_intialise(states_analysis=states_analysis,
                                                                                 target_name=target_name)

                Longitudinal = float(Longitudinal)
                x_val = x_value + Longitudinal
                x_val = float(x_val)
                offset = float(Lateral)

                # Dictionary mapping obj_lane_selection to corresponding y values
                lane_selection_map = {
                    "Right1": -1,
                    "Right2": -2,
                    "Right3": -3,
                    "Right4": -4,
                    "Right5": -5,
                    "Right6": -6,
                    "Left1": 1,
                    "Left2": 2,
                    "Left3": 3,
                    "Left4": 4,
                    "Left5": 5,
                    "Left6": 6
                }

                # Get the corresponding y value from the dictionary, defaulting to -4.625 if no match
                y_val = lane_selection_map.get(obj_lane_selection, -4.625)

                # Single call to target_initialize with the determined y value
                self.VehicleDefines.target_initialize(init=init, step_time=step_time, road_id=road_id,
                                                      targetname=target_name, x=x_val, y=y_val, offset=offset)
                shared_data.obj_lane_init[target_name] = obj_lane_selection




        except:
            print("error")