import os
import sys
import xml.etree.ElementTree as ET

from e2xostream.src.vehiclestream.ebtb_stream import EBTB_API_data
from e2xostream.stk.vehicledynamics.DataControl import DataControls as datacontrol
from e2xostream.stk.scenariogeneration import xosc, prettyprint, ScenarioGenerator
from e2xostream.stk.scenariogeneration.xosc import actions, VehicleComponentType
from e2xostream.stk.scenariogeneration.xosc.utils import _ComponentAnimation, Color, ColorRGB, _ColorDefinition
from e2xostream.stk.vehicledynamics.VehicleControl import VehicleControlMovements as VCM
from datetime import datetime
from e2xostream.stk.vehicledynamics.position_initialization import initialize_position
from e2xostream.stk.vehicledynamics.entity_factor import EntityFactory

MAIN_PATH = os.path.abspath(os.path.join(__file__, "..", "..", "..", ".."))
CURRENT_WORKING_FILE_DIRECTORY = os.path.abspath(os.path.join(__file__))
CURRENT_WORKING_DIRECTORY = os.path.abspath(os.path.join(__file__, ".."))

if MAIN_PATH not in sys.path:
    sys.path.append(MAIN_PATH)

dir_path = os.path.dirname(os.path.realpath(__file__))
for root, dirs, files in os.walk(dir_path):
    sys.path.append(root)


class VehicleScenario:
    def __init__(self, **kwargs):
        self.KArgs = kwargs
        self.CM = VCM()
        self.processed_flags = {}

    obj1_processed = False
    obj2_processed = False

    def create_catalogs(self, VehicleCatalog="../../resources/xosc/Catalogs/Vehicles"):
        """
        Create catalogs action
        """
        catalog = xosc.Catalog()
        catalog.add_catalog("VehicleCatalog", VehicleCatalog)
        return catalog

    def create_road_network(self, XodrPath="dd878027-4265-43bd-9ff1-5fd9f688f1f7.xodr", scenegraph=""):
        """
        Road Network function
        """
        road = xosc.RoadNetwork(
            roadfile=XodrPath, scenegraph=scenegraph
        )
        return road

    def declare_parameters(self):
        """
        ParameterDeclarations function
        """
        paramdec = xosc.ParameterDeclarations()
        return paramdec

    def ego_vehicle_and_entities(self, properties, model="car_white", egoname="Ego", VehicleWidth=2, VehicleLength=5,
                                 vehcilefile="../../resources/models/car_white.osgb"):
        """
        All the entities are mapped to XOSC part which reflects bounding box, axle length etc
        """

        bb = xosc.BoundingBox(VehicleWidth, VehicleLength, 1.8, 1.48, 0, 0)
        fa = xosc.Axle(0.698, 0.678, 1.64, 2.92, 0.34)
        ba = xosc.Axle(0.698, 0.678, 1.64, 0, 0.34)

        white_veh = xosc.Vehicle(model, xosc.VehicleCategory.car, bb, fa, ba,
                                 50, 10, 10)
        # white_veh.add_property_file(vehcilefile)
        for k, v in properties.items():
            white_veh.add_property(k, v)

        return white_veh

    def target_vehicle_and_entities(self, properties, vehicle_category='car',
                                    model="ENCAP_GVT01", width=1.8, length=4.5, height=1.5,
                                    default_cat=False, entity_type='Vehicle', target_name="RedCar",
                                    max_speed=70, max_acceleration=10, max_deceleration=10,
                                    vehcilefile="../../resources/models/car_red.osgb"):
        """
        All the entities are mapped to the XOSC which reflects properties of object,model,name etc
        """

        factory = EntityFactory()

        obj_entities = factory.create_entity(
            entity_type=entity_type,
            name=target_name,
            properties=properties,
            category=vehicle_category,
            model=model,
            width=width,
            length=length,
            height=height,
            default_cat=default_cat,
            vehicle_file=vehcilefile,
            max_speed=max_speed,
            max_acceleration=max_acceleration,
            max_deceleration=max_deceleration
        )

        return obj_entities

    def global_action(self, init):
        """
        All the environment conditions are defined here
        """
        now = datetime.now()
        gloablAction = xosc.EnvironmentAction(xosc.Environment(name="sunny01",
                                                               timeofday=xosc.TimeOfDay(False, now.year, now.month,
                                                                                        now.day, now.hour, now.minute,
                                                                                        now.second),
                                                               weather=xosc.Weather(cloudstate=xosc.CloudState().free,
                                                                                    precipitation=xosc.Precipitation(
                                                                                        intensity=0,
                                                                                        precipitation="dry"),
                                                                                    sun=xosc.Sun(azimuth=0,
                                                                                                 elevation=1.57,
                                                                                                 intensity=60000),
                                                                                    fog=xosc.Fog(visual_range=1200)),
                                                               roadcondition=xosc.RoadCondition(
                                                                   friction_scale_factor="0.85",
                                                                   properties=xosc.Properties().add_property(
                                                                       "humidityLevel", "0.1"))))

        init.add_global_action(gloablAction)

    def ego_initialize(self, init, step_time, road_id, y=0, x=3.75,offset=0):
        """
        Initialise ego vehcle - Lane position
        s - x_value
        lane_id - y_value
        offset - offset
        """
        # Create a trajectory
        trajectory = xosc.Trajectory("example_trajectory", closed=False)

        # if statements for different positions based on EBTB API's
        initialize_position(init, "Ego", step_time,
                            position_types=["LanePosition"],
                            init_speed=0,
                            x=0, y=0, z=0, h=0, p=0, r=0,
                            dx=0, dy=0, dz=0,
                            road_id=road_id, s=x, t=0,
                            ds=0, dt=0,
                            lane_id=y, offset=offset,
                            d_lane=0,
                            route_ref=0,
                            trajectory=trajectory, latitude=0, longitude=0, height=0)

    def target_initialize(self, init, step_time,road_id, targetname="Obj1", x=10, y=-4.625,offset=0):
        """
        Initialise object - Lane position
        s - x
        lane_id = y
        road_id = road_id
        """

        # Create a trajectory
        trajectory = xosc.Trajectory("example_trajectory", closed=False)

        # if statements for different positions based on EBTB API's
        initialize_position(init, targetname, step_time,
                            position_types=["LanePosition"],
                            init_speed=0,
                            x=0, y=0, z=0, h=0, p=0, r=0,
                            dx=0, dy=0, dz=0,
                            road_id=road_id, s=x, t=0,
                            ds=0, dt=0,
                            lane_id=y, offset=offset,
                            d_lane=0,
                            route_ref=0,
                            trajectory=trajectory, latitude=0, longitude=0, height=0)

    def traffic_sign_initialize(self,init,step_time,road_id,t,s,towards,target_name = "TrafficSign1"):
        """
        Initialise traffic sign - Road position
        s - s
        t = t
        road_id = road_id
        """

        # Create a trajectory
        trajectory = xosc.Trajectory("example_trajectory", closed=False)

        # if statements for different positions based on EBTB API's
        initialize_position(init, target_name, step_time,
                            position_types=["RoadPosition"],
                            init_speed=0,
                            x=0, y=0, z=0, h=0, p=0, r=0,
                            dx=0, dy=0, dz=0,
                            road_id=road_id, s=s, t=t,
                            ds=0, dt=0,
                            lane_id=0, offset=0,
                            d_lane=0,
                            route_ref=0,
                            trajectory=trajectory, latitude=0, longitude=0, height=0)



    def create_maneuver_group(self, targetname, event):
        """
        Create maneuver group for ego/object
        """
        man = xosc.Maneuver("maneuver")
        man.add_event(event)

        mangr = xosc.ManeuverGroup("mangroup")
        mangr.add_actor(targetname)
        mangr.add_maneuver(man)
        return mangr

    def create_ego_event(self, value=5):
        """
        Create ego event - SimulationTimeCondition action
        """
        start_trig = xosc.ValueTrigger(
            "start_trigger_ego",
            0,
            xosc.ConditionEdge.none,
            xosc.SimulationTimeCondition(value, xosc.Rule.greaterThan),
        )

        return start_trig


    def absolute_speed_action(self, speed=30):
        """
        AbsoluteSpeedAction - spped action API
        """
        start_action = xosc.AbsoluteSpeedAction(
            speed,
            xosc.TransitionDynamics(
                xosc.DynamicsShapes.step, xosc.DynamicsDimension.time, 3
            ),
        )

        return start_action

    def create_lateral_distance_action(self, value):
        """
        AbsoluteLaneOffsetAction API
        """
        later_control_action = xosc.AbsoluteLaneOffsetAction(value, shape=xosc.DynamicsShapes.linear, maxlatacc=10,
                                                             continuous=True)
        return later_control_action



    def create_lateral_reference_action(self, lane):
        """
        AbsoluteLaneChangeAction API - change the reference axis
        """
        lateral_reference_action = xosc.AbsoluteLaneChangeAction(lane, transition_dynamics=xosc.TransitionDynamics(
            xosc.DynamicsShapes.linear, xosc.DynamicsDimension.time, value=None)
                                                                 , target_lane_offset=None)
        return lateral_reference_action

    def create_obj_lateral_distance_action(self, value,target_name, entity,abs_or_rel, state_data=None):
        """
        Relative/Absolute LaneOffset Action - based on distance
        """
        if entity == "SysVehicle":
            entity = "Ego"
        try:
            for k, v in state_data.items():
                for kv, vv in v["ObjectActions"].items():
                    if target_name == kv:
                        if abs_or_rel == "RelSysLane":
                            later_control_action = xosc.RelativeLaneOffsetAction(value, entity,
                                                                             shape=xosc.DynamicsShapes.linear,
                                                                             maxlatacc=10, continuous=True)
                        if abs_or_rel == "AbsSysLane":
                            later_control_action = xosc.AbsoluteLaneOffsetAction(value,
                                                                                 shape=xosc.DynamicsShapes.linear,
                                                                                 maxlatacc=10, continuous=True)


                        return later_control_action
        except Exception as e:
            print(e)
    def create_obj_lateral_rel_position(self,target_name,units,entity,displacement):
        """
        RelativeLaneOffsetAction API - lane offset to shift based on distnace
        """
        displacement = float(displacement)
        if entity == "SysVehicle":
            entity = "Ego"

        if units == "%":
            displacement = displacement * 1.86 * 0.01

        later_control_act = xosc.RelativeLaneOffsetAction(displacement, entity,
                                                             shape=xosc.DynamicsShapes.linear,
                                                             maxlatacc=10, continuous=False)
        return later_control_act

    def obj_relative_position(self,target_name,state_data,entity,distance):
        """
        LongitudinalDistanceAction API
        """
        if entity == "SysVehicle":
            entity = "Ego"
        relative_position = xosc.LongitudinalDistanceAction(entity,distance)
        return relative_position



    def create_obj_lanechange_action(self, obj_id, direction, present_lane,value_of_dist, state_data=None, param_data=None):
        """
        AbsoluteLaneChangeAction - Change the lane based on lane_value for object
        """
        try:
            lane_mapping = {
                ("Right1", "Right"): -2,
                ("Right2", "Right"): -3,
                ("Right3", "Right"): -4,
                ("Right4", "Right"): -5,
                ("Right5", "Right"): -6,
                ("Left1","Right") : 2,
                ("Left2", "Right"): 3,
                ("Left3", "Right"): 4,
                ("Left4", "Right"): 5,
                ("Left5", "Right"): 6,
                ("Right2", "Left"): -1,
                ("Right3", "Left"): -2,
                ("Right4", "Left"): -3,
                ("Right5", "Left"): -4,
                ("Right6", "Left"): -5,
                ("Left2", "Left"): 1,
                ("Left3", "Left"): 2,
                ("Left4", "Left"): 3,
                ("Left5", "Left"): 4,
                ("Left6", "Left"): 5,
            }

            # Retrieve lane change value
            lane_change_value = lane_mapping.get((present_lane, direction))

            lane_map = {
                -1: "Right1", -2: "Right2", -3: "Right3", -4: "Right4", -5: "Right5", -6: "Right6",
                1: "Left1", 2: "Left2", 3: "Left3", 4: "Left4", 5: "Left5", 6: "Left6"
            }

            upd_lane = lane_map.get(lane_change_value, None)


            if lane_change_value is not None:
                lane_change_action = xosc.AbsoluteLaneChangeAction(
                    lane_change_value,
                    transition_dynamics=xosc.TransitionDynamics(
                        xosc.DynamicsShapes.linear,
                        xosc.DynamicsDimension.time,
                        value=value_of_dist,
                    ),
                    target_lane_offset=None,
                )

            return lane_change_action,upd_lane
        except Exception as e:
            print(e)

    def create_lanechange_ego(self, present_lane,direction,count,target_disp):
        """
        AbsoluteLaneChangeAction - Change the lane based on lane_value for ego
        """
        try:

            lane_selection_dict = {
                "Right1": -1, "Right2": -2, "Right3": -3, "Right4": -4, "Right5": -5, "Right6": -6,
                "Left1": 1, "Left2": 2, "Left3": 3, "Left4": 4, "Left5": 5, "Left6": 6
            }

            lane_current = lane_selection_dict.get(present_lane)

            if direction == "Right":
                count = -int(count)
            if direction == "Left":
                count = int(count)

            lane_current = int(lane_current)
            final_lane = lane_current

            # Step-wise addition, skipping 0
            step = -1 if count > 0 else 1  # Determine direction

            for _ in range(abs(count)):  # Loop for the count value
                final_lane += step
                if final_lane == 0:  # If 0 is encountered, skip it
                    final_lane += step

            print("Final Lane:", final_lane)

            if final_lane is not None:
                lane_change_action = xosc.AbsoluteLaneChangeAction(
                    final_lane,
                    transition_dynamics=xosc.TransitionDynamics(
                        xosc.DynamicsShapes.linear,
                        xosc.DynamicsDimension.time,
                        value= 1,
                    ),
                    target_lane_offset=target_disp,
                )

            return lane_change_action
        except Exception as e:
            print(e)


    def create_set_indicator_state(self, State_Indicator, State_Duration):
        """
        LightStateAction action
        """
        ColorRgb = xosc.ColorRGB(1.0, 0.0, 0.0)
        color = xosc.Color("red", color_definition=ColorRgb)

        set_indicator_light = xosc.LightStateAction(
            State_Indicator,
            mode="on",
            transition_time=State_Duration,
            flashing_off_duration=0.5,
            flashing_on_duration=0.5,
            intensity=1000,
            color=color
        )

        return set_indicator_light

    def create_setVehicleDoor(self, vehicle_door_value, vehicle_door_state_value):
        """
        AnimationAction action - Set Vehicle Door value
        """
        animation_type = _ComponentAnimation("trunk")

        if vehicle_door_state_value == "Opened":
            vehicle_door_action = xosc.AnimationAction(animation_type=animation_type, duration=5, loop=None, state=1)
        elif vehicle_door_state_value == "Closed":
            vehicle_door_action = xosc.AnimationAction(animation_type=animation_type, duration=5, loop=None, state=0)
        return vehicle_door_action

    def create_position_action(self, x=0, y=0, z=0):
        """
        TeleportAction for position assignment
        """
        position = xosc.WorldPosition(x, y, z)
        position_action = xosc.TeleportAction(position)
        return position_action

    def create_target_event(self, value=5):
        """
        Create target event
        """
        start_trig = xosc.ValueTrigger(
            "target_acc",
            0,
            xosc.ConditionEdge.none,
            xosc.SimulationTimeCondition(value, xosc.Rule.greaterThan),
        )

        return start_trig

    def create_offroad_condition_trigger(self, duration=5.0, entity_name="ego_vehicle"):
        """
        OffroadCondition trigger
        """
        offroad_cond = xosc.OffroadCondition(duration)
        offroad_trig = xosc.EntityTrigger(
            "offroad_trigger",
            0,
            xosc.ConditionEdge.none,
            offroad_cond,
            entity_name
        )
        return offroad_trig

    def create_time_headway_condition_trigger(self, headway, relational_operator, target_entity, entity_name):
        """
        TimeHeadwayCondition with target_entity and condition rule
        """
        if target_entity == "SysVehicle":
            target_entity = "Ego"

        if relational_operator == "Less":
            time_headway_cond = xosc.TimeHeadwayCondition(target_entity, headway, xosc.Rule.lessThan)
        if relational_operator == "Greater":
            time_headway_cond = xosc.TimeHeadwayCondition(target_entity, headway, xosc.Rule.greaterThan)

        if entity_name == "SysVehicle":
            entity_name = "Ego"

        time_headway_trig = xosc.EntityTrigger(
            "time_headway_trigger",
            0,
            xosc.ConditionEdge.none,
            time_headway_cond,
            entity_name
        )
        return time_headway_trig

    def create_collision_condition_trigger(self, entity_name="ego_vehicle", target_entity="target_vehicle"):
        """
        CollisionCondition trigger - check collision possibility
        """
        collision_cond = xosc.CollisionCondition(target_entity)
        collision_trig = xosc.EntityTrigger(
            "collision_trigger",
            0,
            xosc.ConditionEdge.none,
            collision_cond,
            entity_name
        )
        return collision_trig

    def create_distance_condition_trigger(self, distance=100, entity_name="ego_vehicle"):
        """
        DistanceCondition trigger - based on entity_name and check condition
        """
        position = xosc.WorldPosition(0, 0, 0, 0)  # Example position, customize as needed
        distance_cond = xosc.DistanceCondition(distance, xosc.Rule.lessThan, position)
        distance_trig = xosc.EntityTrigger(
            "distance_trigger",
            0,
            xosc.ConditionEdge.none,
            distance_cond,
            entity_name
        )
        return distance_trig

    def create_speed_condition_trigger(self, operator, entity_name="ego_vehicle", speed=10):
        """
        SpeedCondition trigger - based on check condition and speed value
        """
        if operator == "Less":
            speed_cond = xosc.SpeedCondition(speed, xosc.Rule.lessThan)
        if operator == "Greater":
            speed_cond = xosc.SpeedCondition(speed, xosc.Rule.greaterThan)
        if operator == "WithinToleranceForDuration":
            speed_cond = xosc.SpeedCondition(speed, xosc.Rule.equalTo)
        if operator == "equalTo":
            speed_cond = xosc.SpeedCondition(speed, xosc.Rule.equalTo)


        speed_trig = xosc.EntityTrigger(
            "speed_trigger",
            0,
            xosc.ConditionEdge.none,
            speed_cond,
            entity_name
        )
        return speed_trig

    def create_standstill_condition_trigger(self, duration=10.0, entity_name="ego_vehicle"):
        """
        StandStillCondition trigger
        """
        standstill_cond = xosc.StandStillCondition(duration)
        standstill_trig = xosc.EntityTrigger(
            "standstill_trigger",
            0,
            xosc.ConditionEdge.none,
            standstill_cond,
            entity_name
        )
        return standstill_trig

    def create_traveled_distance_condition_trigger(self, distance=50.0, entity_name="ego_vehicle"):
        """
        TraveledDistanceCondition - based on distance value
        """
        traveled_distance_cond = xosc.TraveledDistanceCondition(distance)
        traveled_distance_trig = xosc.EntityTrigger(
            "traveled_distance_trigger",
            0,
            xosc.ConditionEdge.none,
            traveled_distance_cond,
            entity_name
        )
        return traveled_distance_trig

    def create_relative_speed_condition_trigger(self, relative_speed=5.0, entity_name="ego_vehicle",
                                                target_entity="target_vehicle"):
        """
        RelativeSpeedCondition trigger
        Use relative speed with respect to entity when it satisfies the condition
        """
        relative_speed_cond = xosc.RelativeSpeedCondition(relative_speed, xosc.Rule.lessThan, target_entity)
        relative_speed_trig = xosc.EntityTrigger(
            "relative_speed_trigger",
            0,
            xosc.ConditionEdge.none,
            relative_speed_cond,
            entity_name
        )
        return relative_speed_trig

    def create_relative_distance_condition_trigger(self, distance, relational_operator, target_entity, object_id):
        """
        RelativeDistanceCondition trigger
        Use relative distance with respect to entity when it satisfies the condition
        """
        if relational_operator == "Auto" or "None":
            relative_distance_cond = xosc.RelativeDistanceCondition(distance, xosc.Rule.lessThan,
                                                                    xosc.RelativeDistanceType.longitudinal,
                                                                    object_id)
        if relational_operator == "Less":
            relative_distance_cond = xosc.RelativeDistanceCondition(distance, xosc.Rule.lessThan,
                                                                    xosc.RelativeDistanceType.longitudinal,
                                                                    object_id)
        if relational_operator == "Greater":
            relative_distance_cond = xosc.RelativeDistanceCondition(distance, xosc.Rule.greaterThan,
                                                                    xosc.RelativeDistanceType.longitudinal,
                                                                    object_id)
        if target_entity == "SysVehicle":
            target_entity = "Ego"

        relative_distance_trig = xosc.EntityTrigger(
            "relative_distance_trigger",
            0,
            xosc.ConditionEdge.none,
            relative_distance_cond,
            target_entity
        )
        return relative_distance_trig

    def create_acceleration_condition_trigger(self, acceleration=2.0, entity_name="ego_vehicle"):
        """
        AccelerationCondition trigger
        Takes acceleration value when it satisfies the condition check
        """
        acceleration_cond = xosc.AccelerationCondition(acceleration, xosc.Rule.greaterThan)
        acceleration_trig = xosc.EntityTrigger(
            "acceleration_trigger",
            0,
            xosc.ConditionEdge.none,
            acceleration_cond,
            entity_name
        )
        return acceleration_trig

    def create_reach_position_condition_trigger(self, x=0.0,y=0.0,tolerance=1.0):
        """
        ReachPositionCondition trigger
        Takes position to reach and tolerance value as inputs
        """
        entity_name = "Ego"
        z = 0.0

        position = xosc.WorldPosition(x, y, z, 0)
        reach_position_cond = xosc.ReachPositionCondition(position, tolerance)
        reach_position_trig = xosc.EntityTrigger(
            "reach_position_trigger",
            0,
            xosc.ConditionEdge.none,
            reach_position_cond,
            entity_name
        )
        return reach_position_trig

    def create_time_to_collision_condition_trigger(self, time=3.0,rule="Less",target_entity="target_vehicle"):
        """
        TimeToCollisionCondition trigger
        Calculates time to collision with resepct to entity and considers check condition
        """
        entity_name = "Ego"
        if rule == "Less":
            time_to_collision_cond = xosc.TimeToCollisionCondition(time, xosc.Rule.lessThan, entity=target_entity)
        elif rule == "Greater":
            time_to_collision_cond = xosc.TimeToCollisionCondition(time, xosc.Rule.greaterThan, entity=target_entity)
        elif rule == "equalTo":
            time_to_collision_cond = xosc.TimeToCollisionCondition(time, xosc.Rule.equalTo, entity=target_entity)

        time_to_collision_trig = xosc.EntityTrigger(
            "time_to_collision_trigger",
            0,
            xosc.ConditionEdge.none,
            time_to_collision_cond,
            entity_name
        )
        return time_to_collision_trig

    def create_simulation_time_condition_trigger(self, time=10.0):
        """
        SimulationTimeCondition trigger
        Start the trigger if simulation time satisfies the condition check
        """
        simulation_time_cond = xosc.SimulationTimeCondition(time, xosc.Rule.greaterThan)
        simulation_time_trig = xosc.ValueTrigger(
            "simulation_time_trigger",
            0,
            xosc.ConditionEdge.none,
            simulation_time_cond
        )
        return simulation_time_trig

    def create_parameter_condition_trigger(self, parameter_name="speed_limit", parameter_value=50):
        """
        ParameterCondition trigger
        """
        parameter_cond = xosc.ParameterCondition(parameter_name, xosc.Rule.equalTo, parameter_value)
        parameter_trig = xosc.ValueTrigger(
            "parameter_trigger",
            0,
            xosc.ConditionEdge.none,
            parameter_cond
        )
        return parameter_trig

    def create_variable_condition_trigger(self, variable_name="traffic_density", variable_value=0.8):
        """
        VariableCondition trigger
        """
        variable_cond = xosc.VariableCondition(variable_name, xosc.Rule.greaterThan, variable_value)
        variable_trig = xosc.ValueTrigger(
            "variable_trigger",
            0,
            xosc.ConditionEdge.none,
            variable_cond
        )
        return variable_trig

    def create_user_defined_value_condition_trigger(self, value_name="custom_value", value=100):
        """
        UserDefinedValueCondition trigger
        """
        user_defined_value_cond = xosc.UserDefinedValueCondition(value_name, xosc.Rule.equalTo, value)
        user_defined_value_trig = xosc.ValueTrigger(
            "user_defined_value_trigger",
            0,
            xosc.ConditionEdge.none,
            user_defined_value_cond
        )
        return user_defined_value_trig

    def create_custom_command_action(self, value):
        """
        CustomCommandAction
        Comes under UserDefinedAction
        Takes a string as input
        Useful for creating dummy functions
        """
        if value == '{"ActorStateAction": {"enabled": true, "name": "DisabledVehicle"}}':
            message = xosc.CustomCommandAction(type="SimOneExtensionAdd", content=value)
        else :
            message = xosc.CustomCommandAction(type="str", content=value)
        user_defined = xosc.UserDefinedAction(message)
        return user_defined

    def create_traffic_signal_condition_trigger(self, signal_id="signal_1", signal_state="green"):
        """
        TrafficSignalCondition trigger
        """
        traffic_signal_cond = xosc.TrafficSignalCondition(signal_id, signal_state)
        traffic_signal_trig = xosc.ValueTrigger(
            "traffic_signal_trigger",
            0,
            xosc.ConditionEdge.none,
            traffic_signal_cond
        )
        return traffic_signal_trig

    def create_traffic_signal_controller_condition_trigger(self, controller_id="controller_1", state="stop"):
        """
        TrafficSignalControllerCondition trigger - based on traffic signals
        """
        traffic_signal_controller_cond = xosc.TrafficSignalControllerCondition(controller_id, state)
        traffic_signal_controller_trig = xosc.ValueTrigger(
            "traffic_signal_controller_trigger",
            0,
            xosc.ConditionEdge.none,
            traffic_signal_controller_cond
        )
        return traffic_signal_controller_trig

    def create_storyboard_element_state_condition_trigger(self, element_name="scenario_1",delay=0, element_state="completeState", type ="action"):
        """
        StoryboardElementStateCondition trigger
        """
        storyboard_element_state_cond = xosc.StoryboardElementStateCondition(type, element_name, element_state)
        storyboard_element_state_trig = xosc.ValueTrigger(
            "storyboard_element_state_trigger",
            delay,
            xosc.ConditionEdge.none,
            storyboard_element_state_cond
        )
        return storyboard_element_state_trig

    def create_time_of_day_condition_trigger(self, rule, year, month, day, hour, minute, second):
        """
        TimeOfDayCondition trigger
        """
        time_of_day_cond = xosc.TimeOfDayCondition(
            rule=rule,
            year=year,
            month=month,
            day=day,
            hour=hour,
            minute=minute,
            second=second
        )
        time_of_day_trig = xosc.ValueTrigger(
            "time_of_day_trigger",
            0,
            xosc.ConditionEdge.none,
            time_of_day_cond
        )
        return time_of_day_trig

    def create_target_action(self, speed=30):
        """
        AbsoluteSpeedAction - speed value
        """
        start_action = xosc.AbsoluteSpeedAction(
            speed,
            xosc.TransitionDynamics(
                xosc.DynamicsShapes.step, xosc.DynamicsDimension.time, 3
            ),
        )

        return start_action

    def create_lane_change_action(self, lane=1, transition_time=3):
        """
        AbsoluteLaneChangeAction - directly takes lane value
        """
        lane_change_action = xosc.AbsoluteLaneChangeAction(
            lane,
            xosc.TransitionDynamics(
                xosc.DynamicsShapes.step, xosc.DynamicsDimension.time, transition_time
            ),
        )
        return lane_change_action


    def create_longitudinal_distance_action(self, entity_ref, distance=10.0, continuous=True):
        """
        LongitudinalDistanceAction
        """
        longitudinal_distance_action = xosc.LongitudinalDistanceAction(
            entity_ref,
            distance=distance,
            continuous=continuous
        )
        return longitudinal_distance_action

    def create_teleport_action(self, position):
        """
        TeleportAction action
        """
        teleport_action = xosc.TeleportAction(
            xosc.WorldPosition(
                x=position[0], y=position[1], z=position[2], h=position[3], p=position[4], r=position[5]
            )
        )
        return teleport_action

    def create_speed_profile_action(self, speeds, times=None):
        """
        SpeedProfileAction action
        """
        speed_profile_action = xosc.SpeedProfileAction(
            speeds,
            xosc.FollowingMode.position,
            times=times,
            dynamics_constraint=xosc.DynamicsConstraints(max_speed=50.0)
        )
        return speed_profile_action

    def create_relative_lane_change_action(self, lane_offset=1, entity_ref='vehicle_1', transition_time=3):
        """
        RelativeLaneChangeAction action with respect to another entity vehicle
        """
        relative_lane_change_action = xosc.RelativeLaneChangeAction(
            lane=lane_offset,
            entity=entity_ref,
            transition_dynamics=xosc.TransitionDynamics(
                xosc.DynamicsShapes.step, xosc.DynamicsDimension.time, transition_time
            )
        )
        return relative_lane_change_action

    def create_assign_route_action(self, route):
        """
        AssignRouteAction action
        """
        assign_route_action = xosc.AssignRouteAction(
            route=xosc.Route(name="route_action"))
        return assign_route_action

    def create_follow_trajectory_action(self, trajectory, following_mode=xosc.FollowingMode.position):
        """
        FollowTrajectoryAction action
        """
        follow_trajectory_action = xosc.FollowTrajectoryAction(
            trajectory=xosc.Trajectory(name="Trajectory", closed=True),
            following_mode=following_mode
        )
        return follow_trajectory_action

    def create_activate_controller_action(self, lateral=True, longitudinal=True):
        """
        ActivateControllerAction action
        """
        activate_controller_action = xosc.ActivateControllerAction(
            lateral=lateral,
            longitudinal=longitudinal
        )
        return activate_controller_action

    def create_controller_action(self, controller, activate_lateral=True, activate_longitudinal=True):
        """
        ControllerAction action for AssignControllerAction
        """
        assign_controller_action = xosc.AssignControllerAction(
            controller=controller,
            activateLateral=activate_lateral,
            activateLongitudinal=activate_longitudinal
        )
        controller_action = xosc.ControllerAction(assignControllerAction=assign_controller_action)
        return controller_action

    def create_storyboard(self, init, act):
        """
        Create story board - 51Simone option
        """
        sb = xosc.StoryBoard(init, xosc.ValueTrigger("stop_simulation", 0, xosc.ConditionEdge.rising,
                                                     xosc.SimulationTimeCondition(300, xosc.Rule.greaterThan),
                                                     "stop"))
        sb.add_act(act)
        return sb

    def create_storyboard_vcar(self, init, act):
        """
        Create story board - VCAR option
        """
        sb = xosc.StoryBoard(init, xosc.ValueTrigger("end", 0, xosc.ConditionEdge.none,
                                                     xosc.SimulationTimeCondition(666, xosc.Rule.greaterThan),
                                                     "stop"))
        sb.add_act(act)
        return sb

    def assemble_scenario(self, catalog, road, paramdec, entities, storyboard, open_scenario_version):
        """
        Used to generate scenario
        """
        sce = xosc.Scenario("adapt_speed_example", "51SimeOne", paramdec, entities=entities, storyboard=storyboard,
                            roadnetwork=road, catalog=catalog, osc_minor_version=open_scenario_version)
        return sce

    def start_trigger(self, time_delay=10):
        """
        SimulationTimeCondition trigger
        """
        starttrigger = xosc.ValueTrigger(
            "starttrigger",
            0,
            xosc.ConditionEdge.rising,
            xosc.SimulationTimeCondition(time_delay, xosc.Rule.greaterThan),
        )

        return starttrigger
    def start_trigger_vcar(self, name="EgoPrepareCompleted", value=1):
        """
        SimulationTimeCondition trigger if VCAR option is chosen
        """
        starttrigger = xosc.ValueTrigger(
            "ParameterCondition",
            0,
            xosc.ConditionEdge.none,
            xosc.UserDefinedValueCondition(name,value,xosc.Rule.equalTo),
        )
        return starttrigger

    def ego_acceleration_actions(self, state_data=None, param_data=None,dict=None):
        """
        Args:
            state_data
            param_data
            dict: None

        Maps the extracted parameters to AbsoluteSpeedAction
        """
        speed, transition_time,transition_distance,transition_acceleration = EBTB_API_data.get_ego_speed_transition_time(dict,states_analysis=state_data)
        try:

            if transition_time:
                transition_dynamics = xosc.TransitionDynamics(xosc.DynamicsShapes.linear,
                                                              xosc.DynamicsDimension.time,
                                                              transition_time)
                acc_action = xosc.AbsoluteSpeedAction(speed, transition_dynamics)
            elif transition_distance:
                transition_dynamics = xosc.TransitionDynamics(xosc.DynamicsShapes.linear,
                                                              xosc.DynamicsDimension.distance,
                                                             transition_distance)
                acc_action = xosc.AbsoluteSpeedAction(speed, transition_dynamics)
            elif transition_acceleration:
                transition_dynamics = xosc.TransitionDynamics(xosc.DynamicsShapes.step,
                                                              xosc.DynamicsDimension.rate,
                                                              transition_acceleration)
                acc_action = xosc.AbsoluteSpeedAction(speed, transition_dynamics)
            else:
                transition_dynamics = xosc.TransitionDynamics(xosc.DynamicsShapes.linear,
                                                              xosc.DynamicsDimension.time,
                                                              transition_time)
                acc_action = xosc.AbsoluteSpeedAction(speed, transition_dynamics)
            acc_action.insert(0, acc_action)

        except:
            pass
        return acc_action

    def obj_acceleration_actions(self, target_name, state_data, param_data,dict=None):
        """
        Args:
            target_name: Object ID
            state_data
            param_data
            dict: None

        Maps the extracted parameters to AbsoluteSpeedAction
        """
        acc_action =[]

        try:
            speed, transition_time, transition_distance, transition_acceleration = EBTB_API_data.get_obj_speed_transition_time(
                                    target_name,dict,
                                    states_analysis=state_data)
            if transition_time:
                transition_dynamics = xosc.TransitionDynamics(xosc.DynamicsShapes.linear,
                                                              xosc.DynamicsDimension.time,
                                                              transition_time)
                acc_action = xosc.AbsoluteSpeedAction(speed, transition_dynamics)
            elif transition_distance:
                transition_dynamics = xosc.TransitionDynamics(xosc.DynamicsShapes.linear,
                                                              xosc.DynamicsDimension.distance,
                                                             transition_distance)
                acc_action = xosc.AbsoluteSpeedAction(speed, transition_dynamics)
            elif transition_acceleration:
                transition_dynamics = xosc.TransitionDynamics(xosc.DynamicsShapes.step,
                                                              xosc.DynamicsDimension.rate,
                                                              transition_acceleration)
                acc_action = xosc.AbsoluteSpeedAction(speed, transition_dynamics)
            else:
                transition_dynamics = xosc.TransitionDynamics(xosc.DynamicsShapes.linear,
                                                              xosc.DynamicsDimension.time,
                                                              transition_time)
                acc_action = xosc.AbsoluteSpeedAction(speed, transition_dynamics)


            acc_action.insert(0,acc_action)

        except Exception as e:
            pass
        return acc_action


    def define_ego_action_event(self, start_trig, start_action, event_name="startevent", action_name="start_action"):
        """
        Args:
            start_trig
            start_action
            event_name
            action_name

        Defines an ego event with trigger and action
        """
        ego_start_action_event = xosc.Event(event_name, xosc.Priority.overwrite)
        ego_start_action_event.add_trigger(start_trig)
        ego_start_action_event.add_action(action_name, start_action)

        return ego_start_action_event

    def define_target_action_event(self, start_trig, start_action, event_name="startevent", action_name="start_action"):
        """
        Args:
            start_trig
            start_action
            event_name
            action_name

        Defines an object event with trigger and action
        """
        target_start_action_event = xosc.Event(event_name, xosc.Priority.overwrite)
        target_start_action_event.add_trigger(start_trig)
        target_start_action_event.add_action(action_name, start_action)

        return target_start_action_event

    def acceleration_trigger_condition_events(self, egoname, obj_speed, obj_transition_time):
        """
        Args:
            egoname
            obj_speed
            obj_transition_time
        """
        trigcond = xosc.AccelerationCondition(2.9, xosc.Rule.greaterThan)
        trigger = xosc.EntityTrigger("testtrigger", 0.2, xosc.ConditionEdge.none, trigcond, egoname)

        event = xosc.Event("firstevent", xosc.Priority.overwrite)
        event.add_trigger(trigger)

        sin_time = xosc.TransitionDynamics(xosc.DynamicsShapes.step, xosc.DynamicsDimension.time, obj_transition_time)
        action = xosc.AbsoluteSpeedAction(obj_speed, sin_time)
        event.add_action("newspeed", action)

        return event

    def cutin_maneuver(self, targetname, lane=2, duration=5, start_time=3):
        """
        Create a cut-in maneuver for a target vehicle that starts at a specific simulation time.
        """
        sim_time_condition = xosc.SimulationTimeCondition(rule=xosc.Rule.greaterThan, value=start_time)

        trigger = xosc.ValueTrigger("cutinTrigger", 0.1, xosc.ConditionEdge.rising, sim_time_condition)

        lane_change_action = xosc.RelativeLaneChangeAction(
            lane,
            targetname,
            xosc.TransitionDynamics(xosc.DynamicsShapes.linear, xosc.DynamicsDimension.rate, 3),
            3,
        )
        event = xosc.Event("cutInEvent", xosc.Priority.overwrite)
        event.add_trigger(trigger)
        event.add_action("newspeed", lane_change_action)
        return event

    def create_controller_override_action(self, throttle=0, brake=0, clutch=0, parkingbrake=0, steeringwheel=0, gear=0):
        """
        Args:
            throttle
            brake
            clutch
            parkingbrake
            steeringwheel
            gear

        Activate whichever parameter is present - OverrideControllerValueAction API
        """
        override_controller_value_action = xosc.OverrideControllerValueAction()

        if throttle == 0:
            override_controller_value_action.set_throttle(active=False, value=throttle)
        else:
            override_controller_value_action.set_throttle(active=True, value=throttle)

        if brake == 0:
            override_controller_value_action.set_brake(active=False, value=brake)
        else:
            override_controller_value_action.set_brake(active=True, value=brake)

        if gear == 0:
            override_controller_value_action.set_gear(active=False, value=gear)
        else:
            if gear == "D":
                gear = 1.0
            elif gear == "R":
                gear = 2.0
            elif gear == "N":
                gear = 0.0
            elif gear == "P":
                gear = 3.0

            override_controller_value_action.set_gear(active=True, value=gear)

        if steeringwheel == 0:
            override_controller_value_action.set_steeringwheel(active=False, value=steeringwheel)
        else:
            override_controller_value_action.set_steeringwheel(active=True, value=steeringwheel)

        if parkingbrake == 0:
            override_controller_value_action.set_parkingbrake(active=False, value=parkingbrake)
        else:
            override_controller_value_action.set_parkingbrake(active=True, value=parkingbrake)

        override_controller_value_action.set_clutch(active=False, value=clutch)
        xosc.ControllerAction(overrideControllerValueAction=override_controller_value_action)

        return override_controller_value_action

    def add_distance_condition_start_trigger(self, target_entity_ref="Obj1", triggering_entity="Ego", value=5.8,
                                             rule="lessThan"):
        """
        Args:
            target_entity_ref: Entity vehicle
            triggering_entity: Triggering vehicle
            value: Value of distance
            rule: Condition check

        DistanceCondition - trigger for the action
        """
        ref_position = xosc.RelativeObjectPosition(dx=0, dy=0, dz=0, entity=target_entity_ref,
                                                   orientation=xosc.Orientation(h=0, p=0, r=0))

        distance_condition = xosc.DistanceCondition(value=value, rule=rule, freespace=True, alongroute=False,
                                                    position=ref_position)

        triggering_entities = xosc.TriggeringEntities("all")
        triggering_entities.add_entity(triggering_entity)

        condition_trigger = xosc.EntityTrigger(
            name="DistanceCondition",
            delay=0,  # Assuming immediate trigger without delay
            entitycondition=distance_condition,
            conditionedge=xosc.ConditionEdge.none,  # No specific edge required for triggering
            triggerentity=triggering_entity,
            triggeringrule=xosc.TriggeringEntitiesRule.all  # List of entities that can trigger this condition
        )

        return condition_trigger

    def add_time_to_collision_start_trigger(self, target_entity_ref="Obj1", triggering_entity="Ego", value=4,
                                            rule="lessThan"):
        """
        Args:
            target_entity_ref: Entity vehicle
            triggering_entity: Triggering vehicle
            value: Value of time
            rule: Condition check

        TimeToCollisionCondition check - trigger for the action
        """

        TTCrule = getattr(xosc.Rule, rule, None)
        ref_position = xosc.RelativeWorldPosition(dx=0, dy=0, dz=0, entity=target_entity_ref,
                                                  orientation=xosc.Orientation(h=0, p=0, r=0))
        # Create the condition for time to collision
        time_to_collision_condition = xosc.TimeToCollisionCondition(rule=TTCrule, value=value, position=ref_position)

        # Create a trigger for the condition
        condition_trigger = xosc.EntityTrigger(
            name="TimeToCollisionTrigger",
            delay=0,  # Assuming immediate trigger without delay
            entitycondition=time_to_collision_condition,
            conditionedge=xosc.ConditionEdge.none,  # No specific edge required for triggering
            triggerentity=triggering_entity,
            triggeringrule=xosc.TriggeringEntitiesRule.any  # List of entities that can trigger this condition
        )

        return condition_trigger

    def lane_set_lateral_ref(self,ego_lane,lane_value_str,lane_split_ego):
        """
        Args:
            ego_lane: Lane of ego vehicle
            lane_value_str: Lane value in string
            lane_split_ego

        To get the lane value - calculation used in Set_LateralReference API
        """
        lane_mapping_right = {
            "Right1": -2,
            "Right2": -3,
            "Right3": -4,
            "Right4": -5,
            "Right5": -6,
            "Left2": 1,
            "Left3": 2,
            "Left4": 3,
            "Left5": 4,
            "Left6": 5
        }

        # Assuming lane_value_str is always "Right" in this logic
        if lane_value_str == "Right":
            lane_value = lane_mapping_right.get(ego_lane, None)

        lane_mapping_left = {
            "Right2": -1,
            "Right3": -2,
            "Right4": -3,
            "Right5": -4,
            "Right6": -5,
            "Left1": 2,
            "Left2": 3,
            "Left3": 4,
            "Left4": 5,
            "Left5": 6
        }

        # Assuming lane_value_str is "Left"
        if lane_value_str == "Left":
            lane_value = lane_mapping_left.get(ego_lane, None)


        if lane_value_str == "Remain":
            lane_value = lane_split_ego

        return lane_value