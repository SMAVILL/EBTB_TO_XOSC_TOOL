import os
import sys
from e2xostream.stk.scenariogeneration import xosc, prettyprint, ScenarioGenerator


def initialize_position(init, entity_name, step_time, position_types=["LanePosition"], **kwargs):
    """
    Initialize the entity with various position types.

    Parameters:
        init: The init object to add actions to.
        entity_name: The name of the entity to initialize (e.g., "Ego" or other object names).
        step_time: The time step for the speed action.
        ini_speed: The initial speed of the entity.
        position_types: A list of position types to use (e.g., ["WorldPosition", "LanePosition"]).
        **kwargs: Additional arguments specific to the position types.

    Position Type Specific Parameters:
        For "WorldPosition":
            x (float): X-coordinate (default: 3.75).
            y (float): Y-coordinate (default: -4.625).
            z (float): Z-coordinate (default: 0).
            h (float): Heading (default: 0).
            p (float): Pitch (default: 0).
            r (float): Roll (default: 0).

        For "RelativeWorldPosition" and "RelativeObjectPosition":
            entity (str): Reference entity (default: "TargetEntity").
            dx (float): Relative X-coordinate (default: 0).
            dy (float): Relative Y-coordinate (default: 0).
            dz (float): Relative Z-coordinate (default: 0).

        For "RoadPosition":
            road_id (int): Road ID (default: 1).
            s (float): Length along the road (default: 0).
            t (float): Lateral offset from the road center (default: 0).

        For "RelativeRoadPosition":
            entity (str): Reference entity (default: "TargetEntity").
            ds (float): Length along the road (default: 0).
            dt (float): Lateral offset from the road center (default: 0).

        For "LanePosition":
            road_id (int): Road ID (default: 1).
            lane_id (int): Lane ID (default: 0).
            s (float): Length along the road (default: 0).
            offset (float): Offset from the lane center (default: 0).

        For "RelativeLanePosition":
            entity (str): Reference entity (default: "TargetEntity").
            ds (float): Length along the road (default: 0).
            d_lane (int): Lane ID difference (default: 0).
            offset (float): Offset from the lane center (default: 0).

        For "RoutePositionOfCurrentEntity":
            route_ref: Reference to the route.
            entity (str): Reference entity.

        For "RoutePositionInRoadCoordinates":
            route_ref: Reference to the route.
            s (float): S-coordinate (default: 0).
            t (float): T-coordinate (default: 0).

        For "RoutePositionInLaneCoordinates":
            route_ref: Reference to the route.
            s (float): S-coordinate (default: 0).
            lane_id (int): Lane ID (default: 0).
            offset (float): Offset from the lane center (default: 0).

        For "TrajectoryPosition":
            trajectory: Reference to the trajectory.
            s (float): S-coordinate (default: 0).
            t (float): T-coordinate (default: 0).

        For "GeoPosition":
            latitude (float): Latitude (default: 0).
            longitude (float): Longitude (default: 0).
            height (float): Height above the surface (default: 0).
    """

    positions = []

    for position_type in position_types:
        if position_type == "WorldPosition":
            x = kwargs.get("x", 3.75)
            y = kwargs.get("y", -4.625)
            z = kwargs.get("z", 0)
            h = kwargs.get("h", 0)
            p = kwargs.get("p", 0)
            r = kwargs.get("r", 0)
            position = xosc.WorldPosition(x, y, z, h, p, r)

        elif position_type == "RelativeWorldPosition":
            entity = kwargs.get("entity", "TargetEntity")
            dx = kwargs.get("dx", 0)
            dy = kwargs.get("dy", 0)
            dz = kwargs.get("dz", 0)
            position = xosc.RelativeWorldPosition(entity, dx, dy, dz)

        elif position_type == "RelativeObjectPosition":
            entity = kwargs.get("entity", "TargetEntity")
            dx = kwargs.get("dx", 0)
            dy = kwargs.get("dy", 0)
            dz = kwargs.get("dz", 0)
            position = xosc.RelativeObjectPosition(entity, dx, dy, dz)

        elif position_type == "RoadPosition":
            road_id = kwargs.get("road_id", 1)
            s = kwargs.get("s", 0)
            t = kwargs.get("t", 0)
            position = xosc.RoadPosition(s, t, road_id)

        elif position_type == "RelativeRoadPosition":
            entity = kwargs.get("entity", "TargetEntity")
            ds = kwargs.get("ds", 0)
            dt = kwargs.get("dt", 0)
            position = xosc.RelativeRoadPosition(ds, dt, entity)

        elif position_type == "LanePosition":
            road_id = kwargs.get("road_id", 1)
            lane_id = kwargs.get("lane_id", 0)
            s = kwargs.get("s", 0)
            offset = kwargs.get("offset", 0)
            position = xosc.LanePosition(s, offset, lane_id, road_id)

        elif position_type == "RelativeLanePosition":
            entity = kwargs.get("entity", "TargetEntity")
            ds = kwargs.get("ds", 0)
            d_lane = kwargs.get("d_lane", 0)
            offset = kwargs.get("offset", 0)
            position = xosc.RelativeLanePosition(d_lane, entity, offset, ds)

        elif position_type == "RoutePositionOfCurrentEntity":
            route_ref = kwargs.get("route_ref")
            entity = kwargs.get("entity")
            position = xosc.RoutePositionOfCurrentEntity(route_ref, entity)

        elif position_type == "RoutePositionInRoadCoordinates":
            route_ref = kwargs.get("route_ref")
            s = kwargs.get("s", 0)
            t = kwargs.get("t", 0)
            position = xosc.RoutePositionInRoadCoordinates(route_ref, s, t)

        elif position_type == "RoutePositionInLaneCoordinates":
            route_ref = kwargs.get("route_ref")
            s = kwargs.get("s", 0)
            lane_id = kwargs.get("lane_id", 0)
            offset = kwargs.get("offset", 0)
            position = xosc.RoutePositionInLaneCoordinates(route_ref, s, lane_id, offset)

        elif position_type == "TrajectoryPosition":
            trajectory = kwargs.get("trajectory")
            s = kwargs.get("s", 0)
            t = kwargs.get("t", 0)
            position = xosc.TrajectoryPosition(trajectory, s, t)

        elif position_type == "GeoPosition":
            latitude = kwargs.get("latitude", 0)
            longitude = kwargs.get("longitude", 0)
            height = kwargs.get("height", 0)
            position = xosc.GeoPosition(latitude, longitude, height)

        else:
            raise ValueError(f"Invalid position type: {position_type}")

        positions.append(position)

    for pos in positions:
        teleport_action = xosc.TeleportAction(pos)
        init.add_init_action(entity_name, teleport_action)

    speed_action = xosc.AbsoluteSpeedAction(kwargs.get("init_speed", 0), step_time)
    control_action = xosc.ActivateControllerAction(lateral=True, longitudinal=True)

    init.add_init_action(entity_name, speed_action)
    init.add_init_action(entity_name, control_action)


if __name__ == "__main__":
    init = xosc.Init()
    step_time = xosc.TransitionDynamics(xosc.DynamicsShapes.step, xosc.DynamicsDimension.rate, 0)

    # Example for initializing ego vehicle with WorldPosition
    initialize_position(init, "Ego", step_time, init_speed=10, position_types=["WorldPosition"], x=3.75, y=-4.625)

    # Example for initializing ego vehicle with combined WorldPosition and LanePosition
    initialize_position(init, "Ego", step_time, init_speed=10, position_types=["WorldPosition", "LanePosition"], x=3.75,
                        y=-4.625, road_id=1, lane_id=0, s=50, offset=0)

    # Example for initializing another object with RelativeWorldPosition
    initialize_position(init, "OtherObject", step_time, init_speed=5, position_types=["RelativeWorldPosition"],
                        entity="TargetEntity", dx=1, dy=2, dz=3)

    # Example for initializing another object with GeoPosition
    initialize_position(init, "OtherObject", step_time, init_speed=5, position_types=["GeoPosition"], latitude=52.5200,
                        longitude=13.4050, height=34)