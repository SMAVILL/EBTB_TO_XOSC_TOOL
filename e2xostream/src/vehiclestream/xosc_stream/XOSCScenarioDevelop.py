import os
import sys
import platform
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
from e2xostream.src.vehiclestream.xosc_stream import ego_mapping_acts, obj_mapping_acts
from e2xostream.stk.scenariogeneration import esmini

MAIN_PATH = os.path.abspath(os.path.join(__file__, "..", "..", "..", ".."))
CURRENT_WORKING_FILE_DIRECTORY = os.path.abspath(os.path.join(__file__))
CURRENT_WORKING_DIRECTORY = os.path.abspath(os.path.join(__file__, ".."))

if MAIN_PATH not in sys.path:
    sys.path.append(MAIN_PATH)

dir_path = os.path.dirname(os.path.realpath(__file__))
for root, dirs, files in os.walk(dir_path):
    sys.path.append(root)


class FuncScenario(ScenarioGenerator):
    def __init__(self, egoname, states_analysis, paramlist_analysis, state_events, param_events, esmini_path):
        super().__init__()

        self.states_analysis = states_analysis
        self.paramlist_analysis = paramlist_analysis
        self.state_events = state_events
        self.param_events = param_events
        self.esmini_path = esmini_path

        self.egoname = egoname
        self.open_scenario_version = 0

        # self.ego_speed, self.obj_speed, self.ego_transition_time, self.obj_transition_time = EBTB_API_data.get_ego_obj_speed_transition_time(
        #     states_analysis=self.states_analysis)

        self.VehicleControls = vehiclecontrol()
        self.Data_Controls = datacontrol()
        self.VehicleDefines = VehicleScenario()

        # Initialized Entities, Act, Init, step_time
        self.vehicle_entities = xosc.Entities()
        self.act = xosc.Act("act", self.VehicleDefines.start_trigger(time_delay=10))
        self.init = xosc.Init()
        self.step_time = xosc.TransitionDynamics(xosc.DynamicsShapes.step, xosc.DynamicsDimension.rate, 0)

        self.base_scenario = BS.BaseScenario()

        self.EgoManeuverActs = ego_mapping_acts.EgoScnearioActs(egoname, states_analysis, paramlist_analysis,
                                                                state_events, param_events, esmini_path)

        self.ObjManeuverActs = obj_mapping_acts.ObjScnearioActs(egoname, states_analysis, paramlist_analysis,
                                                                state_events, param_events, esmini_path)

        self.envp_landmark_offset = None
        self.obj_entities = None
        self.obj_list = None

        self.all_ego_events = []
        self.all_target_events = []

    def ego_maneuver_group_with_condition(self):
        """
        Create manuever group with condition
        Returns
        -------

        """
        ## create an event for the ego
        # start_trig = self.VehicleDefines.create_ego_event(value=5)
        # start_action = self.VehicleDefines.create_ego_action(speed=5)
        ## ego_start_event = self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action)
        # all_events.append(self.VehicleDefines.define_ego_action_event(start_trig=start_trig, start_action=start_action))

        self.EgoManeuverActs(all_ego_events=self.all_ego_events)
        # print("self.all_ego_events: ", self.all_ego_events)
        ego_mangr = self.base_scenario.ego_startevent_maneuver_group(total_events=self.all_ego_events,
                                                                     egoname=self.egoname)
        return ego_mangr

    def target_maneuver_group_with_condition(self, target_name="Obj1"):
        """
        Object manuver group create with condition
        Parameters
        ----------
        target_name

        Returns
        -------

        """
        ## create an event for the object
        # start_trig = self.VehicleDefines.create_target_event(value=5)
        # start_action = self.VehicleDefines.create_target_action(speed=0)
        # target_start_event = self.VehicleDefines.define_target_action_event(start_trig=start_trig,
        #                                                                     start_action=start_action)
        # self.all_target_events.append(target_start_event)

        # Object Acceleration
        self.ObjManeuverActs(self.all_target_events, target_name)

        target_mangr = self.base_scenario.target_startevent_maneuver_group(target_events=self.all_target_events,
                                                                           target_name=target_name)

        return target_mangr

    def VehicleCatlog_RoadNetwork(self):
        # Define the Vehicle Catalog and RoadNetwork
        catalog = self.VehicleDefines.create_catalogs(VehicleCatalog="")

        if self.esmini_path is None:
            road = self.VehicleDefines.create_road_network(
                XodrPath=EBTB_API_data.getroadnetwork(paramlist_analysis=self.paramlist_analysis),
                scenegraph="")
        else:
            if platform.system() == "Linux":
                road = self.VehicleDefines.create_road_network(
                    XodrPath="../../e2xostream/stk/simulator_tools/esmini_linux"
                             "/resources/xodr/straight_500m.xodr",
                    scenegraph="")
            elif platform.system() == "Windows":
                road = self.VehicleDefines.create_road_network(
                    XodrPath="../../e2xostream/stk/simulator_tools/esmini_win"
                             "/resources/xodr/straight_500m.xodr",
                    scenegraph="")
        paramdec = self.VehicleDefines.declare_parameters()

        return catalog, road, paramdec

    def VehicleEgoEntities(self):
        # Define ego and target entities

        if self.paramlist_analysis["Default"]["EgoActions"][0]["Action"] == EgoAPI.SysP_Vehicle:
            self.base_scenario.define_ego_entities(properties=default_properties.DEFAULT_EGO_PROPERTIES,
                                                   width=
                                                   self.paramlist_analysis["Default"]["EgoActions"][0]["Parameters"][0][
                                                       "VehicleWidth"],
                                                   length=
                                                   self.paramlist_analysis["Default"]["EgoActions"][0]["Parameters"][0][
                                                       "VehicleLength"],
                                                   egoname=self.egoname,
                                                   vehicle_entities=self.vehicle_entities)
        else:
            self.base_scenario.define_ego_entities(properties=default_properties.DEFAULT_EGO_PROPERTIES,
                                                   vehicle_entities=self.vehicle_entities)

    def VehicleObjEntities(self):
        # Define Target/Obj entities'

        self.obj_entities, self.obj_list = self.base_scenario.define_target_entities(
            properties=default_properties.DEFAULT_OBJ_PROPERTIES,
            vehicle_entities=self.vehicle_entities,
            paramlist_analysis=self.paramlist_analysis)

    def GlobalEnvironment(self):
        # Global action with environment setting
        self.VehicleDefines.global_action(init=self.init)

    def EgoInitilize(self):
        # Initialize Ego
        self.base_scenario.Ego_initialize(paramlist_analysis=self.paramlist_analysis,
                                          step_time=self.step_time,
                                          init=self.init)

    def ObjectInitialize(self):
        # Initialize Target/Objects
        if len(self.obj_list) > 0:
            for i in self.obj_list:
                self.base_scenario.Target_initialize(step_time=self.step_time, init=self.init, target_name=i,
                                                     states_analysis=self.states_analysis)

    def EgoManeuverGroup(self):
        # Ego Maneuver group
        ego_mnvgr = self.ego_maneuver_group_with_condition()
        self.act.add_maneuver_group(ego_mnvgr)

    def ObjManeuverGroup(self):

        # Dictionary to store processed maneuvers for each object
        if len(self.obj_list) > 0:
            for i in self.obj_list:
                self.all_target_events=[]
                target_mnvgr = self.target_maneuver_group_with_condition(target_name=i)
                self.act.add_maneuver_group(target_mnvgr)

    def scenario(self, **kwargs):
        """
        scenario create and define.
        Parameters
        ----------
        kwargs

        Returns
        -------

        """
        # Define the Vehicle Catalog and RoadNetwork
        catalog, road, paramdec = self.VehicleCatlog_RoadNetwork()

        # Define ego and target entities
        self.VehicleEgoEntities()
        self.VehicleObjEntities()

        # Global action with environment setting
        self.GlobalEnvironment()

        # Initialize Ego and Target
        self.EgoInitilize()
        self.ObjectInitialize()

        # Ego and Obj Maneuver group
        self.EgoManeuverGroup()
        self.ObjManeuverGroup()

        # Scenario Init, storyboard and Start
        storyboard = self.VehicleDefines.create_storyboard(init=self.init, act=self.act)

        aeb_sce = self.VehicleDefines.assemble_scenario(catalog=catalog, road=road, paramdec=paramdec,
                                                        entities=self.vehicle_entities, storyboard=storyboard,
                                                        open_scenario_version=self.open_scenario_version)
        return aeb_sce


def execute_sce_proc(xml_file_path, report_path, esmini_path):
    """
    execute scenario procedure
    Parameters
    ----------
    xml_file_path
    report_path

    Returns
    -------

    """
    states_analysis, paramlist_analysis, state_events, param_events = EBTBAnalyzer.main(xml_file_path)

    Func_Sce = FuncScenario(egoname="Ego", states_analysis=states_analysis, paramlist_analysis=paramlist_analysis,
                            state_events=state_events, param_events=param_events, esmini_path=esmini_path)

    Func_Sce.generate(report_path)

    if esmini_path is None:
        pass
    else:
        esmini(Func_Sce, esmini_path, generation_path=report_path)


if __name__ == "__main__":
    """
    AEB Scenario start
    """
    report_path = r"C:\Users\jadhavc\Downloads\MBRDI_WorkSpace\EBTB_XOSC_ToolDevelopment\EB2XOStreamline\NewProjectStructureE2X\e2xstreamline\Report"
    xml_file_path = r"C:\Users\HMOHANK\Desktop\Pangu_dev\e2xstreamline\EBTBs"
    esmini_path = r"C:\Users\jadhavc\Downloads\MBRDI_WorkSpace\EBTB_XOSC_ToolDevelopment\EB2XOStreamline\NewProjectStructureE2X\e2xstreamline\esmini",
    execute_sce_proc(xml_file_path, report_path, esmini_path)