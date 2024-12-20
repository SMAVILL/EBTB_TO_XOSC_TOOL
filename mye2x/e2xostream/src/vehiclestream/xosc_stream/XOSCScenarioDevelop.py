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
from e2xostream.src.acts_algo import ego_acts, obj_acts, shared_data
from e2xostream.src.vehiclestream.xosc_stream import ego_mapping_acts, obj_mapping_acts
from e2xostream.stk.scenariogeneration import esmini
from e2xostream.src.acts_algo.shared_data import event_counter_obj

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

        result = {}
        for key, value in states_analysis.items():
            actions = []
            actions.extend(action["Action"] for action in value.get("EgoActions", []))
            for obj_actions in value.get("ObjectActions", {}).values():
                actions.extend(action["Action"] for action in obj_actions)
            result[key] = actions

        shared_data.state_e_mapping = {}
        # Iterate through the data and create the dictionary
        for state_key in list(result.keys())[1:]:  # Start from second key to access previous state
            previous_state_key = str(int(state_key) - 1)
            current_e_names = result[state_key]
            previous_e_names = result.get(previous_state_key, [])
            e_name_from_previous = next((name for name in previous_e_names if name.startswith('E_')), None)
            if e_name_from_previous:
                shared_data.state_e_mapping[previous_state_key] = e_name_from_previous

        excluded_apis = {"Dri_PrepareVehicle", "Obj_Initialize"}

        # Initialize the main dictionary
        dri_obj_mapping = {}
        for state_key, e_names in result.items():
            # Filter out the excluded APIs
            filtered_apis = [api for api in e_names if api not in excluded_apis]

            # Create a sub-dictionary for each API that is not excluded
            sub_dict = {api: {"event_count": None, "action_count": None} for api in filtered_apis}

            if sub_dict:  # Only add to the dictionary if there are APIs for the state
                dri_obj_mapping[state_key] = sub_dict

        event_count = 1
        action_count = 1

        # Result dictionary
        shared_data.res = {}

        for state_key, actions in dri_obj_mapping.items():
            # Separate E_ values and non-E_ values
            e_values = [key for key in actions if key.startswith('E_')]
            non_e_values = [key for key in actions if not key.startswith('E_')]

            # Create a list to store the API dictionaries for the current state_key
            state_actions = []

            # Handle non-E_ values first
            if non_e_values:
                for key in non_e_values:
                    state_actions.append({
                        'api_name': key,
                        'event_count': event_count,  # Assign current event_count
                        'action_count': action_count
                    })
                    action_count += 1  # Increment action_count for each non-E_ API
                event_count += 1  # Increment event_count after all non-E_ APIs are processed

            # Handle E_ values
            for e_key in e_values:
                state_actions.append({
                    'api_name': e_key,
                    'event_count': event_count,  # Assign current event_count
                    'action_count': action_count
                })
                action_count += 1  # Increment action_count for each E_ API
                event_count += 1  # Increment event_count for each E_ API

            # Store the list of dictionaries in the result
            shared_data.res[state_key] = state_actions

        # # Output the updated dictionary
        # import pprint
        # pprint.pprint(shared_data.res)

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
                                                     states_analysis=self.states_analysis,paramlist_analysis=self.paramlist_analysis)



    def EgoManeuverGroup(self):
        # Ego Maneuver group
        ego_mnvgr = self.ego_maneuver_group_with_condition()
        self.act.add_maneuver_group(ego_mnvgr)

    def ObjManeuverGroup(self):
        from e2xostream.src.acts_algo.shared_data import event_counter_obj

        # Dictionary to store processed maneuvers for each object
        if len(self.obj_list) > 0:
            for i in self.obj_list:
                self.all_target_events=[]
                shared_data.event_counter_obj = 1
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