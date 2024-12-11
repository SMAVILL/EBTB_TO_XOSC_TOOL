import os
import sys
import xml.etree.ElementTree as ET


from e2xostream.config import default_properties, global_parameters, settings
from e2xostream.stk.vehicledynamics.DataControl import DataControls as datacontrol
from e2xostream.config.api_constants import (api_methods_constants as ApiMethods,
                                             ego_api_constants as EgoAPI,
                                             obj_api_constants as ObjAPI,
                                             other_api_constants as OtherAPI)
from e2xostream.config.api_constants import api_methods_constants as AMC
from e2xostream import xlmrmaps,xodrmaps






def EBTB_anlyses_info(paramlist_analysis, states_analysis):
    """
    EBTB Anlyzed information display
    Returns
    -------

    """
    print("\nParameterList Analysis:")
    for k, v in paramlist_analysis.items():
        print(v.get("EgoActions")[0])
        print(v.get("ObjectActions"))

    print("\nStates Analysis:")
    for k, v in states_analysis.items():
        print("\n*********", k, "**********")
        # print(v.get("EgoActions"))
        # print(v.get("ObjectActions"))
        # print(v.get("OtherConditions"))
        # print(v.get("E_ObjectDistanceLaneBasedActions"))
        print(v)

def get_ego_obj_speed_transition_time(states_analysis):
    extracted_info = {}

    # Iterate over the states analysis dictionary
    for k, v in states_analysis.items():
        # Process EgoActions
        for action in v.get('EgoActions', []):
            if action.get('Action') == EgoAPI.Dri_SetLongitudinalSpeed:
                for param in action.get('Parameters', []):
                    target_speed = param.get('TargetSpeed', 'Not Available')
                    transition_time = param.get('TransitionTime', 0.0)  # Default to 0.0 if missing
                    extracted_info[EgoAPI.Dri_SetLongitudinalSpeed] = [
                        {'TargetSpeed': target_speed, 'TransitionTime': transition_time}]

        # Process ObjectActions
        for obj_id, actions in v.get('ObjectActions', {}).items():
            for action in actions:
                if action.get('Action') == ObjAPI.Obj_SetLongitudinalSpeed:
                    for param in action.get('Parameters', []):
                        target_speed = param.get('TargetSpeed', 'Not Available')
                        transition_time = param.get('TransitionTime', 0.0)  # Default to 0.0 if missing
                        # Append the object action information
                        obj_key = f'{obj_id}_Obj_SetLongitudinalSpeed'
                        if obj_key not in extracted_info:
                            extracted_info[obj_key] = []
                        extracted_info[obj_key].append(
                            {'TargetSpeed': target_speed, 'TransitionTime': transition_time})

    print("extras", extracted_info)
    # Ensure all values are lists for consistency
    for key, value in extracted_info.items():
        if isinstance(value, dict):
            extracted_info[key] = [value]

    # Extract ego vehicle information
    if EgoAPI.Dri_SetLongitudinalSpeed in extracted_info:
        speed_kmhr = float(extracted_info[EgoAPI.Dri_SetLongitudinalSpeed][0]['TargetSpeed'])
        speed_ms_float = datacontrol().kmhr_to_ms(speed_kmhr)
        ego_speed_ms = round(speed_ms_float, 2)
        ego_transition_time = extracted_info[EgoAPI.Dri_SetLongitudinalSpeed][0]['TransitionTime']
    else:
        ego_speed_ms = 0
        ego_transition_time = 0  # Default if no ego action found

    # Extract object vehicle information (Obj1 and Obj2)
    obj_speeds = []
    obj_transition_times = []

    for obj_id in ['Obj1', 'Obj2']:
        obj_key = f'{obj_id}_Obj_SetLongitudinalSpeed'
        if obj_key in extracted_info:
            for i in range(len(extracted_info[obj_key])):
                obj_speed_kmhr = float(extracted_info[obj_key][i]['TargetSpeed'])
                obj_speed_ms_float = round(datacontrol().kmhr_to_ms(obj_speed_kmhr), 2)
                obj_speed = round(obj_speed_ms_float,2)
                # obj_speeds.append(obj_speed_ms_float)  # Collect speeds
                obj_transition_time = extracted_info[obj_key][i]['TransitionTime']  # Collect transition times

    # Return the extracted values
    return ego_speed_ms, obj_speed, ego_transition_time, obj_transition_time

def get_ego_speed_transition_time(last_index_ego,states_analysis):
    extracted_info = {}

    # Iterate over the states analysis dictionary
    for k, v in states_analysis.items():
        # Process EgoActions
        for action in v.get('EgoActions', []):
            if action.get('Action') == EgoAPI.Dri_SetLongitudinalSpeed:
                for param in action.get('Parameters', []):
                    target_speed = param.get('TargetSpeed', 'Not Available')
                    transition_time = param.get('TransitionTime', 0.0)  # Default to 0.0 if missing
                    transition_acceleration = param.get('TransitionAcceleration',0)
                    transition_distance = param.get('TransitionDistance',0)

                    # Append to the list of extracted information for Ego's speed and transition time
                    if EgoAPI.Dri_SetLongitudinalSpeed not in extracted_info:
                        extracted_info[EgoAPI.Dri_SetLongitudinalSpeed] = []

                    extracted_info[EgoAPI.Dri_SetLongitudinalSpeed].append(
                        {'TargetSpeed': target_speed, 'TransitionTime': transition_time,'TransitionDistance':transition_distance,'TransitionAcceleration':transition_acceleration}
                    )

    # Default values for speed and transition time
    speed = 0
    transition_time = 0
    transition_distance = 0
    transition_acceleration = 0
    key = EgoAPI.Dri_SetLongitudinalSpeed

    # Initialize last_index_ego for this action if not already initialized
    if key not in last_index_ego:
        last_index_ego[key] = 0


    # If there are actions for Ego's longitudinal speed, access them sequentially
    if key in extracted_info:
        actions = extracted_info[key]

        # Ensure that the index does not exceed the length of the actions list
        if last_index_ego[key] < len(actions):
            current_action = actions[last_index_ego[key]]

            target_speed = current_action['TargetSpeed']
            if target_speed != 'Not Available':
                try:
                    speed_kmhr = float(target_speed)
                    speed_ms_float = datacontrol().kmhr_to_ms(speed_kmhr)
                    speed = round(speed_ms_float, 2)
                except ValueError:
                    print(f"Invalid speed value: {target_speed}")
                    speed = 0
            transition_time = current_action['TransitionTime']
            transition_distance = current_action['TransitionDistance']
            transition_acceleration = current_action['TransitionAcceleration']

            # Increment the index for the next call
            last_index_ego[key] += 1

        # Handle the case where all actions are processed
        else:
            pass
    return speed, transition_time,transition_distance,transition_acceleration




def get_obj_speed_transition_time(target_name,last_index,states_analysis):
    extracted_info = {}

    for k, v in states_analysis.items():
        for obj_id, actions in v.get('ObjectActions', {}).items():
            for action in actions:
                if action.get('Action') == ObjAPI.Obj_SetLongitudinalSpeed:

                    for param in action.get('Parameters', []):
                        target_speed = param.get('TargetSpeed', 'Not Available')
                        transition_time = param.get('TransitionTime', 0.0)  # Default to 0.0 if missing
                        transition_acceleration = param.get('TransitionAcceleration', 0.0)
                        transition_distance = param.get('TransitionDistance', 0.0)


                        obj_key = f'{obj_id}_Obj_SetLongitudinalSpeed'
                        if obj_key not in extracted_info:
                            extracted_info[obj_key] = []
                        extracted_info[obj_key].append(
                            {'TargetSpeed': target_speed, 'TransitionTime': transition_time, 'TransitionAcceleration':transition_acceleration,'TransitionDistance': transition_distance })

    key_to_access = f"{target_name}_Obj_SetLongitudinalSpeed"

    #global last_index  # Use global dictionary to retain state across calls

    if key_to_access not in last_index:
        last_index[key_to_access] = 0

        # Check if the key exists in extracted_info
    if key_to_access in extracted_info:
        # Get the list of values for the key
        extracted_value = extracted_info[key_to_access]

        # Ensure last_index does not exceed the list length
        if last_index[key_to_access] < len(extracted_value):
            # Access the current value based on last accessed index
            current_value = extracted_value[last_index[key_to_access]]
            speed_kmhr = float(current_value['TargetSpeed'])
            speed_ms_float = round(datacontrol().kmhr_to_ms(speed_kmhr), 2)
            speed = round(speed_ms_float, 2)

            transition_time = current_value['TransitionTime']
            transition_acceleration = current_value['TransitionAcceleration']
            transition_distance = current_value['TransitionDistance']


            # Update the last accessed index to move to the next item on the next call
            last_index[key_to_access] += 1
        else:
            # Reset or handle if all values have been processed
            speed = 0
            transition_time = 0
            transition_distance = 0
            transition_acceleration = 0
            last_index[key_to_access] = 0  # Optional reset if you want to loop back
    else:
        # Default values if key is not found
        speed = 0
        transition_time = 0
        transition_distance = 0
        transition_acceleration = 0

    return speed, transition_time, transition_distance, transition_acceleration


def xlmr_to_xodr_mapping(paramlist_analysis):
    """
    Mapping xlmr to xodr
    Parameters
    ----------
    paramlist_analysis

    Returns
    -------
    """
    # Loop over paramlist_analysis and return XlmrFile key value
    xlmr_file = "germany_hw_4lanes_rq31_10km_straight_v2.xlmr"
    for key, value in paramlist_analysis.items():
        ego_actions = value.get('EgoActions', [])
        for action in ego_actions:
            if action.get('Action', []) == OtherAPI.EnvP_RoadNetwork:
                parameters = action.get('Parameters', [])
                for param in parameters:
                    if AMC.XlmrFile in param:
                        xlmr_file = param[AMC.XlmrFile]
                        break
                else:
                    continue
                break

    # xlmr_file to xodr mapping
    xodr_path = global_parameters.XLMR[
        xlmr_file] if xlmr_file in global_parameters.XLMR else "dd878027-4265-43bd-9ff1-5fd9f688f1f7.xodr"

    return xodr_path

def xlmr_mapping_landmark(states_analysis,paramlist_analysis):

    global landmark_type
    for key, value in states_analysis.items():
            ego_actions = value.get('EgoActions', [])
            for action in ego_actions:
                if action.get('Action', []) == "E_Landmark":
                    landmark_type = (action.get('Parameters')[0]['Landmark'])
                    landmark_offset = (action.get('Parameters')[0]['LandmarkOffset'])


    current_directory = os.getcwd()

    subdirectory = "xlmrmaps"

    xlmr_paths = os.path.join(current_directory, subdirectory)

    for key, value in paramlist_analysis.items():
        ego_actions = value.get('EgoActions', [])
        for action in ego_actions:
            if action.get('Action', []) == OtherAPI.EnvP_RoadNetwork:
                parameters = action.get('Parameters', [])
                for param in parameters:
                    if AMC.XlmrFile in param:
                        xlmr_file = param[AMC.XlmrFile]
                        break
                else:
                    continue
                break
        xlmr_file_path = os.path.basename(xlmr_file)

    if os.path.exists(xlmr_paths) and os.path.isdir(xlmr_paths):
        # Iterate over files in the folder
        for filename in os.listdir(xlmr_paths):
            # Compare the filename with `xlmr_file_path`
            if filename == xlmr_file_path:
                file_path = os.path.join(xlmr_paths, filename)

                # Read and print the file contents
                tree = ET.parse(file_path)  # Replace 'your_file.xml' with your XML file path
                root = tree.getroot()

                ds_value_float = None
                for landmark in root.findall(".//landmark"):
                    if landmark.get('name') == landmark_type:
                        ds_value = landmark.get('ds',None)
                        ds_value_float = float(ds_value)

                if ds_value_float:
                    return ds_value_float, landmark_offset
                else:
                    return None,None



def extract_lenthoflane(paramlist_analysis):
    current_directory = os.getcwd()

    subdirectory = "xodrmaps"

    folder_path = os.path.join(current_directory, subdirectory)

    for key, value in paramlist_analysis.items():
        ego_actions = value.get('EgoActions', [])
        for action in ego_actions:
            if action.get('Action', []) == OtherAPI.EnvP_RoadNetwork:
                parameters = action.get('Parameters', [])
                for param in parameters:
                    if AMC.XlmrFile in param:
                        xlmr_file = param[AMC.XlmrFile]
                        break
                else:
                    continue
                break
        cleaned_file_path = os.path.basename(xlmr_file)

    """
    Converts the given .xlmr file name to .xodr, compares it with files in the folder,
    parses the .xodr file as XML, and extracts data from <lanes> and <laneSections> tags.
    For each lane, checks for the <left> tag and prints the lane id and width.

    Parameters
    ----------
    xlmr_file_name : str
        Name of the .xlmr file to be converted to .xodr and searched.
    folder_path : str
        Path to the folder where files are located.
    """
    try:
        xodr_map = cleaned_file_path.replace('.xlmr', '.xodr')
        if os.path.exists(folder_path) and os.path.isdir(folder_path):
            # Iterate over files in the folder
            for filename in os.listdir(folder_path):
                # Compare the filename with the generated xodr_map
                if filename == xodr_map:
                    file_path = os.path.join(folder_path, filename)

                    # Parse the .xodr file as XML
                    try:
                        tree = ET.parse(file_path)
                        root = tree.getroot()

                        # Navigate to <road> tag
                        road = root.find(".//road")
                        if road is not None:
                            # Extract the 'length' attribute
                            length_str = road.get('length')
                            if length_str:
                                try:
                                    road_length = float(length_str)  # Convert the string to a float
                                    return road_length  # Return the length value
                                except ValueError:
                                    print(f"Error converting length to float: {length_str}")
                                    return None
                            else:
                                print("No 'length' attribute found in the <road> tag.")
                                return None
                        else:
                            print("<road> tag not found in the file.")
                            return None

                    except ET.ParseError as e:
                        print(f"Error parsing the .xodr file: {e}")
                        return None
            print(f"File {xodr_map} not found in the folder {folder_path}.")
            return None
        else:
            print(f"The folder {folder_path} does not exist.")
            return None
    except Exception as e:
        print(f"Error reading the file: {e}")
        return None


def getroadnetwork(paramlist_analysis):
    xodr_path = xlmr_to_xodr_mapping(paramlist_analysis)
    return xodr_path


def get_obj_entities(paramlist_analysis):
    """
    Get the Object entities
    Returns
    -------

    """
    obj_details = {}
    obj_list = []
    if 'Default' in paramlist_analysis and 'ObjectActions' in paramlist_analysis['Default']:
        for obj_name, obj_data in paramlist_analysis['Default']['ObjectActions'].items():
            if obj_data and obj_data[0]['Parameters']:
                obj_parameters = obj_data[0]['Parameters'][0]
                obj_id = obj_parameters.get('ObjectID')
                asset_id = (obj_parameters.get('AssetID').split('/')[0] if obj_parameters.get('AssetID') else None)


                if asset_id is not None:
                    asset_id = global_parameters.VEHICLE_CATEGORIES.get(asset_id.lower(), 'car')
                else:
                     asset_id = "car"



                if asset_id == "bicycle":
                    vehicle_name = global_parameters.VEHICLE_NAME.get(asset_id.lower(),'bicycle')
                    width = default_properties.DEFAULT_SIZE_PROPERTIES.get('bicycle')['width']
                    length = default_properties.DEFAULT_SIZE_PROPERTIES.get('bicycle')['length']
                    height = default_properties.DEFAULT_SIZE_PROPERTIES.get('bicycle')['height']
                if asset_id == "van" :
                    vehicle_name = global_parameters.VEHICLE_NAME.get(asset_id.lower(),'van')
                    width = default_properties.DEFAULT_SIZE_PROPERTIES.get('van')['width']
                    length = default_properties.DEFAULT_SIZE_PROPERTIES.get('van')['length']
                    height = default_properties.DEFAULT_SIZE_PROPERTIES.get('van')['height']
                if asset_id == "truck":
                    vehicle_name = global_parameters.VEHICLE_NAME.get(asset_id.lower(),'truck')
                    width = default_properties.DEFAULT_SIZE_PROPERTIES.get('truck')['width']
                    length = default_properties.DEFAULT_SIZE_PROPERTIES.get('truck')['length']
                    height = default_properties.DEFAULT_SIZE_PROPERTIES.get('truck')['height']
                if asset_id == "trailer":
                    vehicle_name = global_parameters.VEHICLE_NAME.get(asset_id.lower(),'trailer')
                    width = default_properties.DEFAULT_SIZE_PROPERTIES.get('trailer')['width']
                    length = default_properties.DEFAULT_SIZE_PROPERTIES.get('trailer')['length']
                    height = default_properties.DEFAULT_SIZE_PROPERTIES.get('trailer')['height']
                if asset_id == "semitrailer":
                    vehicle_name = global_parameters.VEHICLE_NAME.get(asset_id.lower(),'semitrailer')
                    width = default_properties.DEFAULT_SIZE_PROPERTIES.get('semitrailer')['width']
                    length = default_properties.DEFAULT_SIZE_PROPERTIES.get('semitrailer')['length']
                    height = default_properties.DEFAULT_SIZE_PROPERTIES.get('semitrailer')['height']
                if asset_id == "bus":
                    vehicle_name = global_parameters.VEHICLE_NAME.get(asset_id.lower(),'bus')
                    width = default_properties.DEFAULT_SIZE_PROPERTIES.get('bus')['width']
                    length = default_properties.DEFAULT_SIZE_PROPERTIES.get('bus')['length']
                    height = default_properties.DEFAULT_SIZE_PROPERTIES.get('bus')['height']
                if asset_id == "motorbike":
                    vehicle_name = global_parameters.VEHICLE_NAME.get(asset_id.lower(),'twowheelers')
                    width = default_properties.DEFAULT_SIZE_PROPERTIES.get('twowheelers')['width']
                    length = default_properties.DEFAULT_SIZE_PROPERTIES.get('twowheelers')['length']
                    height = default_properties.DEFAULT_SIZE_PROPERTIES.get('twowheelers')['height']
                if asset_id == "train":
                    vehicle_name = global_parameters.VEHICLE_NAME.get(asset_id.lower(),'train')
                    width = default_properties.DEFAULT_SIZE_PROPERTIES.get('train')['width']
                    length = default_properties.DEFAULT_SIZE_PROPERTIES.get('train')['length']
                    height = default_properties.DEFAULT_SIZE_PROPERTIES.get('train')['height']
                if asset_id == "tram":
                    vehicle_name = global_parameters.VEHICLE_NAME.get(asset_id.lower(),'tram')
                    width = default_properties.DEFAULT_SIZE_PROPERTIES.get('tram')['width']
                    length = default_properties.DEFAULT_SIZE_PROPERTIES.get('tram')['length']
                    height = default_properties.DEFAULT_SIZE_PROPERTIES.get('tram')['height']
                if asset_id == "pedestrian":
                    vehicle_name = global_parameters.VEHICLE_NAME.get(asset_id.lower(),'human')
                    width = default_properties.DEFAULT_SIZE_PROPERTIES.get('human')['width']
                    length = default_properties.DEFAULT_SIZE_PROPERTIES.get('human')['length']
                    height = default_properties.DEFAULT_SIZE_PROPERTIES.get('human')['height']
                if asset_id == "car":

                    vehicle_name = global_parameters.VEHICLE_NAME.get(asset_id.lower(),'car')
                    width = default_properties.DEFAULT_SIZE_PROPERTIES.get('car')['width']
                    length = default_properties.DEFAULT_SIZE_PROPERTIES.get('car')['length']
                    height = default_properties.DEFAULT_SIZE_PROPERTIES.get('car')['height']


                if obj_id and width and length and height:
                    obj_details[obj_id] = [width, length, height, str(asset_id), vehicle_name]

                else:
                    obj_details[obj_id] = [1.8, 4.5, 1.5, str(asset_id), vehicle_name]


                obj_list.append(obj_id)


    return obj_details, obj_list


def get_landmark_offset_obj(states_analysis):
    """
    Get the landmark offset for Obj
    Returns
    -------

    """
    landmark_offsets = {}
    for entity_id, entity_data in states_analysis.items():
        if 'ObjectActions' in entity_data:
            object_actions = entity_data['ObjectActions']
            for obj_name, obj_data in object_actions.items():
                for obj_action in obj_data:
                    if 'Parameters' in obj_action:
                        parameters = obj_action['Parameters']
                        for parameter in parameters:
                            landmark_offset = parameter.get(AMC.LandmarkOffset)
                            if landmark_offset is not None:
                                landmark_offsets[obj_name] = landmark_offset
    # print("Landmark offset for obj", landmark_offsets)
    return landmark_offsets


def get_Obj_set_lateral_relative(states_analysis):
    result = []

    for key, value in states_analysis.items():
        if 'ObjectActions' in value:
            for obj_key, actions in value['ObjectActions'].items():
                for action in actions:
                    if action['Action'] == ObjAPI.Obj_SetLateralRelativePosition:
                        result.append(action)

    return result[-1]["Parameters"]


def get_vehicle_braking_info(TBA_eval_key, states_analysis):
    """
    Get the vehicle braking info
    Parameters
    ----------
    TBA_eval_key

    Returns
    -------

    """
    driver_set_brake = {"Brake": 0}
    # TBA_eval_key = -1

    for k, v in states_analysis.items():
        if k != TBA_eval_key:
            for egoaction in v["EgoActions"]:
                try:
                    if egoaction["Action"] == EgoAPI.Dri_SetBrakePedal:
                        driver_set_brake["Brake"] = int(egoaction["Parameters"][0]["Position"])

                        return driver_set_brake
                    else:
                        pass
                except:
                    pass
        else:
            driver_set_brake["Brake"] = -1
    return driver_set_brake


def get_vehicle_throttle_info(TBA_eval_key, states_analysis):
    """
    Get the vehicle throttle info
    Parameters
    ----------
    TBA_eval_key

    Returns
    -------

    """
    driver_set_acc = {"Throttle": 0}

    for k, v in states_analysis.items():
        if k != TBA_eval_key:
            for egoaction in v["EgoActions"]:
                try:
                    if egoaction["Action"] == EgoAPI.Dri_SetAccelerationPedal:
                        driver_set_acc["Throttle"] = int(egoaction["Parameters"][0]["Position"])
                        return driver_set_acc
                    else:
                        pass
                except:
                    driver_set_acc["Throttle"] = 5
                    return driver_set_acc
        else:
            driver_set_acc["Throttle"] = -1

    return driver_set_acc


def get_TBA_key_value(states_analysis):
    """
    Get the TBA key value
    Returns
    -------

    """
    TBA_eval_key = -1
    for k, v in states_analysis.items():
        for egoaction in v["EgoActions"]:
            try:
                if egoaction["Action"] == OtherAPI.TBA_WriteEvaluationEvent:
                    if egoaction["Parameters"][0]['Type'] == "Stop":
                        TBA_eval_key = k
                        return TBA_eval_key
                else:
                    pass
                    # print("TBA_WriteEvaluationEvent not found.")
            except:
                pass
    return TBA_eval_key


def get_landmark_offset_ego(paramlist_analysis):
    """
    Get the Landmark offset of Ego
    Returns
    -------

    """
    envp_landmark_offset = 10.125
    ego_actions = paramlist_analysis['Default']['EgoActions']
    for ego_action in ego_actions:
        parameters = ego_action['Parameters']
        for parameter in parameters:
            envp_landmark_offset = parameter.get(AMC.LandmarkOffset)
    return envp_landmark_offset

def get_lane_selection_ego(paramlist_analysis):
    """
    Get the Landmark offset of Ego
    Returns
    -------

    """
    ego_actions = paramlist_analysis['Default']['EgoActions']
    for ego_action in ego_actions:
        parameters = ego_action['Parameters']
        for parameter in parameters:
            envp_lane_selection = parameter.get(AMC.LaneSelection)
    return envp_lane_selection
def get_lane_selection_object(states_analysis, target_name):
    extracted_info = {}
    for k, v in states_analysis.items():
        for obj_id, actions in v.get('ObjectActions', {}).items():
            for action in actions:
                if action.get('Action') == ObjAPI.Obj_Initialize:
                    for param in action.get('Parameters', []):
                        lane_selection = param['LaneSelection']

                        # Append the object action information
                        obj_key = f'{obj_id}_Obj_Initialize'
                        if obj_key not in extracted_info:
                            extracted_info[obj_key] = []
                        extracted_info[obj_key].append(
                            {'LaneSelection': lane_selection})

    key_to_access = f"{target_name}_Obj_Initialize"
    if key_to_access in extracted_info:
        extracted_value = extracted_info[key_to_access]
        lane_selection = extracted_value[0]['LaneSelection']
    return lane_selection


def get_landmark_offset(states_analysis, target_name):
    extracted_info = {}
    for k, v in states_analysis.items():
        for obj_id, actions in v.get('ObjectActions', {}).items():
            for action in actions:
                if action.get('Action') == ObjAPI.Obj_Initialize:
                    for param in action.get('Parameters', []):
                        landmark_offset= param.get('LandmarkOffset', 0)

                        # Append the object action information
                        obj_key = f'{obj_id}_Obj_Initialize'
                        if obj_key not in extracted_info:
                            extracted_info[obj_key] = []
                        extracted_info[obj_key].append(
                            {'LandmarkOffset':landmark_offset })


    key_to_access = f"{target_name}_Obj_Initialize"
    if key_to_access in extracted_info:
         extracted_value = extracted_info[key_to_access]
         landmark_offset =  float(extracted_value[0]['LandmarkOffset'])
    if landmark_offset:
        return landmark_offset
    else:
        return 0

def get_obj_intialise(states_analysis,target_name):
    extracted_info = {}
    for k, v in states_analysis.items():
        for obj_id, actions in v.get('ObjectActions', {}).items():
            for action in actions:
                if action.get('Action') == ObjAPI.Obj_Initialize:
                    for param in action.get('Parameters', []):
                        Longitudinal = param.get('DistanceLongitudinal',0)
                        Lateral = param.get('DistanceLateral',0)

                        obj_key = f'{obj_id}_Obj_Initialize'
                        if obj_key not in extracted_info:
                            extracted_info[obj_key] = []
                        extracted_info[obj_key].append(
                            {'DistanceLongitudinal': Longitudinal,'DistanceLateral':Lateral})

    key_to_access = f"{target_name}_Obj_Initialize"
    if key_to_access in extracted_info:
        extracted_value = extracted_info[key_to_access]
        Longitudinal = extracted_value[0]['DistanceLongitudinal']
        Lateral = extracted_value[0]['DistanceLateral']
    return Longitudinal,Lateral

def get_obj_initialise_ver1(states_analysis,target_name):
    extracted_info = {}
    for k, v in states_analysis.items():
        for obj_id, actions in v.get('ObjectActions', {}).items():
            for action in actions:
                if action.get('Action') == ObjAPI.Obj_Initialize:
                    for param in action.get('Parameters', []):
                        landmark_offset = param.get('LaneOffset', 0)

                        # Append the object action information
                        obj_key = f'{obj_id}_Obj_Initialize'
                        if obj_key not in extracted_info:
                            extracted_info[obj_key] = []
                        extracted_info[obj_key].append(
                            {'LaneOffset': landmark_offset})

    key_to_access = f"{target_name}_Obj_Initialize"
    if key_to_access in extracted_info:
        extracted_value = extracted_info[key_to_access]
        lane_offset = float(extracted_value[0]['LaneOffset'])
    if lane_offset:
        return lane_offset
    else:
        return 0

def get_ego_initialise(paramlist_analysis):
    ego_actions = paramlist_analysis['Default']['EgoActions']
    for ego_action in ego_actions:
        parameters = ego_action['Parameters']
        for parameter in parameters:
            lane_offset = parameter.get('LaneOffset',0)
    return lane_offset

def ego_landmark_start_init(paramlist_analysis):
    global landmark_type

    ego_actions = paramlist_analysis['Default']['EgoActions']
    for ego_action in ego_actions:
        parameters = ego_action['Parameters']
        for parameter in parameters:
            landmark_start = parameter.get('LandmarkStart')

    current_directory = os.getcwd()

    subdirectory = "xlmrmaps"

    xlmr_paths = os.path.join(current_directory, subdirectory)

    for key, value in paramlist_analysis.items():
        ego_actions = value.get('EgoActions', [])
        for action in ego_actions:
            if action.get('Action', []) == OtherAPI.EnvP_RoadNetwork:
                parameters = action.get('Parameters', [])
                for param in parameters:
                    if AMC.XlmrFile in param:
                        xlmr_file = param[AMC.XlmrFile]
                        break
                else:
                    continue
                break
        xlmr_file_path = os.path.basename(xlmr_file)

    if os.path.exists(xlmr_paths) and os.path.isdir(xlmr_paths):
        # Iterate over files in the folder
        for filename in os.listdir(xlmr_paths):
            # Compare the filename with `xlmr_file_path`
            if filename == xlmr_file_path:
                file_path = os.path.join(xlmr_paths, filename)

                # Read and print the file contents
                tree = ET.parse(file_path)  # Replace 'your_file.xml' with your XML file path
                root = tree.getroot()

                ds_value_float = 0
                for landmark in root.findall(".//landmark"):
                    if landmark.get('name') == landmark_start:
                        ds_value = landmark.get('ds')
                        ds_value_float = float(ds_value)

    return ds_value_float

def obj_landmark_start_init(states_analysis,target_name,paramlist_analysis):
    extracted_info = {}
    for k, v in states_analysis.items():
        for obj_id, actions in v.get('ObjectActions', {}).items():
            for action in actions:
                if action.get('Action') == ObjAPI.Obj_Initialize:
                    for param in action.get('Parameters', []):
                        landmark_start = param.get('LandmarkStart', 'Not Available')

                        # Append the object action information
                        obj_key = f'{obj_id}_Obj_Initialize'
                        if obj_key not in extracted_info:
                            extracted_info[obj_key] = []
                        extracted_info[obj_key].append(
                            {'LandmarkStart': landmark_start})

    key_to_access = f"{target_name}_Obj_Initialize"
    if key_to_access in extracted_info:
        extracted_value = extracted_info[key_to_access]
        landmark_start = (extracted_value[0]['LandmarkStart'])


    current_directory = os.getcwd()

    subdirectory = "xlmrmaps"

    xlmr_paths = os.path.join(current_directory, subdirectory)

    for key, value in paramlist_analysis.items():
        ego_actions = value.get('EgoActions', [])
        for action in ego_actions:
            if action.get('Action', []) == OtherAPI.EnvP_RoadNetwork:
                parameters = action.get('Parameters', [])
                for param in parameters:
                    if AMC.XlmrFile in param:
                        xlmr_file = param[AMC.XlmrFile]
                        break
                else:
                    continue
                break
        xlmr_file_path = os.path.basename(xlmr_file)

    if os.path.exists(xlmr_paths) and os.path.isdir(xlmr_paths):
        # Iterate over files in the folder
        for filename in os.listdir(xlmr_paths):
            # Compare the filename with `xlmr_file_path`
            if filename == xlmr_file_path:
                file_path = os.path.join(xlmr_paths, filename)

                # Read and print the file contents
                tree = ET.parse(file_path)  # Replace 'your_file.xml' with your XML file path
                root = tree.getroot()

                ds_value_float = None
                for landmark in root.findall(".//landmark"):
                    if landmark.get('name') == landmark_start:
                        ds_value = landmark.get('ds')
                        ds_value_float = float(ds_value)

    if ds_value_float:
        return ds_value_float
    else:
        return 0


def ego_brake(ego_brake_index,states_analysis):
    extracted_info_brake = {}

    # Iterate over the states analysis dictionary
    for k, v in states_analysis.items():
        for action in v.get('EgoActions', []):
            if action.get('Action') == EgoAPI.Dri_SetBrakePedal:
                for param in action.get('Parameters', []):
                    value_break = param.get('Position','Not Available')

                    # Append to the list of extracted information for Ego's speed and transition time
                    if EgoAPI.Dri_SetBrakePedal not in extracted_info_brake:
                        extracted_info_brake[EgoAPI.Dri_SetBrakePedal] = []

                    extracted_info_brake[EgoAPI.Dri_SetBrakePedal].append(
                        {'Position': value_break}
                    )

    # Default values for speed and transition time
    value_break = 0
    key = EgoAPI.Dri_SetBrakePedal

    # Initialize last_index_ego for this action if not already initialized
    if key not in ego_brake_index:
        ego_brake_index[key] = 0


    # If there are actions for Ego's longitudinal speed, access them sequentially
    if key in extracted_info_brake:
        actions = extracted_info_brake[key]

        # Ensure that the index does not exceed the length of the actions list
        if ego_brake_index[key] < len(actions):
            current_action = actions[ego_brake_index[key]]

            value_break = current_action['Position']
            if value_break != 'Not Available':
                try:
                    value_break = value_break
                except ValueError:
                    print(f"Invalid speed value: {value_break}")
                    value_break = 0


            # Increment the index for the next call
            ego_brake_index[key] += 1

        # Handle the case where all actions are processed
        else:
            pass
    return value_break

def ego_throttle(ego_throttle_index,states_analysis):
    extracted_info_throttle = {}

    # Iterate over the states analysis dictionary
    for k, v in states_analysis.items():
        for action in v.get('EgoActions', []):
            if action.get('Action') == EgoAPI.Dri_SetAccelerationPedal:
                for param in action.get('Parameters', []):
                    value_throttle = param.get('Position','Not Available')

                    # Append to the list of extracted information for Ego's speed and transition time
                    if EgoAPI.Dri_SetAccelerationPedal not in extracted_info_throttle:
                        extracted_info_throttle[EgoAPI.Dri_SetAccelerationPedal] = []

                    extracted_info_throttle[EgoAPI.Dri_SetAccelerationPedal].append(
                        {'Position': value_throttle}
                    )

    # Default values for speed and transition time
    value_throttle = 0
    key = EgoAPI.Dri_SetAccelerationPedal

    # Initialize last_index_ego for this action if not already initialized
    if key not in ego_throttle_index:
        ego_throttle_index[key] = 0


    # If there are actions for Ego's longitudinal speed, access them sequentially
    if key in extracted_info_throttle:
        actions = extracted_info_throttle[key]

        # Ensure that the index does not exceed the length of the actions list
        if ego_throttle_index[key] < len(actions):
            current_action = actions[ego_throttle_index[key]]

            value_throttle = current_action['Position']
            if value_throttle != 'Not Available':
                try:
                    value_throttle = value_throttle
                except ValueError:
                    print(f"Invalid speed value: {value_throttle}")
                    value_throttle = 0


            # Increment the index for the next call
            ego_throttle_index[key] += 1

        # Handle the case where all actions are processed
        else:
            pass
    return value_throttle

def ego_switchgear(ego_gear_index,states_analysis):
    extracted_info_gear = {}

    # Iterate over the states analysis dictionary
    for k, v in states_analysis.items():
        for action in v.get('EgoActions', []):
            if action.get('Action') == EgoAPI.Dri_SwitchGear:

                for param in action.get('Parameters', []):
                    value_gear = param.get('Position', 'Not Available')

                    # Append to the list of extracted information for Ego's speed and transition time
                    if EgoAPI.Dri_SwitchGear not in extracted_info_gear:
                        extracted_info_gear[EgoAPI.Dri_SwitchGear] = []

                    extracted_info_gear[EgoAPI.Dri_SwitchGear].append(
                        {'Position': value_gear}
                    )

    # Default values for speed and transition time
    value_gear = 0
    key = EgoAPI.Dri_SwitchGear

    # Initialize last_index_ego for this action if not already initialized
    if key not in ego_gear_index:
        ego_gear_index[key] = 0

    # If there are actions for Ego's longitudinal speed, access them sequentially
    if key in extracted_info_gear:
        actions = extracted_info_gear[key]

        # Ensure that the index does not exceed the length of the actions list
        if ego_gear_index[key] < len(actions):
            current_action = actions[ego_gear_index[key]]

            value_gear = current_action['Position']
            if value_gear != 'Not Available':
                try:
                    value_gear = value_gear
                except ValueError:
                    print(f"Invalid speed value: {value_gear}")
                    value_gear = 0

            # Increment the index for the next call
            ego_gear_index[key] += 1

        # Handle the case where all actions are processed
        else:
            pass
    return value_gear

def ego_parkingbrake(ego_pb_index,states_analysis):
    extracted_info_pb = {}

    # Iterate over the states analysis dictionary
    for k, v in states_analysis.items():
        for action in v.get('EgoActions', []):
            if action.get('Action') == EgoAPI.Dri_SetParkingBrake:

                for param in action.get('Parameters', []):
                    value_pb = param.get('State', 'Not Available')

                    # Append to the list of extracted information for Ego's speed and transition time
                    if EgoAPI.Dri_SetParkingBrake not in extracted_info_pb:
                        extracted_info_pb[EgoAPI.Dri_SetParkingBrake] = []

                    extracted_info_pb[EgoAPI.Dri_SetParkingBrake].append(
                        {'State': value_pb}
                    )

    # Default values for speed and transition time
    value_pb = 0
    key = EgoAPI.Dri_SetParkingBrake

    # Initialize last_index_ego for this action if not already initialized
    if key not in ego_pb_index:
        ego_pb_index[key] = 0

    # If there are actions for Ego's longitudinal speed, access them sequentially
    if key in extracted_info_pb:
        actions = extracted_info_pb[key]

        # Ensure that the index does not exceed the length of the actions list
        if ego_pb_index[key] < len(actions):
            current_action = actions[ego_pb_index[key]]

            value_pb = current_action['State']
            if value_pb != 'Not Available':
                try:
                    value_pb = value_pb
                except ValueError:
                    print(f"Invalid speed value: {value_pb}")
                    value_pb = 0

            # Increment the index for the next call
            ego_pb_index[key] += 1

        # Handle the case where all actions are processed
        else:
            pass
    return value_pb


def ego_steeringwheel_angle(ego_sw_index,states_analysis):
    extracted_info_sw = {}

    # Iterate over the states analysis dictionary
    for k, v in states_analysis.items():
        for action in v.get('EgoActions', []):
            if action.get('Action') == EgoAPI.Dri_SetSteeringWheelAngle:

                for param in action.get('Parameters', []):
                    value_sw = param.get('Angle', 'Not Available')

                    # Append to the list of extracted information for Ego's speed and transition time
                    if EgoAPI.Dri_SetSteeringWheelAngle not in extracted_info_sw:
                        extracted_info_sw[EgoAPI.Dri_SetSteeringWheelAngle] = []

                    extracted_info_sw[EgoAPI.Dri_SetSteeringWheelAngle].append(
                        {'Angle': value_sw}
                    )

    # Default values for speed and transition time
    value_sw = 0
    key = EgoAPI.Dri_SetSteeringWheelAngle

    # Initialize last_index_ego for this action if not already initialized
    if key not in ego_sw_index:
        ego_sw_index[key] = 0

    # If there are actions for Ego's longitudinal speed, access them sequentially
    if key in extracted_info_sw:
        actions = extracted_info_sw[key]

        # Ensure that the index does not exceed the length of the actions list
        if ego_sw_index[key] < len(actions):
            current_action = actions[ego_sw_index[key]]

            value_sw = current_action['Angle']
            if value_sw != 'Not Available':
                try:
                    value_sw = value_sw
                except ValueError:
                    print(f"Invalid speed value: {value_sw}")
                    value_sw = 0

            # Increment the index for the next call
            ego_sw_index[key] += 1

        # Handle the case where all actions are processed
        else:
            pass
    return value_sw

def ego_set_lateral_ref(ego_lateralref_index,states_analysis):
    extracted_info_setlateralref = {}

    # Iterate over the states analysis dictionary
    for k, v in states_analysis.items():
        for action in v.get('EgoActions', []):
            if action.get('Action') == EgoAPI.Dri_SetLateralReference:

                for param in action.get('Parameters', []):
                    lane_value_str = param.get('LaneSelection', 'Not Available')

                    # Append to the list of extracted information for Ego's speed and transition time
                    if EgoAPI.Dri_SetLateralReference not in extracted_info_setlateralref:
                        extracted_info_setlateralref[EgoAPI.Dri_SetLateralReference] = []

                    extracted_info_setlateralref[EgoAPI.Dri_SetLateralReference].append(
                        {'LaneSelection': lane_value_str}
                    )

    # Default values for speed and transition time
    lane_value_str = 0
    key = EgoAPI.Dri_SetLateralReference

    # Initialize last_index_ego for this action if not already initialized
    if key not in ego_lateralref_index:
        ego_lateralref_index[key] = 0

    # If there are actions for Ego's longitudinal speed, access them sequentially
    if key in extracted_info_setlateralref:
        actions = extracted_info_setlateralref[key]

        # Ensure that the index does not exceed the length of the actions list
        if ego_lateralref_index[key] < len(actions):
            current_action = actions[ego_lateralref_index[key]]

            lane_value_str = current_action['LaneSelection']
            if lane_value_str != 'Not Available':
                try:
                    lane_value_str = lane_value_str
                except ValueError:
                    print(f"Invalid speed value: {lane_value_str}")
                    lane_value_str = 0

            # Increment the index for the next call
            ego_lateralref_index[key] += 1

        # Handle the case where all actions are processed
        else:
            pass
    return lane_value_str


def ego_set_lateral_disp(ego_lateraldisp_index,states_analysis):
    extracted_info_setlateraldisp = {}

    # Iterate over the states analysis dictionary
    for k, v in states_analysis.items():
        for action in v.get('EgoActions', []):
            if action.get('Action') == EgoAPI.Dri_SetLateralDisplacement:

                for param in action.get('Parameters', []):
                    dispvalue = param.get('TargetDisplacement', 'Not Available')

                    # Append to the list of extracted information for Ego's speed and transition time
                    if EgoAPI.Dri_SetLateralDisplacement not in extracted_info_setlateraldisp:
                        extracted_info_setlateraldisp[EgoAPI.Dri_SetLateralDisplacement] = []

                    extracted_info_setlateraldisp[EgoAPI.Dri_SetLateralDisplacement].append(
                        {'TargetDisplacement': dispvalue}
                    )


    dispvalue = 0
    key = EgoAPI.Dri_SetLateralDisplacement

    # Initialize last_index_ego for this action if not already initialized
    if key not in ego_lateraldisp_index:
        ego_lateraldisp_index[key] = 0

    # If there are actions for Ego's longitudinal speed, access them sequentially
    if key in extracted_info_setlateraldisp:
        actions = extracted_info_setlateraldisp[key]

        # Ensure that the index does not exceed the length of the actions list
        if ego_lateraldisp_index[key] < len(actions):
            current_action = actions[ego_lateraldisp_index[key]]

            dispvalue = current_action['TargetDisplacement']
            if dispvalue != 'Not Available':
                try:
                    dispvalue = dispvalue
                except ValueError:
                    print(f"Invalid speed value: {dispvalue}")
                    lane_value_str = 0

            # Increment the index for the next call
            ego_lateraldisp_index[key] += 1

        # Handle the case where all actions are processed
        else:
            pass
    return dispvalue