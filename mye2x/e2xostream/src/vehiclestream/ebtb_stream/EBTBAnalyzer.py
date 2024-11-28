import os
import xml.etree.ElementTree as ET
from collections import defaultdict
import sys

from e2xostream.config.api_constants.other_api_constants import E_TimeToCollision

MAIN_PATH = os.path.abspath(os.path.join(__file__, "..", "..", "..", ".."))
CURRENT_WORKING_FILE_DIRECTORY = os.path.abspath(os.path.join(__file__))
CURRENT_WORKING_DIRECTORY = os.path.abspath(os.path.join(__file__, ".."))

if MAIN_PATH not in sys.path:
    sys.path.append(MAIN_PATH)

dir_path = os.path.dirname(os.path.realpath(__file__))
for root, dirs, files in os.walk(dir_path):
    sys.path.append(root)

from e2xostream.config.api_constants import (api_methods_constants as ApiMethods,
                                             ego_api_constants as EgoAPI,
                                             obj_api_constants as ObjAPI,
                                             other_api_constants as OtherAPI)


def parse_element_details(element):
    """
    Parse element details
    Parameters
    ----------
    element

    Returns
    -------

    """
    # Check for specific elements to extract certain attributes directly
    if element.tag == OtherAPI.E_TimeToCollision:
        reference_time = next((child.attrib.get('value', '') for child in element if child.tag == 'ReferenceTime'), None)
        if reference_time is not None:
            return {'ReferenceTime': reference_time}

    elif element.tag == OtherAPI.E_ObjectDistanceLaneBased:
        value = next((child.attrib.get('value', '') for child in element if child.tag == 'Distance'), None)
        relational_operator = next((child.attrib.get('value', '') for child in element if child.tag == 'RelationalOperator'), None)
        reference_object = next((child.attrib.get('value', '') for child in element if child.tag == 'ReferenceObject'),None)
        object_id = next((child.attrib.get('value', '') for child in element if child.tag == 'ObjectID'),None)
        if value is not None:
            return {'Distance': value, 'RelationalOperator': relational_operator, 'ReferenceObject': reference_object,"ObjectID" : object_id}

    elif element.tag == OtherAPI.E_DistanceTimeBased:
        dist_value = next((child.attrib.get('value', '') for child in element if child.tag == 'Offset'), None)
        relational_operator = next((child.attrib.get('value', '') for child in element if child.tag == 'RelationalOperator'), None)
        refernce_object = next((child.attrib.get('value', '') for child in element if child.tag == 'ReferenceObject'),None)
        object = next((child.attrib.get('value', '') for child in element if child.tag == 'Object'),None)
        return {'Offset': dist_value, 'RelationalOperator': relational_operator, 'ReferenceObject': refernce_object,'Object':object}

    elif element.tag == OtherAPI.E_ADASState:
        adas_warning = next((child.attrib.get('value', '') for child in element if child.tag == 'CMSVisualWarning'), None)
        if adas_warning is not None:
            return {'CMSVisualWarning': adas_warning}

    elif element.tag == OtherAPI.E_SysVehicleVelocity:
        velocity = next((child.attrib.get('value', '') for child in element if child.tag == 'Velocity'), None)
        operator = next((child.attrib.get('value', '') for child in element if child.tag == 'Operator'), None)
        if velocity is not None:
            return {'Velocity': velocity, 'Operator': operator}
    # Default case for other elements
    return {param.tag: param.text for param in element}


def categorize_actions(entry, categories, keywords):
    """
    Categorize the actions
    Parameters
    ----------
    entry
    categories
    keywords

    Returns
    -------

    """
    if any(keyword in entry['tag'] for keyword in keywords['ego']):
        categories["ego"].append(entry)
    elif any(keyword in entry['tag'] for keyword in keywords['objects']) or entry['tag'].startswith('ObjP_'):
        object_id = None
        for param in entry['parameters']:
            if param['tag'] == 'ObjectID':
                object_id = param['value']
                break
        if object_id is None:
            object_id = next((param['value'] for param in entry['parameters'] if param['tag'] == 'ObjectId'), None)
        if object_id:
            categories["objects"][object_id].append(entry)


def process_entries(root, keywords, tag):
    """
    Process entries
    Parameters
    ----------
    root
    keywords
    tag

    Returns
    -------

    """
    info = defaultdict(lambda: {"ego": [], "objects": defaultdict(list), "info": {}, "OtherConditions": [],
                                "E_ObjectDistanceLaneBased": [], "E_TimeToCollision": [],"E_DistanceTimeBased":[],"E_ADASState": [],"E_SysVehicleVelocity":[]})

    events_data = {}

    for element in root.findall(f".//{tag}"):
        element_id = element.attrib.get('id', 'Default')
        info[element_id]["info"] = element.attrib

        for child in element:
            if child.tag in [OtherAPI.E_ObjectDistanceLaneBased, OtherAPI.E_TimeToCollision,OtherAPI.E_DistanceTimeBased, OtherAPI.E_ADASState,OtherAPI.E_SysVehicleVelocity]:
                specific_details = parse_element_details(child)
                events_data[child.tag] = specific_details
                # Directly add the specific details to the relevant category
                # if child.tag == 'E_ObjectDistanceLaneBased':
                #     # This assumes a structure where distance information is relevant to objects
                #     # You may need to adjust based on your actual data and intent
                #     #object_id = specific_details.get('ObjectID')# Hypothetical way to get an ObjectID, adjust as necessary
                #     print("element_id", element_id)
                #     print("info", info.items())
                # #     if object_id:
                # #         info[element_id]["objects"][object_id].append({"tag": child.tag, "details": specific_details})
                # #         E_ObjectDistanceLaneBased["E_ObjectDistanceLaneBased"] = specific_details
                # elif child.tag == 'E_TimeToCollision':
                #     #object_id = specific_details.get('ObjectID')
                #     # Hypothetical way to get an ObjectID, adjust as necessary
                #     print("element_id",element_id)
                #     print("info", info.items())
                #     # info[element_id]["ego"].append([{"tag": child.tag,"parameters":[ "value": specific_details})
                #     # E_TimeToCollision["E_TimeToCollision"] = specific_details
                #    # Assuming TimeToCollision is an ego action; adjust this assumption as necessary
                # elif child.tag == 'E_ADASState':
                #     #object_id = specific_details.get('ObjectID')
                #     # Hypothetical way to get an ObjectID, adjust as necessary
                #     print("element_id", element_id)
                #     print("info", info.items())

            # continue

            entry = {
                "tag": child.tag,
                "parameters": [{"tag": param.tag, "attributes": param.attrib,
                                "value": param.attrib.get('value', '')} for param in child]
            }
            categorize_actions(entry, info[element_id], keywords)

    return info, events_data


def construct_analysis_dict(info):
    """
    Construct analysis dict
    Parameters
    ----------
    info

    Returns
    -------

    """
    analysis_dict = {}
    for element_id, details in info.items():
        element_dict = {
            "Info": details["info"],
            "EgoActions": [],
            "ObjectActions": {},
            "OtherConditions": details["OtherConditions"],
            "E_ObjectDistanceLaneBasedActions": details[OtherAPI.E_ObjectDistanceLaneBased],
            "E_TimeToCollisionActions": details[OtherAPI.E_TimeToCollision],
            "E_ADASState": details[OtherAPI.E_ADASState],
            "E_DistanceTimeBased":details[OtherAPI.E_DistanceTimeBased],
            "E_SysVehicleVelocity": details[OtherAPI.E_SysVehicleVelocity]
        }

        for ego_action in details["ego"]:
            action_details = {
                "Action": ego_action['tag'],
                "Parameters": [{param['tag']: param['value'] for param in ego_action['parameters']}]
            }
            element_dict["EgoActions"].append(action_details)

        for object_id, actions in details["objects"].items():
            object_actions = [{
                "Action": action['tag'],
                "Parameters": [{param['tag']: param['value'] for param in action['parameters']}]
            } for action in actions]
            element_dict["ObjectActions"][object_id] = object_actions

        analysis_dict[element_id] = element_dict

    return analysis_dict


def main(file_path):
    print("read",file_path)
    """
    Main run
    Parameters
    ----------
    file_path

    Returns
    -------

    """
    keywords = {
        "ego": ["Dri_", "Sys_", "Ego_", "SysP_", "EnvP_", "TBA_", "E_","Ethernet_"],
        "objects": ["Obj_", "Object", "ObjP_"]
    }

    tree = ET.parse(file_path)
    root = tree.getroot()

    states_info, state_events = process_entries(root, keywords, "State")
    paramlist_info, param_events = process_entries(root, keywords, "ParameterList")


    states_analysis = construct_analysis_dict(states_info)
    paramlist_analysis = construct_analysis_dict(paramlist_info)

    return states_analysis, paramlist_analysis, state_events, param_events


if __name__ == "__main__":
    if len(sys.argv) > 1:
        file_path = sys.argv[1]
    else:
        print("Usage: python script.py <file_path>")
        sys.exit(1)

    states_analysis, paramlist_analysis = main(file_path)
    print("States Analysis:")
    for k, v in states_analysis.items():
        print(k, v.get("EgoActions"), v.get("ObjectActions"), v.get("OtherConditions"), v.get("E_ObjectDistanceLaneBasedActions"), v.get("E_TimeToCollisionActions"),v.get("E_DistanceTimeBased"), v.get("E_ADASState"),v.get("E_SysVehicleVelocity"))

    print("\nParameterList Analysis:")
    for k, v in paramlist_analysis.items():
        print(k, v.get("EgoActions"), v.get("ObjectActions"), v.get("OtherConditions"), v.get("E_ObjectDistanceLaneBasedActions"), v.get("E_TimeToCollisionActions"),v.get("E_ADASState") ,v.get("E_DistanceTimeBased"),v.get("E_SysVehicleVelocity"))
