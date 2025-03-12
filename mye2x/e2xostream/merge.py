import os

from lxml import etree
from collections import defaultdict


def merge_events(events):
    """
    For merging the events with same trigger so that multiple actions can be triggered using a single trigger
    """
    merged_events = {}
    for event in events:
        event_name = event.get("name")
        if event_name not in merged_events:
            merged_event = etree.Element("Event", name=event_name, priority = "overwrite", maximumExecutionCount = "1")
            merged_events[event_name] = merged_event
        else:
            merged_event = merged_events[event_name]

        for child in event:
            if child.tag == "Action":
                merged_event.append(child)
            elif child.tag == "StartTrigger":
                for existing_trigger in merged_event.findall("StartTrigger"):
                    merged_event.remove(existing_trigger)
                merged_event.append(child)  # Keep only the last <StartTrigger>

    return list(merged_events.values())


def process_file(input_folder, output_folder):
    """
    Args:
        input_folder: Report path
        output_folder: XOSC path

    Returns: Process the entire XOSC to identify common triggers

    """
    input_file = os.path.basename(input_folder)
    output_file = os.path.basename(output_folder)
    parser = etree.XMLParser(remove_blank_text=True)
    tree = etree.parse(input_file, parser)
    root = tree.getroot()

    for maneuver_group in root.findall(".//ManeuverGroup"):
        for maneuver in maneuver_group.findall(".//Maneuver"):
            event_elements = maneuver.findall("Event")
            if event_elements:
                merged_events = merge_events(event_elements)

                for event in event_elements:
                    maneuver.remove(event)

                for merged_event in merged_events:
                    maneuver.append(merged_event)

    # Apply stop condition using lxml
    stop_condition(root)

    # Define the output file path
    with open(output_file, "wb") as file:
        tree.write(file, pretty_print=True, encoding="utf-8")

    print(f"XML has been written to '{output_file}' successfully!")

def stop_condition(root):
    """
    Modify the XML tree by adding a stop condition if required when VCAR option is selected
    """
    for parent in root.xpath(".//ConditionGroup/.."):  # Get parent elements of ConditionGroup
        for condition_group in parent.findall("ConditionGroup"):
            condition = condition_group.find("./Condition/ByValueCondition/SimulationTimeCondition")
            if condition is not None and condition.get("value") == "666.0":
                # Create a new ConditionGroup using lxml
                new_condition_group = etree.Element("ConditionGroup")

                new_condition = etree.SubElement(new_condition_group, "Condition", {
                    "conditionEdge": "none",
                    "delay": "0",
                    "name": "StopParameterCondition"
                })

                by_value_condition = etree.SubElement(new_condition, "ByValueCondition")
                etree.SubElement(by_value_condition, "UserDefinedValueCondition", {
                    "name": "EndTheCase",
                    "rule": "equalTo",
                    "value": "1"
                })

                # Insert the new ConditionGroup **immediately after** the found one
                index = list(parent).index(condition_group) + 1
                parent.insert(index, new_condition_group)

                break  # Stop after inserting once

    print("Stop condition applied successfully.")