import os

from lxml import etree
from collections import defaultdict


def merge_events(events):
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

        # Define the output file path
    with open(output_file, "wb") as file:
        tree.write(file, pretty_print=True, encoding="utf-8")

    print(f"XML has been written to '{output_file}' successfully!")

