import os
import sys

MAIN_PATH = os.path.abspath(os.path.join(__file__, "..", ".."))
CURRENT_WORKING_FILE_DIRECTORY = os.path.abspath(os.path.join(__file__))
CURRENT_WORKING_DIRECTORY = os.path.abspath(os.path.join(__file__, ".."))

if MAIN_PATH not in sys.path:
    sys.path.append(MAIN_PATH)

dir_path = os.path.dirname(os.path.realpath(__file__))
for root, dirs, files in os.walk(dir_path):
    sys.path.append(root)


def global_parameter_info():
    """
    Global Parameters are added
    """
    pass


XLMR = {
    "ger_urban_traffic_light_junction.xlmr": "ger_urban_traffic_light_junction.xodr",
    "germany_urban_2lane_junction_v2.xlmr": "germany_urban_2lane_junction_v2.xodr",
    "ger_urban_2lane_junction.xlmr": "ger_urban_2lane_junction.xodr",
    "ger_urban_intersec_stopline.xlmr": "ger_urban_intersec_stopline.xodr",
    "ger_urban_t_junctions_v2.xlmr": "ger_urban_t_junctions_v2.xodr",
    "ger_urban_guard_rails_v2.xlmr": "ger_urban_guard_rails_v2.xodr",
    "parking_bay_loop_v2.xlmr": "parking_bay_loop_v2.xodr",
    "parking_slots_frontal.xlmr": "parking_slots_frontal.xodr",
    "parking_bay_loop_inclination.xlmr": "parking_bay_loop_inclination.xodr",
    "curved_parking_slots.xlmr": "curved_parking_slots.xodr",
    "ger_country_roundabout_2lanes_v2.xlmr": "ger_country_roundabout_2lanes_v2.xodr",
    "ger_country_2lanes.xlmr": "ger_country_2lanes.xodr",
    "ger_country_2lanes_tight_curves.xlmr": "ger_country_2lanes_tight_curves.xodr",
    "ger_country_roundabout_2lanes.xlmr": "ger_country_roundabout_2lanes.xodr",
    "germany_country_2lanes_rq11_straight_double_lane_marking.xlmr": "germany_country_2lanes_rq11_straight_double_lane_marking.xodr",
    "germany_country_2lanes_rq11_straight_missing_markings.xlmr": "germany_country_2lanes_rq11_straight_missing_markings.xodr",
    "germany_hw_6lanes_rq36_10km_straight_v2.xlmr": "germany_hw_6lanes_rq36_10km_straight_v2.xodr",
    "germany_hw_6lanes_rq36_10km_straight.xlmr": "germany_hw_6lanes_rq36_10km_straight.xodr",
    "germany_hw_4lanes_rq31_10km_straight_solid_markings_v2.xlmr": "germany_hw_4lanes_rq31_10km_straight_solid_markings_v2.xodr",
    "germany_hw_4lanes_rq31_10km_straight_solid_markings.xlmr": "germany_hw_4lanes_rq31_10km_straight_solid_markings.xodr",
    "germany_hw_5lanes_straight_split_merge.xlmr": "germany_hw_5lanes_straight_split_merge.xodr",
    "germany_hw_5lanes_straight_split_merge_v2.xlmr": "germany_hw_5lanes_straight_split_merge_v2.xodr",
    "germany_hw_4lanes_rq31_10km_straight.xlmr": "germany_hw_4lanes_rq31_10km_straight.xodr",
    "germany_hw_4lanes_rq31_10km_straight_v2.xlmr": "germany_hw_4lanes_rq31_10km_straight_v2.xodr",
    "ger_hw_entry_exit_ramp_exit.xlmr": "ger_hw_entry_exit_ramp_exit.xodr",
    "ger_hw_4lanes_missing_markings.xlmr": "ger_hw_4lanes_missing_markings.xodr",
    "ger_hw_4lanes_missing_markings_v2.xlmr": "ger_hw_4lanes_missing_markings_v2.xodr",
    "germany_hw_4lanes_rq31_10km_straight_construction_v2.xlmr": "germany_hw_4lanes_rq31_10km_straight_construction_v2.xodr",
    "ger_hw_6lanes_side_variations.xlmr": "ger_hw_6lanes_side_variations.xodr",
    "germany_hw_4lanes_rq31_10km_straight_reduced_width_v2.xlmr": "germany_hw_4lanes_rq31_10km_straight_reduced_width_v2.xodr",
    "scene_herrenberg_cloverleaf.xlmr": "scene_herrenberg_cloverleaf.xodr",
    "ger_hw_6lanes_side_variations_v2.xlmr": "ger_hw_6lanes_side_variations_v2.xodr",
    "ger_hw_2lanes_4lanes.xlmr": "ger_hw_2lanes_4lanes.xodr",
    "ger_hw_4lane_dependent_speedlimit.xlmr": "ger_hw_4lane_dependent_speedlimit.xodr",
    "ger_hw_6lanes_with_guardrail_and_lane_marking_variation.xlmr": "ger_hw_6lanes_with_guardrail_and_lane_marking_variation.xodr",
    "2lane_road_10km.xlmr": "2lane_road_10km.xodr",
    "ger_hw_s_bend_road.xlmr": "ger_hw_s_bend_road.xodr",
    "oval_map_v2.xlmr": "oval_map_v2.xodr",
    "oval_map.xlmr": "oval_map.xodr",
    "scene_loop_merge_v2.xlmr": "scene_loop_merge_v2.xodr"
}

VEHICLE_CATEGORIES = {'car': 'car',
                      'cars': 'car',
                      'van': 'van',
                      'vans': 'van',
                      'truck': 'truck',
                      'trucks': 'truck',
                      'trailer': 'trailer',
                      'trailers': 'trailer',
                      'semitrailer': 'semitrailer',
                      'semitrailers': 'semitrailer',
                      'bus': 'bus',
                      'buses': 'bus',
                      'twowheelers': 'motorbike',
                     'twowheeler': 'motorbike',
                      # 'twowheeler': 'bicycle',
                      'human': 'pedestrian',
                      'motorbikes': 'motorbike',
                      'bicycle': 'bicycle',
                      'bicycles': 'bicycle',
                      'train': 'train',
                      'trains': 'train',
                      'tram': 'tram',
                      'trams': 'tram'
                      }

VEHICLE_NAME = {'car': 'ENCAP_GVT01',
                'bicycle': 'Bike01',
                'pedestrian': 'Ped00',
                'twowheelers': 'Motorcycle02'

                #'motorbike' : 'Bike03'
                # 'van' :,
                # 'truck' :,
                # 'trailer' :,
                # 'semitrailer':,
                # 'bus' :,
                # 'motorbike':,
                # 'train':,
                # 'tram' :
                }
