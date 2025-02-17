
import os
import sys
import argparse
import time
import logging
from e2xostream.src import E2X_Convert
import platform
import getpass
from e2xostream import GUI
import subprocess
import shutil
import re
from pathlib import Path
import tkinter as tk
from tkinter import messagebox
from e2xostream.src.E2X_Convert import E2XOStream
from e2xostream.src.vehiclestream.xosc_stream.ego_mapping_acts import EgoScnearioActs
from e2xostream.src.vehiclestream.ebtb_stream import EBTBAnalyzer
from e2xostream.src.vehiclestream.xosc_stream.XOSCScenarioDevelop import FuncScenario



username = getpass.getuser()
print("Username:", username)

# Initialize ebtb path
print("Initializing GUI")
ebtb = GUI.ebtb_GUI()
print("GUI Initialized")



reports_path = os.path.join(ebtb, 'report')

# Set the working directory to the report directory within EBTB
os.makedirs(reports_path, exist_ok=True)
os.chdir(reports_path)

# Clear previous files in the report directory if needed
for file in os.listdir(reports_path):
    file_path = os.path.join(reports_path, file)
    if os.path.isfile(file_path):
        os.remove(file_path)
        print(f"Deleted old file: {file_path}")

# Configure logging
log_file_path = os.path.join(reports_path, 'EBTB_TO_XOSC_Conv_Status.log')

# Remove any existing log file to ensure a fresh start
if os.path.exists(log_file_path):
    os.remove(log_file_path)

# Set up logger
logger = logging.getLogger()
logger.setLevel(logging.DEBUG)

# Remove any pre-existing handlers (important if this code runs multiple times)
for handler in logger.handlers[:]:
    handler.close()
    logger.removeHandler(handler)

# File handler to write logs to a file in the EBTB report directory
file_handler = logging.FileHandler(log_file_path, mode='w')
file_handler.setLevel(logging.DEBUG)

# Console handler to also log to the console
console_handler = logging.StreamHandler()
console_handler.setLevel(logging.DEBUG)

# Formatter for both handlers
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
file_handler.setFormatter(formatter)
console_handler.setFormatter(formatter)

# Add both handlers to the logger
logger.addHandler(file_handler)
logger.addHandler(console_handler)
logger = logging.getLogger(__name__)

logger.info("Logging initialized successfully.")

MAIN_PATH = os.path.abspath(os.path.join(__file__, "..", ".."))
CURRENT_WORKING_FILE_DIRECTORY = os.path.abspath(os.path.join(__file__))
CURRENT_WORKING_DIRECTORY = os.path.abspath(os.path.join(__file__, ".."))

if MAIN_PATH not in sys.path:
    sys.path.append(MAIN_PATH)

dir_path = os.path.dirname(os.path.realpath(__file__))
for root, dirs, files in os.walk(dir_path):
    sys.path.append(root)


def create_path_if_not_exists(path):
    """
    Create the path if not exists
    """
    if not os.path.exists(path):
        os.makedirs(path)
        logger.info(f"Path '{path}' created.")
    else:
        logger.info(f"Path '{path}' already exists.")


def find_and_read_xebtb_files(directory):

    """
    Find the EBTB files and read the content
    """

    ebtb_files = []
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith('.xebtb'):
                filepath = os.path.join(root, file)
                ebtb_files.append(filepath)

    if len(ebtb_files) == 0:
        logger.info("Input folder given is empty")
        exit()

    logger.debug(f"Found {len(ebtb_files)} .xebtb files in directory '{directory}'.")
    return ebtb_files




def args_data():
    """
    Read the arguments passed to this script
    """

    parser = argparse.ArgumentParser(description="Read the XEBTB files from a directory or direct file and "
                                                 "read the user choice function")

    parser = argparse.ArgumentParser(description="Read the XEBTB files from a directory or direct file and "
                                                 "read the user choice function")
    parser.add_argument('--ebtb', type=str, default=ebtb,
                        help="The path to the .xebtb file or directory containing .xebtb files (default: %(default)s)")

    parser.add_argument('--function', type=str, default="AEB",
                        help="The function to convert and create folder")
    parser.add_argument('--report', type=str, default=ebtb + "/report", help="XOSC result output path")
    parser.add_argument('--esmini', default="False", type=str, help="Use esmini simulator")

    args = parser.parse_args()

    esmini_tool_path = None
    if str(args.esmini) == "True":
        if platform.system() == "Linux":
            esmini_tool_path = os.path.join(CURRENT_WORKING_DIRECTORY, "stk", "simulator_tools", "esmini_linux",
                                            "esmini")
        elif platform.system() == "Windows":
            esmini_tool_path = os.path.join(CURRENT_WORKING_DIRECTORY, "stk", "simulator_tools", "esmini_win", "esmini")

    create_path_if_not_exists(args.report)

    if os.path.isfile(args.ebtb) and args.ebtb.endswith('.xebtb'):
        logger.info(f"Provided path is a single .xebtb file: {args.ebtb}")
        return args.ebtb, args.function, args.report, esmini_tool_path
    elif os.path.isdir(args.ebtb):
        logger.info(f"Provided path is a directory: {args.ebtb}")
        return find_and_read_xebtb_files(args.ebtb), args.function, args.report, esmini_tool_path
    else:
        logger.error("The provided path is neither a .xebtb file nor a directory.")
        sys.exit(1)


def copy_xodr_share_to_local(sharepath, local_path):
    files_list = []
    if not os.path.isdir(local_path):
        os.makedirs(local_path, exist_ok=True)

    # Define the pattern to match XML files
    pattern = r".*\.xodr$"

    # Traverse the directory structure
    for dir1, subdir, files in os.walk(sharepath):
        if os.path.basename(dir1) == "XODR":
            for name in files:
                if re.match(pattern, name):  # Match XODR files
                    files_list.append(name)
                    src = os.path.join(dir1, name)
                    dest = os.path.join(local_path, name)
                    try:
                        shutil.copy(src, dest)
                    except Exception as e:
                        print(f"Error copying file {name}: {e}")  # Log specific errors
        else:
            print(f"Skipping directory: {dir1}")  # Debug: Log skipped directories



def list_files_in_directory(directory_path):
    try:
        files = [entry for entry in os.listdir(directory_path) if os.path.isfile(os.path.join(directory_path, entry))]
        print(files)
        return files
    except FileNotFoundError:
        print(f"The directory {directory_path} does not exist.")
        return []
    except PermissionError:
        print(f"Permission denied to access {directory_path}.")
        return []


def resolve_relative_path(base_path, relative_path):
    """
    Resolves `../../` in relative paths and constructs the correct absolute path.
    """
    # Count occurrences of "../" to determine levels up
    level_up_count = relative_path.count("../")

    # Remove "../" occurrences to get the actual filename
    cleaned_filename = relative_path.replace("../", "").strip("/")

    # Traverse up in the directory hierarchy
    for _ in range(level_up_count):
        base_path = os.path.dirname(base_path)

    return os.path.join(base_path, cleaned_filename)


def extract_and_copy_files(directory_path, text_file_path, destination_base_dir):
    filenames = list_files_in_directory(directory_path)
    joined_paths = []

    try:
        with open(text_file_path, 'r') as file:
            lines = file.readlines()

        for line in lines:
            parts = line.strip().split('=:=')
            if len(parts) == 2:
                part1, part2 = parts

                # Extract the filename from part1
                file_name_in_part1 = os.path.basename(part1)

                if file_name_in_part1 in filenames:
                    print(file_name_in_part1)

                    # Determine the base directory from part1
                    directory_path_part1 = os.path.dirname(part1)
                    print("Base Directory:", directory_path_part1)

                    # Resolve the correct path for the .xlmr file
                    resolved_xlmr_path = resolve_relative_path(directory_path_part1, part2)
                    print("Resolved XLMR Path:", resolved_xlmr_path)

                    joined_paths.append(resolved_xlmr_path)

                    # Create a new directory for the .xebtb file
                    new_dir = os.path.join(destination_base_dir, os.path.splitext(file_name_in_part1)[0])
                    os.makedirs(new_dir, exist_ok=True)

                    if resolved_xlmr_path.startswith(r"\\"):
                        resolved_xlmr_path = r"\\?\UNC" + resolved_xlmr_path[1:]

                    resolved_xlmr_path = resolved_xlmr_path.strip()
                    resolved_xlmr_path = resolved_xlmr_path.encode("utf-8").decode("utf-8")

                    # Copy the .xlmr file to the new directory
                    if os.path.exists(resolved_xlmr_path):
                        print(f"✅ File found: {resolved_xlmr_path}")
                        shutil.copy(resolved_xlmr_path, new_dir)
                    else:
                        print(f"❌ File does NOT exist: '{resolved_xlmr_path}'")

    except FileNotFoundError:
        print(f"The file {text_file_path} does not exist.")
    except Exception as e:
        print(f"An error occurred: {e}")

    return joined_paths


import os
import getpass
import tkinter as tk
from tkinter import messagebox



def show_popup(message, title="Access Error"):
    """
    Display a popup with the specified message and title.
    """
    root = tk.Tk()
    root.withdraw()  # Hide the main tkinter window
    messagebox.showerror(title, message)
    root.destroy()


# Get the current username
username = getpass.getuser()
print(f"Username: {username}")

# Check access to the shared path
def check_shared_path_access(path):
    """
    Check if the user has access to the shared path.
    """
    if os.path.exists(path) and os.access(path, os.R_OK):
        logger.info(f"Access confirmed for path: {path}")
        # show_popup(f"User '{username}' has access to the shared path:\n{path}", "Access Confirmed")
        return True
    else:
        logger.error(f"Access denied for path: {path}")
        show_popup(f"User '{username}' does not have access to the shared path:\n{path}", "Access Denied")
        return False


if __name__ == "__main__":
    """
    Execute the main script
    """
    try:
        print("try")

        sharepath = r"\\srtif007\RDI-IAZ\01_Projects\02_IDC6\03_DigitalValidation\02_Pangu_Project\2_TestCase_development\1_EBTB_To_XOSC_Study\_EBTB_Assets\XODR"
        #sharepath1 = r"\\srtif007\RDI-IAZ\01_Projects\02_IDC6\03_DigitalValidation\02_Pangu_Project\2_TestCase_development\1_EBTB_To_XOSC_Study\_EBTB_Assets\XLMR"

        # logger.info("Script execution started.")
        try:
            print("try1")

            xml_file_path, function, report_path, esmini_path = args_data()

            sharepath = r"\\srtif007\RDI-IAZ\01_Projects\02_IDC6\03_DigitalValidation\02_Pangu_Project\2_TestCase_development\1_EBTB_To_XOSC_Study\_EBTB_Assets\XODR"
            #sharepath1 = r"\\srtif007\RDI-IAZ\01_Projects\02_IDC6\03_DigitalValidation\02_Pangu_Project\2_TestCase_development\1_EBTB_To_XOSC_Study\_EBTB_Assets\XLMR"


            #Check access for both shared paths
            if not check_shared_path_access(sharepath):
                sys.exit(1)  # Exit if access to the first path is denied

            # if not check_shared_path_access(sharepath1):
            #     sys.exit(1)  # Exit if access to the second path is denied

            local_path = Path(os.path.join(ebtb, "report", "xlmrmaps"))
            destination_base_dir = local_path
            text_file_path = r"\\srtif007\RDI-IAZ\01_Projects\02_IDC6\03_DigitalValidation\02_Pangu_Project\2_TestCase_development\1_EBTB_To_XOSC_Study\5_Verification_Reports_by_Tools_Team\PR13\XLMR_Mapping.txt"
            directory_path = ebtb
            extract_and_copy_files(directory_path, text_file_path, destination_base_dir)

            local_path = Path(os.path.join(ebtb, "report", "xodrmaps"))
            copy_xodr_share_to_local(sharepath, local_path)

        except Exception as e:
            logger.error(f"An unexpected error occurred: {str(e)}")
            show_popup(f"An unexpected error occurred:\n{str(e)}", "Unexpected Error")
            sys.exit(1)

        original_file_path = os.path.join(report_path, "xosc")
        destination_directory = Path(report_path)

        E2XObj = E2X_Convert.E2XOStream()

        if os.path.isfile(xml_file_path) and xml_file_path.endswith('.xebtb'):
            E2XObj.XOSCStream(destination_directory=destination_directory,
                              original_file_path=original_file_path,
                              xml_file_path=xml_file_path,
                              report_path=report_path,
                              esmini_path=esmini_path)
            logger.info(f"Processed file: {xml_file_path}")

        elif isinstance(xml_file_path, list):
            for xml_file in xml_file_path:
                xml_file_path = [os.path.normpath(xml_file)]
                E2XObj.XOSCStream(destination_directory=destination_directory,
                                  original_file_path=original_file_path,
                                  xml_file_path=xml_file,
                                  report_path=report_path,
                                  esmini_path=esmini_path)
                logger.info(f"Processed file: {xml_file}")
                time.sleep(1)

    except Exception as e:
        print("excepts")

        #logger.error(f"An  occurred: {str(e)}")
        for xml_file in xml_file_path:
            try:
                destination_file_path =E2XObj.XOSCStream(destination_directory=destination_directory,
                                  original_file_path=original_file_path,
                                  xml_file_path=xml_file,
                                  report_path=report_path,
                                  esmini_path=esmini_path)

                if EgoScnearioActs.flag == 1:
                    os.remove(E2XOStream.new_file_path)
                    EgoScnearioActs.flag = 0
                    logger.error(f"Check in API {EgoScnearioActs.error_name}  - {E2XOStream.new_file_path}")
                    EgoScnearioActs.error_name = None


                if EBTBAnalyzer.parking_flag == 1:
                    os.remove(E2XOStream.new_file_path)
                    EBTBAnalyzer.parking_flag = 0
                    logger.error(f"Check in API {EBTBAnalyzer.error_name}  - {E2XOStream.new_file_path}")
                    EBTBAnalyzer.error_name = None

                if FuncScenario.ego_flag == 1:
                    os.remove(E2XOStream.new_file_path)
                    FuncScenario.ego_flag = 0
                    logger.error(f"{FuncScenario.ego_error} - {E2XOStream.new_file_path}")
                    FuncScenario.ego_error = None

                if FuncScenario.obj_flag == 1:
                    os.remove(E2XOStream.new_file_path)
                    FuncScenario.obj_flag = 0
                    logger.error(f"{FuncScenario.obj_error} - {E2XOStream.new_file_path}")
                    FuncScenario.obj_error = None


                from e2xostream import merge
                merge.process_file(destination_file_path, destination_file_path)

                logger.info(f"Processed file in exception handler - {xml_file}")
                time.sleep(1)
            except Exception as ex:
                logger.error(f"{str(ex)} - {xml_file} ")
                logger.info(f"Processed file in exception handler - {xml_file}")

    import shutil

    unwanted_folders = ["xosc", "xodr"]

    for folder in unwanted_folders:
        folder_path = os.path.join(report_path, folder)
        if os.path.exists(folder_path):
            shutil.rmtree(folder_path)
            logger.info(f"Removed unwanted folder: {folder_path}")


    GUI.create_html(ebtb)

    logger.info("Script execution finished.")
