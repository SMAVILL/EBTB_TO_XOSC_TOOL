# import os
# import sys
# import argparse
# import time
# import logging
# from pathlib import Path
# from e2xostream.src import E2X_Convert
# import platform
# import getpass
# from e2xostream import GUI
# import subprocess
# import shutil
# import re
# from pathlib import Path
# import tkinter as tk
# from tkinter import messagebox
#
# username = getpass.getuser()
# print("Username:", username)
#
# # Initialize ebtb path
# ebtb = GUI.ebtb_GUI()
#
#
#
# reports_path = os.path.join(ebtb, 'report')
#
# # Set the working directory to the report directory within EBTB
# os.makedirs(reports_path, exist_ok=True)
# os.chdir(reports_path)
#
#
# # Clear previous files in the report directory if needed
# for file in os.listdir(reports_path):
#     file_path = os.path.join(reports_path, file)
#     if os.path.isfile(file_path):
#         os.remove(file_path)
#         print(f"Deleted old file: {file_path}")
#
# # Configure logging
# log_file_path = os.path.join(reports_path, 'EBTB_TO_XOSC_Conv_Status.log')
#
# # Remove any existing log file to ensure a fresh start
# if os.path.exists(log_file_path):
#     os.remove(log_file_path)
#
# # Set up logger
# logger = logging.getLogger()
# logger.setLevel(logging.DEBUG)
#
# # Remove any pre-existing handlers (important if this code runs multiple times)
# for handler in logger.handlers[:]:
#     handler.close()
#     logger.removeHandler(handler)
#
# # File handler to write logs to a file in the EBTB report directory
# file_handler = logging.FileHandler(log_file_path, mode='w')
# file_handler.setLevel(logging.DEBUG)
#
# # Console handler to also log to the console
# console_handler = logging.StreamHandler()
# console_handler.setLevel(logging.DEBUG)
#
# # Formatter for both handlers
# formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
# file_handler.setFormatter(formatter)
# console_handler.setFormatter(formatter)
#
# # Add both handlers to the logger
# logger.addHandler(file_handler)
# logger.addHandler(console_handler)
# logger = logging.getLogger(__name__)
#
#
# logger.info("Logging initialized successfully.")
#
# MAIN_PATH = os.path.abspath(os.path.join(__file__, "..", ".."))
# CURRENT_WORKING_FILE_DIRECTORY = os.path.abspath(os.path.join(__file__))
# CURRENT_WORKING_DIRECTORY = os.path.abspath(os.path.join(__file__, ".."))
#
# if MAIN_PATH not in sys.path:
#     sys.path.append(MAIN_PATH)
#
# dir_path = os.path.dirname(os.path.realpath(__file__))
# for root, dirs, files in os.walk(dir_path):
#     sys.path.append(root)
#
# def create_path_if_not_exists(path):
#     """
#     Create the path if not exists
#     """
#     if not os.path.exists(path):
#         os.makedirs(path)
#         logger.info(f"Path '{path}' created.")
#     else:
#         logger.info(f"Path '{path}' already exists.")
#
#
#
# def find_and_read_xebtb_files(directory):
#     """
#     Find the EBTB files and read the content
#     """
#
#     ebtb_files = []
#     for root, dirs, files in os.walk(directory):
#         for file in files:
#             if file.endswith('.xebtb'):
#                 filepath = os.path.join(root, file)
#                 ebtb_files.append(filepath)
#
#     if len(ebtb_files) == 0:
#         logger.info("Input folder given is empty")
#         exit()
#
#     logger.debug(f"Found {len(ebtb_files)} .xebtb files in directory '{directory}'.")
#     return ebtb_files
#
#
# # ebtb = GUI.ebtb_GUI()
#
# def args_data():
#
#     """
#     Read the arguments passed to this script
#     """
#
#     parser = argparse.ArgumentParser(description="Read the XEBTB files from a directory or direct file and "
#                                                  "read the user choice function")
#
#     parser = argparse.ArgumentParser(description="Read the XEBTB files from a directory or direct file and "
#                                                  "read the user choice function")
#     parser.add_argument('--ebtb', type=str, default=ebtb,
#                         help="The path to the .xebtb file or directory containing .xebtb files (default: %(default)s)")
#
#     parser.add_argument('--function', type=str, default = "AEB",
#                         help="The function to convert and create folder")
#     parser.add_argument('--report', type=str, default=ebtb+"/report", help="XOSC result output path")
#     parser.add_argument('--esmini', default="False", type=str, help="Use esmini simulator")
#
#     args = parser.parse_args()
#
#     esmini_tool_path = None
#     if str(args.esmini) == "True":
#         if platform.system() == "Linux":
#             esmini_tool_path = os.path.join(CURRENT_WORKING_DIRECTORY, "stk", "simulator_tools", "esmini_linux", "esmini")
#         elif platform.system() == "Windows":
#             esmini_tool_path = os.path.join(CURRENT_WORKING_DIRECTORY, "stk", "simulator_tools", "esmini_win", "esmini")
#
#     create_path_if_not_exists(args.report)
#
#     if os.path.isfile(args.ebtb) and args.ebtb.endswith('.xebtb'):
#         logger.info(f"Provided path is a single .xebtb file: {args.ebtb}")
#         return args.ebtb, args.function, args.report, esmini_tool_path
#     elif os.path.isdir(args.ebtb):
#         logger.info(f"Provided path is a directory: {args.ebtb}")
#         return find_and_read_xebtb_files(args.ebtb), args.function, args.report, esmini_tool_path
#     else:
#         logger.error("The provided path is neither a .xebtb file nor a directory.")
#         sys.exit(1)
#
#
# def copy_xodr_share_to_local(sharepath, local_path):
#     files_list = []
#     if not os.path.isdir(local_path):
#         os.makedirs(local_path, exist_ok=True)
#
#     # Define the pattern to match XML files
#     pattern = r".*\.xodr$"
#
#     # Traverse the directory structure
#     for dir1, subdir, files in os.walk(sharepath):
#         if os.path.basename(dir1) == "xodrmaps":
#             for name in files:
#                 if re.match(pattern, name):  # Match XODR files
#                     files_list.append(name)
#                     src = os.path.join(dir1, name)
#                     dest = os.path.join(local_path, name)
#                     try:
#                         shutil.copy(src, dest)
#                     except Exception as e:
#                         print(f"Error copying file {name}: {e}")  # Log specific errors
#         else:
#             print(f"Skipping directory: {dir1}")  # Debug: Log skipped directories
#
# def copy_xlmr_share_to_local(sharepath1, local_path):
#     files_list = []
#     if not os.path.isdir(local_path):
#         os.makedirs(local_path, exist_ok=True)
#
#     # Define the pattern to match XML files
#     pattern = r".*\.xlmr$"
#
#     # Traverse the directory structure
#     for dir1, subdir, files in os.walk(sharepath1):
#         if os.path.basename(dir1) == "xlmrmaps":
#             for name in files:
#                 if re.match(pattern, name):  # Match XODR files
#                     files_list.append(name)
#                     src = os.path.join(dir1, name)
#                     dest = os.path.join(local_path, name)
#                     try:
#                         shutil.copy(src, dest)
#                     except Exception as e:
#                         print(f"Error copying file {name}: {e}")  # Log specific errors
#         else:
#             print(f"Skipping directory: {dir1}")  # Debug: Log skipped directories
#
#
# if __name__ == "__main__":
#     """
#     Execute the main script
#     """
#     logger.info("Script execution started.")
#     try:
#
#         xml_file_path, function, report_path, esmini_path = args_data()
#
#
#         sharepath = r"\\srtif007\RDI-CEA\Projects\Artemis\03_ADAS\02_Pangu_Project\5_Training_Material\Pangu_Automation_Tools\EBTB_XOSC_Tool_Conv\xodrmaps"
#         sharepath1 = r"\\srtif007\RDI-CEA\Projects\Artemis\03_ADAS\02_Pangu_Project\5_Training_Material\Pangu_Automation_Tools\EBTB_XOSC_Tool_Conv\xlmrmaps"
#         if not os.path.exists(sharepath):
#             print(f"Error: Network path not found: {sharepath}")
#         else:
#             local_path = Path(os.path.join(ebtb, "report", "xodrmaps"))
#             copy_xodr_share_to_local(sharepath, local_path)
#
#         if not os.path.exists(sharepath1):
#             print(f"Error: Network path not found: {sharepath1}")
#         else:
#             local_path = Path(os.path.join(ebtb, "report", "xlmrmaps"))
#             copy_xlmr_share_to_local(sharepath1, local_path)
#
#
#         original_file_path = os.path.join(report_path,"xosc")
#         destination_directory = Path(report_path)
#
#         E2XObj = E2X_Convert.E2XOStream()
#
#         if os.path.isfile(xml_file_path) and xml_file_path.endswith('.xebtb'):
#             E2XObj.XOSCStream(destination_directory=destination_directory,
#                               original_file_path=original_file_path,
#                               xml_file_path=xml_file_path,
#                               report_path=report_path,
#                               esmini_path=esmini_path)
#             logger.info(f"Processed file: {xml_file_path}")
#         elif isinstance(xml_file_path, list):
#             for xml_file in xml_file_path:
#                 xml_file_path = [os.path.normpath(xml_file)]
#                 E2XObj.XOSCStream(destination_directory=destination_directory,
#                                   original_file_path=original_file_path,
#                                   xml_file_path=xml_file,
#                                   report_path=report_path,
#                                   esmini_path=esmini_path)
#                 logger.info(f"Processed file: {xml_file}")
#                 time.sleep(1)
#
#     except Exception as e:
#         logger.error(f"An error occurred: {str(e)}")
#         for xml_file in xml_file_path:
#             try:
#                 E2XObj.XOSCStream(destination_directory=destination_directory,
#                                   original_file_path=original_file_path,
#                                   xml_file_path=xml_file,
#                                   report_path=report_path,
#                                   esmini_path=esmini_path)
#                 logger.info(f"Processed file in exception handler: {xml_file}")
#                 time.sleep(1)
#             except Exception as ex:
#                 logger.error(f"Failed to process file {xml_file}: {str(ex)}")
#     import shutil
#     unwanted_folders = ["xosc", "xodr"]
#     for folder in unwanted_folders:
#         folder_path = os.path.join(report_path, folder)
#         if os.path.exists(folder_path):
#             shutil.rmtree(folder_path)
#             logger.info(f"Removed unwanted folder: {folder_path}")
#
#
#     GUI.create_html(ebtb)
#
#
#     logger.info("Script execution finished.")


import os
import sys
import argparse
import time
import logging
from pathlib import Path
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

username = getpass.getuser()
print("Username:", username)

# Initialize ebtb path
ebtb = GUI.ebtb_GUI()


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


# ebtb = GUI.ebtb_GUI()

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
        if os.path.basename(dir1) == "xodrmaps":
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


def copy_xlmr_share_to_local(sharepath1, local_path):
    files_list = []
    if not os.path.isdir(local_path):
        os.makedirs(local_path, exist_ok=True)

    # Define the pattern to match XML files
    pattern = r".*\.xlmr$"

    # Traverse the directory structure
    for dir1, subdir, files in os.walk(sharepath1):
        if os.path.basename(dir1) == "xlmrmaps":

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

# Define shared paths
sharepath = r"\\srtif007\RDI-CEA\Projects\Artemis\03_ADAS\02_Pangu_Project\5_Training_Material\Pangu_Automation_Tools\EBTB_XOSC_Tool_Conv\xodrmaps"
sharepath1 = r"\\srtif007\RDI-CEA\Projects\Artemis\03_ADAS\02_Pangu_Project\5_Training_Material\Pangu_Automation_Tools\EBTB_XOSC_Tool_Conv\xlmrmaps"


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

        sharepath = r"\\srtif007\RDI-CEA\Projects\Artemis\03_ADAS\02_Pangu_Project\5_Training_Material\Pangu_Automation_Tools\EBTB_XOSC_Tool_Conv\xodrmaps"
        sharepath1 = r"\\srtif007\RDI-CEA\Projects\Artemis\03_ADAS\02_Pangu_Project\5_Training_Material\Pangu_Automation_Tools\EBTB_XOSC_Tool_Conv\xlmrmaps"

        # logger.info("Script execution started.")
        try:

            xml_file_path, function, report_path, esmini_path = args_data()

            sharepath = r"\\srtif007\RDI-CEA\Projects\Artemis\03_ADAS\02_Pangu_Project\5_Training_Material\Pangu_Automation_Tools\EBTB_XOSC_Tool_Conv\xodrmaps"
            sharepath1 = r"\\srtif007\RDI-CEA\Projects\Artemis\03_ADAS\02_Pangu_Project\5_Training_Material\Pangu_Automation_Tools\EBTB_XOSC_Tool_Conv\xlmrmaps"

            # Check access for both shared paths
            if not check_shared_path_access(sharepath):
                sys.exit(1)  # Exit if access to the first path is denied

            if not check_shared_path_access(sharepath1):
                sys.exit(1)  # Exit if access to the second path is denied

            local_path = Path(os.path.join(ebtb, "report", "xlmrmaps"))
            copy_xlmr_share_to_local(sharepath1, local_path)

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
        logger.error(f"An error occurred: {str(e)}")
        for xml_file in xml_file_path:
            try:
                E2XObj.XOSCStream(destination_directory=destination_directory,
                                  original_file_path=original_file_path,
                                  xml_file_path=xml_file,
                                  report_path=report_path,
                                  esmini_path=esmini_path)
                logger.info(f"Processed file in exception handler: {xml_file}")
                time.sleep(1)
            except Exception as ex:
                logger.error(f"Failed to process file {xml_file}: {str(ex)}")
    import shutil

    unwanted_folders = ["xosc", "xodr"]
    for folder in unwanted_folders:
        folder_path = os.path.join(report_path, folder)
        if os.path.exists(folder_path):
            shutil.rmtree(folder_path)
            logger.info(f"Removed unwanted folder: {folder_path}")

    GUI.create_html(ebtb)

    logger.info("Script execution finished.")