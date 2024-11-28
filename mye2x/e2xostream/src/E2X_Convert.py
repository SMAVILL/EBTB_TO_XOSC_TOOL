# import os
# import sys
# import shutil
# from pathlib import Path
# from e2xostream.src.vehiclestream.xosc_stream import XOSCScenarioDevelop as XSD
#
# MAIN_PATH = os.path.abspath(os.path.join(__file__, "..", "..", "..", ".."))
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
#
# class E2XOStream:
#     def __init__(self, **kwargs):
#         """Constructor with kwargs."""
#         self.Kwargs = kwargs
#
#     def XOSCStream(self, destination_directory, original_file_path, xml_file_path, report_path, esmini_path):
#         """
#         Define scenario streamline
#         Parameters
#         ----------
#         destination_directory
#         original_file_path
#         xml_file_path
#         report_path
#
#         Returns
#         -------
#
#         """
#         original_file_path = os.path.join(original_file_path, "XOSCScenarioDevelop0.xosc")
#         new_file_path = str(Path(xml_file_path).stem) + ".xosc"
#         XSD.execute_sce_proc(xml_file_path=xml_file_path, report_path=report_path, esmini_path=esmini_path)
#         if not destination_directory.exists():
#             destination_directory.mkdir(parents=True, exist_ok=True)
#             destination_file_path = destination_directory / new_file_path
#             shutil.copy2(original_file_path, destination_file_path)
#         else:
#             destination_file_path = destination_directory / new_file_path
#             shutil.copy2(original_file_path, destination_file_path)
#
#         separator = "*" * 20
#         print(f"{separator}\nEBTB Input File: {xml_file_path}\n")
#         print(f"XOSC generated at: {destination_file_path}\n{separator}\n")
#
#
# if __name__ == "__main__":
#     pass

import os
import sys
import shutil
from pathlib import Path
from e2xostream.src.vehiclestream.xosc_stream import XOSCScenarioDevelop as XSD

MAIN_PATH = os.path.abspath(os.path.join(__file__, "..", "..", "..", ".."))
CURRENT_WORKING_FILE_DIRECTORY = os.path.abspath(os.path.join(__file__))
CURRENT_WORKING_DIRECTORY = os.path.abspath(os.path.join(__file__, ".."))

if MAIN_PATH not in sys.path:
    sys.path.append(MAIN_PATH)

dir_path = os.path.dirname(os.path.realpath(__file__))
for root, dirs, files in os.walk(dir_path):
    sys.path.append(root)


class E2XOStream:
    def __init__(self, **kwargs):
        """Constructor with kwargs."""
        self.Kwargs = kwargs

    def XOSCStream(self, destination_directory, original_file_path, xml_file_path, report_path, esmini_path):
        """
        Define scenario streamline
        Parameters
        ----------
        destination_directory
        original_file_path
        xml_file_path
        report_path

        Returns
        -------

        """
        #original_file_path = Path(original_file_path)
        original_file_path = os.path.join(original_file_path, "XOSCScenarioDevelop0.xosc")
        new_file_path = str(Path(xml_file_path).stem) + ".xosc"
        XSD.execute_sce_proc(xml_file_path=xml_file_path, report_path=report_path, esmini_path=esmini_path)
        if not destination_directory.exists():
            destination_directory.mkdir(parents=True, exist_ok=True)
            destination_file_path = destination_directory / new_file_path
            shutil.copy2(original_file_path, destination_file_path)
        else:
            destination_file_path = destination_directory / new_file_path
            shutil.copy2(original_file_path, destination_file_path)

        separator = "*" * 20
        print(f"{separator}\nEBTB Input File: {xml_file_path}\n")
        print(f"XOSC generated at: {destination_file_path}\n{separator}\n")


if __name__ == "__main__":
    pass
