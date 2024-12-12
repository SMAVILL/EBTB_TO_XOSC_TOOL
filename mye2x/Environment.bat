set ENV_PATH=C:\4_Pangu_Gen6_Project\Pangu_Automation_Tools\EBTB_TO_XOSC_TOOL\Myenv\Scripts\activate

:: Activate the environment
echo Activating the Python environment from: %ENV_PATH%
call "%ENV_PATH%"

:: Keep the command prompt open
echo.
echo Python environment is now active.
echo You can run Python commands or scripts from here.
echo.
cmd /k