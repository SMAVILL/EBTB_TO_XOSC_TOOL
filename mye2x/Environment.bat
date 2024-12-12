

@echo off
:: Prompt user for input
:: Define paths
set ENV_PATH=C:\4_Pangu_Gen6_Project\Pangu_Automation_Tools\EBTB_TO_XOSC_TOOL\Myenv\Scripts\activate
set PYTHON_SCRIPT=C:\4_Pangu_Gen6_Project\Pangu_Automation_Tools\EBTB_TO_XOSC_TOOL\mye2x\main.py

echo Activating the Python environment from: %ENV_PATH%
call "%ENV_PATH%"
echo.
echo Python environment is now active.


echo Select an option:
echo [1] Running Main.py 
echo [2] Create .exe
echo [3] Exit
set /p CHOICE=Enter your choice (1/2/3):

:: Handle user choice
if "%CHOICE%"=="1" (
    echo Running Python script: %PYTHON_SCRIPT%  
	python "%PYTHON_SCRIPT%"
	goto end
)else if "%CHOICE%"=="2" (
    echo Building New exe: %PYTHON_SCRIPT%  
	pyinstaller --onefile "%PYTHON_SCRIPT%"
	goto end
	
) else if "%CHOICE%"=="3" (
    echo Exiting. Goodbye!
    goto end
) else (
    echo Invalid choice. Please run the script again and choose 1, 2, or 3.
    goto end
)

:end


:: Keep the command prompt open
cmd /k