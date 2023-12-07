REM batch file to generate Pyslvs.exe
set Disk=y

for /f %%v in ('python -c "from pyslvs_ui import __version__;print(__version__)"') do set "PYSLVSVER=%%v"
for /f %%v in ('python -c "import platform;print(''.join(platform.python_compiler().split()[:2]).replace('.', '').lower())"') do set "COMPILERVER=%%v"
for /f %%v in ('python -c "import platform;print(platform.machine().lower())"') do set "SYSVER=%%v"
set "EXENAME=pyslvs-%PYSLVSVER%.%COMPILERVER%-%SYSVER%"

python -m PyInstaller -c -F "%Disk%:/tmp/Pyslvs-UI/scripts/entry.py" -n %EXENAME% -i "pyslvs_ui/icons/main.ico"  --add-data "pyslvs_ui/icons/*;pyslvs_ui/icons" --additional-hooks-dir "%Disk%:/tmp/Pyslvs-UI/scripts"