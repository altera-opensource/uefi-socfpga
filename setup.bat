@ECHO OFF
@REM #/** @file
@REM #  Windows environment setup batch file for building Altera SOCFPGA UEFI Bootloader
@REM #
@REM #  Copyright (c) 2015, Altera Corporation. All rights reserved.
@REM #
@REM #  Redistribution and use in source and binary forms, with or without modification,
@REM #  are permitted provided that the following conditions are met:
@REM #
@REM #  1. Redistributions of source code must retain the above copyright notice, this
@REM #  list of conditions and the following disclaimer.
@REM #
@REM #  2. Redistributions in binary form must reproduce the above copyright notice, this
@REM #  list of conditions and the following disclaimer in the documentation and/or other
@REM #  materials provided with the distribution.
@REM #
@REM #  3. Neither the name of the copyright holder nor the names of its contributors may
@REM #  be used to endorse or promote products derived from this software without specific
@REM #  prior written permission.
@REM #
@REM #  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
@REM #  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
@REM #  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
@REM #  SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
@REM #  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
@REM #  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
@REM #  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
@REM #  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
@REM #  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
@REM #  DAMAGE.
@REM #
@REM #**/

@ECHO.
@ECHO ==================================================
@ECHO           Altera SoCFPGA UEFI Bootloader
@ECHO --------------------------------------------------
@ECHO Build Instruction for Windows Development Platform
@ECHO ==================================================
@ECHO.
@ECHO Pre-Requisites
@ECHO 1. Install SoC Embedded Design Suite (SoCEDS)
@ECHO 2. Install ARM DS-5 (ds-5_installer available in SoCEDS root directory)
@ECHO 3. Install Quartus or Quartus Programmer
@ECHO.
@ECHO Start Menu Run --^> cmd.exe
@ECHO Open a new command prompt window. From Windows Start menu type cmd.
@ECHO Note: Do not use the SoC EDS Command Shell it is not compatible.
@ECHO First set the path of SOCEDS_DEST_ROOT to SoCEDS embedded folder.
@ECHO Change to the top-level directory of the source then type setup and make.
@ECHO For example:
@ECHO    c:\uefi-socfpga^> set SOCEDS_DEST_ROOT=C:\altera\15.1\embedded
@ECHO    c:\uefi-socfpga^> setup
@ECHO    c:\uefi-socfpga^> make help
@ECHO    c:\uefi-socfpga^> make
@ECHO.
@ECHO The latest documentations can be found on
@ECHO http://www.alterawiki.com/wiki/UEFI_Bootloader
@ECHO.

@REM ######################################
@REM # If cannot find SOCEDS_DEST_ROOT, last try path
set FALLBACK_SOCEDS_DEST_ROOT=S:\tools\soceds\15.1\183\windows64

@REM ######################################
@REM # Find SOCEDS_DEST_ROOT
IF NOT DEFINED SOCEDS_DEST_ROOT (GOTO:NOT_FOUND_SOCEDS_DEST_ROOT)
IF EXIST %SOCEDS_DEST_ROOT%\Embedded_Command_Shell.bat (GOTO:FOUND_SOCEDS_DEST_ROOT)
:NOT_FOUND_SOCEDS_DEST_ROOT
set SOCEDS_DEST_ROOT=C:\altera\16.1\embedded
IF EXIST %SOCEDS_DEST_ROOT%\Embedded_Command_Shell.bat (GOTO:FOUND_SOCEDS_DEST_ROOT)
set SOCEDS_DEST_ROOT=C:\altera\16.0\embedded
IF EXIST %SOCEDS_DEST_ROOT%\Embedded_Command_Shell.bat (GOTO:FOUND_SOCEDS_DEST_ROOT)
set SOCEDS_DEST_ROOT=C:\altera\15.1\embedded
IF EXIST %SOCEDS_DEST_ROOT%\Embedded_Command_Shell.bat (GOTO:FOUND_SOCEDS_DEST_ROOT)
set SOCEDS_DEST_ROOT=C:\altera\15.0\embedded
IF EXIST %SOCEDS_DEST_ROOT%\Embedded_Command_Shell.bat (GOTO:FOUND_SOCEDS_DEST_ROOT)
set SOCEDS_DEST_ROOT=%FALLBACK_SOCEDS_DEST_ROOT%
IF EXIST %SOCEDS_DEST_ROOT%\Embedded_Command_Shell.bat (GOTO:FOUND_SOCEDS_DEST_ROOT)
ECHO.
ECHO Cannot find Altera's Soc Embedded Design Suite (SoC EDS)
ECHO Please download and install it from http://dl.altera.com/soceds/
ECHO.
GOTO:END_SETUP
:FOUND_SOCEDS_DEST_ROOT
ECHO SOCEDS_DEST_ROOT = %SOCEDS_DEST_ROOT%

@REM ######################################
@REM # DS5_ROOT
set DS5_ROOT=%SOCEDS_DEST_ROOT%\ds-5

@REM ######################################
@REM # UEFI_BUILD_TOOLS
set UEFI_BUILD_TOOLS=%~dp0\tools
set NASM_PREFIX=%UEFI_BUILD_TOOLS%\nasm211\
set GCC48_BIN=%UEFI_BUILD_TOOLS%\gcc493-x86\bin\
set GCC48_DLL=%UEFI_BUILD_TOOLS%\gcc493-x86\dll\;%GCC48_BIN%

@REM ######################################
@REM # ARM Cross Compiler
set GCC48_ARM_PREFIX=%DS5_ROOT%\sw\gcc\bin\arm-linux-gnueabihf-

set RVCT_TOOLS_PATH=%DS5_ROOT%\sw\ARMCompiler5.05u1\bin\
IF EXIST %RVCT_TOOLS_PATH%\armcc.exe (GOTO:FOUND_ARMCC_EXE)
set RVCT_TOOLS_PATH=%DS5_ROOT%\sw\ARMCompiler5.05u2\bin\
IF EXIST %RVCT_TOOLS_PATH%\armcc.exe (GOTO:FOUND_ARMCC_EXE)
set RVCT_TOOLS_PATH=%DS5_ROOT%\sw\ARMCompiler5.06u1\bin\
IF EXIST %RVCT_TOOLS_PATH%\armcc.exe (GOTO:FOUND_ARMCC_EXE)
ECHO.
ECHO Cannot find armcc.exe
ECHO Please install DS-5
ECHO.
:FOUND_ARMCC_EXE

@REM ######################################
@REM # Device Tree Compiler
set PATH=%UEFI_BUILD_TOOLS%\dtc;%PATH%

@REM ######################################
@REM # set path to mkpimage.exe
set PATH=%UEFI_BUILD_TOOLS%\sed;%PATH%

@REM ######################################
@REM # set path to sed.exe
set PATH=%SOCEDS_DEST_ROOT%\host_tools\altera\mkpimage;%PATH%

@REM ######################################
@REM # set path to make.exe
set PATH=%GCC48_BIN%;%PATH%

@REM ######################################
@REM # set path to quartus_hps.exe
set PATH=%QUARTUS_ROOTDIR%\bin64;%PATH%

@REM ######################################
@REM # Run EDK2SETUP.bat
call edk2setup.bat

:END_SETUP


