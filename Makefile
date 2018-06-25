#/** @file
#  Makefile to build Altera SOCFPGA UEFI
#
#  Copyright (c) 2015, Altera Corporation. All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without modification,
#  are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice, this
#  list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice, this
#  list of conditions and the following disclaimer in the documentation and/or other
#  materials provided with the distribution.
#
#  3. Neither the name of the copyright holder nor the names of its contributors may
#  be used to endorse or promote products derived from this software without specific
#  prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
#  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
#  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
#  SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
#  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
#  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
#  DAMAGE.
#
#**/

SHELL := /bin/bash

.SUFFIXES: # Delete the default suffixes

mkfile_path := $(dir $(CURDIR)/$(word $(words $(MAKEFILE_LIST)),$(MAKEFILE_LIST)))
export EDK_TOOLS_PATH?=$(mkfile_path)BaseTools

#-----------------------------------------------------------------------------
#                                  TOOLS
#-----------------------------------------------------------------------------

ifdef ComSpec
# Windows
NEXT_CMD         :=&
PATHSEP2         =\\
PATHSEP          =$(strip $(PATHSEP2))
ECHO_NEWLINE     := cmd /c echo.
ECHO_START       := echo
ECHO_END         :=
SED_START        :=
SED_END          :=
SED_DEL_TMP_FILE := del sed*.
DTC              := dtc
CP               := copy /y
RM               := del /f
CPDIR            := xcopy /s /q
RMDIR            := rmdir /s /q
MKDIR            := mkdir
MAKE_BUILD_TOOL  :=
CAT_TYPE         := type
EXIST            := exist
else
# Linux
NEXT_CMD         :=;
PATHSEP2         =/
PATHSEP          =$(strip $(PATHSEP2))
ECHO_NEWLINE     := echo ""
ECHO_START       := echo "
ECHO_END         := "
SED_START        := "
SED_END          := "
SED_DEL_TMP_FILE :=
DTC              := dtc
CP               := cp -f
RM               := rm -f
CPDIR            := cp -Rf
RMDIR            := rm -Rf
MKDIR            := mkdir
MAKE_BUILD_TOOL  := $(MAKE) -C $(EDK_TOOLS_PATH)
CAT_TYPE         := cat
EXIST            := test -d
endif

ifdef ComSpec
# Windows
else
# Linux
export GCC48_ARM_PREFIX=arm-linux-gnueabihf-
export GCC48_AARCH64_PREFIX=aarch64-linux-gnu-
endif

#-----------------------------------------------------------------------------
# Definition
#-----------------------------------------------------------------------------

# Default when user just type make
DEFAULT_MAKE_DEVICE := s10
DEFAULT_MAKE_COMPILER := gcc
DEFAULT_MAKE_TARGET := bootloader

# Build option whether RELEASE or DEBUG. For SOCFPGA, please only use 'RELEASE'
export EDK2_BUILD=RELEASE

# Additional build macros
export EDK2_MACROS= -y report.log -j build.log -Y PCD -Y LIBRARY -Y FLASH -Y DEPEX -Y BUILD_FLAGS -Y FIXED_ADDRESS

MKPIMAGE_HEADER_VERSION := 1

PEI_FD_FILENAME := ALTERA_HPS_OCRAM_EFI_PART1.fd
DXE_FD_FILENAME := ALTERA_HPS_DRAM_EFI_PART2.fd

PEI_FLASH_ADDR := 0x00000000

#-----------------------------------------------------------------------------
# Process the Command-line Arguments
#-----------------------------------------------------------------------------

ifeq ("$(device)$(D)$(d)","")
# Set Default DEVICE when user do not specify via command-line argument
DEVICE?=$(DEFAULT_MAKE_DEVICE)
endif

ifeq ("$(target)$(T)$(t)","")
# Set Default DEVICE when user do not specify via command-line argument
TARGET?=$(DEFAULT_MAKE_TARGET)
endif

# If the first argument is "app"...
ifeq (app,$(firstword $(MAKECMDGOALS)))
TARGET := app
endif

# Set Default COMPILER when user do not specify via command-line argument
# Support upper and lower case 'COMPILER' in command-line argument
ifeq ("$(compiler)$(C)$(c)","")
COMPILER?=$(DEFAULT_MAKE_COMPILER)
endif

# Type of Compiler toolchain
ifeq ("$(COMPILER)$(compiler)$(C)$(c)",$(filter "$(COMPILER)$(compiler)$(C)$(c)","armcc" "ARMCC"))
export EDK2_TOOLCHAIN=RVCTLINUX
# RVCT_TOOLS_PATH not needed as proper environment setup by SOCEDS
#export RVCT_TOOLS_PATH=$(SOCEDS_DEST_ROOT)$(PATHSEP)ds-5$(PATHSEP)sw$(PATHSEP)ARMCompiler5.05u1$(PATHSEP)bin$(PATHSEP)
else ifeq ("$(COMPILER)$(compiler)$(C)$(c)",$(filter "$(COMPILER)$(compiler)$(C)$(c)","gcc" "GCC"))
export EDK2_TOOLCHAIN=GCC48
else
$(error ERROR: Unsupported Compiler in command-line argument COMPILER=""$(COMPILER)$(compiler)$(C)$(c))
endif


ifeq ("$(DEVICE)$(device)$(D)$(d)",$(filter "$(DEVICE)$(device)$(D)$(d)","s10" "S10"))
  # Processor architecture which is AARCH64 for SOCFPGA
  export EDK2_ARCH=AARCH64

  # ENTRY_MINUS_40HEX = 0x188 (SEC EntryPoint) - 0x40
  ENTRY_MINUS_40HEX := 328

  # Build full path of the .FD files
  SEC_FD_FILENAME := ALTERA_HPS_OCRAM_EFI_PART1.fd
  PEI_FD_FILENAME := ALTERA_HPS_DRAM_EFI_PART2.fd
  DXE_FD_FILENAME := ALTERA_HPS_DRAM_EFI_PART3.fd
  SEC_FD_FILENAME_FULLPATH := Build$(PATHSEP)Stratix10SoCPkg$(PATHSEP)$(EDK2_BUILD)_$(EDK2_TOOLCHAIN)$(PATHSEP)FV$(PATHSEP)$(SEC_FD_FILENAME)
  PEI_FD_FILENAME_FULLPATH := Build$(PATHSEP)Stratix10SoCPkg$(PATHSEP)$(EDK2_BUILD)_$(EDK2_TOOLCHAIN)$(PATHSEP)FV$(PATHSEP)$(PEI_FD_FILENAME)
  DXE_FD_FILENAME_FULLPATH := Build$(PATHSEP)Stratix10SoCPkg$(PATHSEP)$(EDK2_BUILD)_$(EDK2_TOOLCHAIN)$(PATHSEP)FV$(PATHSEP)$(DXE_FD_FILENAME)
  PEI_SMALLER_x1_ROM       := Build$(PATHSEP)Stratix10SoCPkg$(PATHSEP)$(EDK2_BUILD)_$(EDK2_TOOLCHAIN)$(PATHSEP)PEI.256
  PEI_FINAL_ROM            := Build$(PATHSEP)Stratix10SoCPkg$(PATHSEP)$(EDK2_BUILD)_$(EDK2_TOOLCHAIN)$(PATHSEP)PEI.ROM
  DXE_FINAL_ROM            := Build$(PATHSEP)Stratix10SoCPkg$(PATHSEP)$(EDK2_BUILD)_$(EDK2_TOOLCHAIN)$(PATHSEP)DXE.ROM

  FILE_ArmPlatformSec        := $(mkfile_path)Build$(PATHSEP)Stratix10SoCPkg$(PATHSEP)$(EDK2_BUILD)_$(EDK2_TOOLCHAIN)$(PATHSEP)$(EDK2_ARCH)$(PATHSEP)AlteraPlatformPkg$(PATHSEP)Sec$(PATHSEP)Sec$(PATHSEP)DEBUG$(PATHSEP)ArmPlatformSec.dll
  FILE_ArmPlatformPrePeiCore := $(mkfile_path)Build$(PATHSEP)Stratix10SoCPkg$(PATHSEP)$(EDK2_BUILD)_$(EDK2_TOOLCHAIN)$(PATHSEP)$(EDK2_ARCH)$(PATHSEP)AlteraPlatformPkg$(PATHSEP)PrePeiCore$(PATHSEP)PrePeiCoreMPCore$(PATHSEP)DEBUG$(PATHSEP)ArmPlatformPrePeiCore.dll
  FILE_AlteraSocFpgaPeiMain  := $(mkfile_path)Build$(PATHSEP)Stratix10SoCPkg$(PATHSEP)$(EDK2_BUILD)_$(EDK2_TOOLCHAIN)$(PATHSEP)$(EDK2_ARCH)$(PATHSEP)AlteraPlatformPkg$(PATHSEP)Stratix10SoCPkg$(PATHSEP)PlatformPei$(PATHSEP)AlteraSocFpgaPeiMain$(PATHSEP)DEBUG$(PATHSEP)AlteraSocFpgaPeiMain.dll

  SNR_BUILD_PEI  := $(SED_START)SEC.ROM$(SED_END)
  FILE_BUILD_PEI := $(mkfile_path)$(SEC_FINAL_ROM)

  ENTRY_ArmPlatformSec        := $(SED_START)0x00ffe00188$(SED_END)
  ENTRY_ArmPlatformPrePeiCore := $(SED_START)0x0000050808$(SED_END)
  ENTRY_AlteraSocFpgaPeiMain  := $(SED_START)0x0000056180$(SED_END)
  # DS5 script path
  DS5_SCRIPT_PATH            := AlteraPlatformPkg$(PATHSEP)Stratix10SoCPkg$(PATHSEP)Stratix10SoCPkg.ds5
  DS_SCRIPT                  := Build$(PATHSEP)Stratix10SoCPkg$(PATHSEP)$(EDK2_BUILD)_$(EDK2_TOOLCHAIN)$(PATHSEP)load_uefi_fw.ds

  ifeq ("$(TARGET)$(target)$(T)$(t)",$(filter "$(TARGET)$(target)$(T)$(t)","bootloader" "BOOTLOADER"))
  export EDK2_DSC=AlteraPlatformPkg$(PATHSEP)Stratix10SoCPkg$(PATHSEP)Stratix10SoCPkg.dsc
  else ifeq ("$(TARGET)$(target)$(T)$(t)",$(filter "$(TARGET)$(target)$(T)$(t)","app" "APP"))
  export EDK2_DSC=AlteraPlatformPkg$(PATHSEP)Applications$(PATHSEP)SocFpgaAppPkg.dsc
  EFIAPP_FILENAME_FULLPATH := Build$(PATHSEP)SocFpgaAppPkg$(PATHSEP)$(EDK2_BUILD)_$(EDK2_TOOLCHAIN)$(PATHSEP)$(EDK2_ARCH)$(PATHSEP)*.efi
  else
  $(error ERROR: No .DSC for unsupported TARGET="$(TARGET)$(target)$(T)$(t))"
  endif
else
$(error ERROR: No .DSC for unsupported device in command-line argument DEVICE="$(DEVICE)$(device)$(D)$(d))"
endif



#-----------------------------------------------------------------------------
# DS-5 Script Setup
#-----------------------------------------------------------------------------
SNR_ENTRY_ArmPlatformSec        := $(SED_START)SEARCH_AND_REPLACE_ArmPlatformSec_entrypoint$(SED_END)
SNR_ENTRY_ArmPlatformPrePeiCore := $(SED_START)SEARCH_AND_REPLACE_ArmPlatformPrePeiCore_entrypoint$(SED_END)
SNR_ENTRY_AlteraSocFpgaPeiMain  := $(SED_START)SEARCH_AND_REPLACE_AlteraSocFpgaPeiMain_entrypoint$(SED_END)

SNR_FILE_ArmPlatformSec        := $(SED_START)SEARCH_AND_REPLACE_ArmPlatformSec_dll$(SED_END)
SNR_FILE_ArmPlatformPrePeiCore := $(SED_START)SEARCH_AND_REPLACE_ArmPlatformPrePeiCore_dll$(SED_END)
SNR_FILE_AlteraSocFpgaPeiMain  := $(SED_START)SEARCH_AND_REPLACE_AlteraSocFpgaPeiMain_dll$(SED_END)

HWLIB_SOCEDS_PATH := $(SOCEDS_DEST_ROOT)$(PATHSEP)ip$(PATHSEP)altera$(PATHSEP)hps$(PATHSEP)altera_hps$(PATHSEP)hwlib$(PATHSEP)*
HWLIB_UEFI_PATH   := AlteraPlatformPkg$(PATHSEP)HwLib
HWLIB_INCLUDE_PATH := $(HWLIB_UEFI_PATH)$(PATHSEP)include
HWLIB_SRC_PATH := $(HWLIB_UEFI_PATH)$(PATHSEP)src

ifdef ComSpec
REMOVE_HWLIB_INCLUDE := if $(EXIST) $(HWLIB_INCLUDE_PATH) $(RMDIR) $(HWLIB_INCLUDE_PATH)
REMOVE_HWLIB_SRC     := if $(EXIST) $(HWLIB_SRC_PATH)     $(RMDIR) $(HWLIB_SRC_PATH)
else
REMOVE_HWLIB_INCLUDE := $(EXIST) $(HWLIB_INCLUDE_PATH) || $(RMDIR) $(HWLIB_INCLUDE_PATH)
REMOVE_HWLIB_SRC     := $(EXIST) $(HWLIB_SRC_PATH)     || $(RMDIR) $(HWLIB_SRC_PATH)
endif

COPY_HWLIB_FROM_SOCEDS := $(CPDIR) $(HWLIB_SOCEDS_PATH) $(HWLIB_UEFI_PATH)

#-----------------------------------------------------------------------------
# Default Behavior when just type "make"
#-----------------------------------------------------------------------------
.PHONY: default
default: build_setup build_tool build_firmware build_mkpimage build_ds_script build_done

#-----------------------------------------------------------------------------
# when user type "make fast"
#-----------------------------------------------------------------------------
.PHONY: fast
fast: build_setup build_firmware build_mkpimage build_ds_script build_done

#-----------------------------------------------------------------------------
# when user type "make clean" or "make mrproper"
#-----------------------------------------------------------------------------
.PHONY: clean
clean: build_clean

.PHONY: mrproper
mrproper: build_clean

#-----------------------------------------------------------------------------
# when user type "make tool" or "make tools"
#-----------------------------------------------------------------------------
.PHONY: tool
tool: build_tool

.PHONY: tools
tools: build_tool

#-----------------------------------------------------------------------------
# when user type "make mkpimage COMPILER=armcc" or "make mkpimage COMPILER=gcc"
#-----------------------------------------------------------------------------
.PHONY: mkpimage
mkpimage: build_mkpimage build_ds_script

#-----------------------------------------------------------------------------
# when user type "make program_qspi" or "make program_nand"
#-----------------------------------------------------------------------------
.PHONY: program_qspi
program_qspi: program_flash_using_quartus_hps

.PHONY: program_nand
program_nand: program_flash_using_quartus_hps

#-----------------------------------------------------------------------------
# when user type "make program_qspi_256" or "make program_nand_256"
#-----------------------------------------------------------------------------
.PHONY: program_qspi_256
program_qspi_256: program_flash_using_quartus_hps_256

.PHONY: program_nand_256
program_nand_256: program_flash_using_quartus_hps_256

#-----------------------------------------------------------------------------
# when user type "make app"
#-----------------------------------------------------------------------------
.PHONY: app
app: build_setup_app build_tool build_firmware build_app_post build_done


#-----------------------------------------------------------------------------
# when user type "make help"
#-----------------------------------------------------------------------------
.PHONY: help
help:
	@$(ECHO_NEWLINE)
	@$(ECHO_START) +---------------------------------------------------------+$(ECHO_END)
	@$(ECHO_START) +   Makefile to build Altera SoC UEFI Firmware   +$(ECHO_END)
	@$(ECHO_START) +---------------------------------------------------------+$(ECHO_END)
	@$(ECHO_NEWLINE)
	@$(ECHO_START) USAGE:$(ECHO_END)
	@$(ECHO_NEWLINE)
	@$(ECHO_START)     make DEVICE=[device type]   COMPILER=[compiler type]   HANDOFF_DTB=[file path]$(ECHO_END)
	@$(ECHO_START)     make clean$(ECHO_END)
	@$(ECHO_NEWLINE)
	@$(ECHO_START) OPTIONS:$(ECHO_END)
	@$(ECHO_NEWLINE)
	@$(ECHO_START)     DEVICE      - Device type$(ECHO_END)
	@$(ECHO_START)                   s10 for Stratix 10$(ECHO_END)
	@$(ECHO_START)     COMPILER    - Compiler selection$(ECHO_END)
	@$(ECHO_START)                   armcc for ARMCC$(ECHO_END)
	@$(ECHO_START)                   gcc for Linaro GCC$(ECHO_END)
	@$(ECHO_START)     HANDOFF_DTB - Handoff DTB file path$(ECHO_END)
	@$(ECHO_NEWLINE)
	@$(ECHO_START) ADVANCED OPTIONS:$(ECHO_END)
	@$(ECHO_NEWLINE)
	@$(ECHO_START)     app         - build UEFI application for SocFpga$(ECHO_END)
	@$(ECHO_START)     clean       - will delete the entire 'Build' folder and also clean the BaseTools$(ECHO_END)
	@$(ECHO_START)     fast        - skip building BaseTools$(ECHO_END)
	@$(ECHO_START)     HANDOFF_DTS = [file path]$(ECHO_END)
	@$(ECHO_START)                   Call DTC tool to convert the .DTS file to replace the default .DTB file$(ECHO_END)
	@$(ECHO_START)                   Note: This cannot be use if HANDOFF_DTB is already specify in the argument$(ECHO_END)
	@$(ECHO_START)     mkpimage    - mkpimage $(PEI_FD_FILENAME) and then copy it to $(PEI_FINAL_ROM)$(ECHO_END)
	@$(ECHO_START)     program_qspi- program the PEI.ROM to QSPI using quartus_hps.exe in the path $(ECHO_END)
	@$(ECHO_NEWLINE)
	@$(ECHO_START) DESCRIPTIONS:$(ECHO_END)
	@$(ECHO_NEWLINE)
	@$(ECHO_START)     make - just typing 'make' is equal to typing: 'make DEVICE=$(DEFAULT_MAKE_DEVICE) COMPILER=$(DEFAULT_MAKE_COMPILER)'$(ECHO_END)
	@$(ECHO_START)     make fast - like make but skip building BaseTools$(ECHO_END)
	@$(ECHO_NEWLINE)
	@$(ECHO_START) EXAMPLES:$(ECHO_END)
	@$(ECHO_NEWLINE)
	@$(ECHO_START)     make$(ECHO_END)
	@$(ECHO_START)     make DEVICE=s10 COMPILER=gcc$(ECHO_END)
	@$(ECHO_START)     make DEVICE=s10 COMPILER=armcc$(ECHO_END)
	@$(ECHO_START)     make DEVICE=s10 COMPILER=armcc HANDOFF_DTB=AlteraPlatformPkg$(PATHSEP)Stratix10SoCPkg$(PATHSEP)Stratix10SoCPkgNand.dtb$(ECHO_END)
	@$(ECHO_START)     make DEVICE=s10 COMPILER=armcc HANDOFF_DTS=AlteraPlatformPkg$(PATHSEP)Stratix10SoCPkg$(PATHSEP)Stratix10SoCPkgNand.dts$(ECHO_END)
	@$(ECHO_START)     make fast DEVICE=s10 COMPILER=gcc$(ECHO_END)
	@$(ECHO_START)     make fast DEVICE=s10 COMPILER=gcc HANDOFF_DTB=s10_soc_devkit_ghrd/software/bootloader/devicetree.dtb$(ECHO_END)
	@$(ECHO_START)     make fast DEVICE=s10 COMPILER=gcc HANDOFF_DTS=s10_soc_devkit_ghrd/software/bootloader/devicetree.dts$(ECHO_END)
	@$(ECHO_START)     make mkpimage COMPILER=gcc$(ECHO_END)
	@$(ECHO_START)     make app$(ECHO_END)



#-----------------------------------------------------------------------------
# Shell Scripts: build_clean
#-----------------------------------------------------------------------------
build_clean:
	@$(ECHO_START) -------------------- $(ECHO_END)
	@$(ECHO_START) Cleaning BaseTools   $(ECHO_END)
	@$(ECHO_START) -------------------- $(ECHO_END)
	cd BaseTools $(NEXT_CMD) make clean
	@$(ECHO_START) ---------------------- $(ECHO_END)
	@$(ECHO_START) Removing Build folder  $(ECHO_END)
	@$(ECHO_START) ---------------------- $(ECHO_END)
	$(RMDIR) Build
	@$(ECHO_START) ---------------------- $(ECHO_END)
	@$(ECHO_START) Removing Conf/.cache/  $(ECHO_END)
	@$(ECHO_START) ---------------------- $(ECHO_END)
	$(RMDIR) Conf$(PATHSEP).cache$(PATHSEP)

#-----------------------------------------------------------------------------
# Shell Scripts: build_setup
#-----------------------------------------------------------------------------
build_setup:
	@$(ECHO_START) -----------------------------$(ECHO_END)
	@$(ECHO_START) Setting up build environemnt $(ECHO_END)
	@$(ECHO_START) -----------------------------$(ECHO_END)
	@$(ECHO_START) DEVICE         : $(DEVICE)$(device)$(D)$(d)$(ECHO_END)
	@$(ECHO_START) COMPILER       : $(COMPILER)$(compiler)$(C)$(c)$(ECHO_END)
	@$(ECHO_START) EDK2_ARCH      : $(EDK2_ARCH)$(ECHO_END)
	@$(ECHO_START) EDK2_DSC       : $(EDK2_DSC)$(ECHO_END)
	@$(ECHO_START) EDK2_BUILD     : $(EDK2_BUILD)$(ECHO_END)
	@$(ECHO_START) EDK2_MACROS    : $(EDK2_MACROS)$(ECHO_END)
	@$(ECHO_START) EDK2_TOOLCHAIN : $(EDK2_TOOLCHAIN)$(ECHO_END)
	@$(CMD_ECHO_HANDOFF_DTS)
	@$(CMD_ECHO_HANDOFF_DTB)
	$(CMD_COMPILE_HANDOFF_DTS)
	$(CMD_UPDATE_HANDOFF_DTB)

#-----------------------------------------------------------------------------
# Shell Scripts: build_setup
#-----------------------------------------------------------------------------
build_setup_app:
	@$(ECHO_START) -----------------------------$(ECHO_END)
	@$(ECHO_START) Setting up build environemnt $(ECHO_END)
	@$(ECHO_START) -----------------------------$(ECHO_END)
	@$(ECHO_START) DEVICE         : $(DEVICE)$(device)$(D)$(d)$(ECHO_END)
	@$(ECHO_START) COMPILER       : $(COMPILER)$(compiler)$(C)$(c)$(ECHO_END)
	@$(ECHO_START) EDK2_ARCH      : $(EDK2_ARCH)$(ECHO_END)
	@$(ECHO_START) EDK2_DSC       : $(EDK2_DSC)$(ECHO_END)
	@$(ECHO_START) EDK2_BUILD     : $(EDK2_BUILD)$(ECHO_END)
	@$(ECHO_START) EDK2_MACROS    : $(EDK2_MACROS)$(ECHO_END)
	@$(ECHO_START) EDK2_TOOLCHAIN : $(EDK2_TOOLCHAIN)$(ECHO_END)
	$(REMOVE_HWLIB_INCLUDE)
	$(REMOVE_HWLIB_SRC)

#-----------------------------------------------------------------------------
# Shell Scripts: build_tool
#-----------------------------------------------------------------------------
build_tool:
	$(MAKE_BUILD_TOOL)


#-----------------------------------------------------------------------------
# Shell Scripts: build_firmware
#-----------------------------------------------------------------------------
build_firmware:
	@$(ECHO_START) "--------------------"$(ECHO_END)
	@$(ECHO_START) "Compiling Firmware :"$(ECHO_END)
	@$(ECHO_START) "--------------------"$(ECHO_END)
	$(MAKE) -f ArmPlatformPkg$(PATHSEP)Scripts$(PATHSEP)Makefile

#-----------------------------------------------------------------------------
# Shell Scripts: build_mkpimage
#-----------------------------------------------------------------------------
build_mkpimage:
	@$(CP) $(DXE_FD_FILENAME_FULLPATH) $(DXE_FINAL_ROM)
	@$(CP) $(PEI_FD_FILENAME_FULLPATH) $(PEI_FINAL_ROM)

#-----------------------------------------------------------------------------
# Shell Scripts: build_ds_script
#-----------------------------------------------------------------------------
build_ds_script:
	@$(ECHO_START) Creating DS-5 script : $(DS_SCRIPT)$(ECHO_END)
	@$(CP) $(DS5_SCRIPT_PATH) $(DS_SCRIPT)
	@sed -i "s%$(SNR_ENTRY_ArmPlatformSec)%$(ENTRY_ArmPlatformSec)%g" $(DS_SCRIPT)
	@sed -i "s%$(SNR_ENTRY_ArmPlatformPrePeiCore)%$(ENTRY_ArmPlatformPrePeiCore)%g" $(DS_SCRIPT)
	@sed -i "s%$(SNR_ENTRY_AlteraSocFpgaPeiMain)%$(ENTRY_AlteraSocFpgaPeiMain)%g" $(DS_SCRIPT)
	@sed -i "s%$(SNR_FILE_ArmPlatformSec)%$(FILE_ArmPlatformSec)%g" $(DS_SCRIPT)
	@sed -i "s%$(SNR_FILE_ArmPlatformPrePeiCore)%$(FILE_ArmPlatformPrePeiCore)%g" $(DS_SCRIPT)
	@sed -i "s%$(SNR_FILE_AlteraSocFpgaPeiMain)%$(FILE_AlteraSocFpgaPeiMain)%g" $(DS_SCRIPT)
	@sed -i "s%$(SNR_BUILD_PEI)%$(FILE_BUILD_PEI)%g" $(DS_SCRIPT)
	@$(SED_DEL_TMP_FILE)

#-----------------------------------------------------------------------------
# Shell Scripts: build_app_post
#-----------------------------------------------------------------------------
build_app_post:
	@$(CP) $(EFIAPP_FILENAME_FULLPATH) Build$(PATHSEP)SocFpgaAppPkg$(PATHSEP)$(EDK2_BUILD)_$(EDK2_TOOLCHAIN)

#-----------------------------------------------------------------------------
# Shell Scripts: build_done
#-----------------------------------------------------------------------------
build_done:
	@$(ECHO_START) Build Done. $(ECHO_END)

#-----------------------------------------------------------------------------
# Shell Scripts: program_flash_using_quartus_hps
#-----------------------------------------------------------------------------
program_flash_using_quartus_hps:
	quartus_hps --help=c
	quartus_hps --help=o
	quartus_hps -c1 -oP $(PEI_FINAL_ROM) --addr=$(PEI_FLASH_ADDR)

program_flash_using_quartus_hps_256:
	quartus_hps --help=c
	quartus_hps --help=o
	quartus_hps -c1 -oP $(PEI_SMALLER_x1_ROM) --addr=$(PEI_FLASH_ADDR)

#-----------------------------------------------------------------------------
# Makefile testing
# for simple test, comment out these 2 line:
# make -C $(EDK_TOOLS_PATH)
# make -f ArmPlatformPkg/Scripts/Makefile
#-----------------------------------------------------------------------------

# list_of_makefile_self_test_command:
# make
# make DEVICE=s10
# make DEVICE=s10 COMPILER=GCC
# make DEVICE=S10 COMPILER=GCC
# make device=s10
# make device=s10 COMPILER=GCC
# make device=S10 COMPILER=GCC
# make DEVICE=invalid COMPILER=invalid
# make device=invalid compiler=invalid

