# Project Name
TARGET = DaisyTape

# Sources
CPP_SOURCES = $(wildcard src/*.cpp)

# Add include folder for headers
C_INCLUDES += -Iinclude

# Use C++17 (not needed for now)
# CPP_STANDARD = -std=c++17

# # Added this to load the program into SRAM instead of FLASH (speed slightly lower than internal flash but we have more space)
APP_TYPE = BOOT_SRAM

# Library Locations
LIBDAISY_DIR = ../libDaisy/
DAISYSP_DIR = ../DaisySP/

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile
