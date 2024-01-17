# Project Name
TARGET = chnl

# Sources
CPP_SOURCES = chnl.cpp

# APP_TYPE = BOOT_SRAM

# Library Locations
LIBDAISY_DIR = ./libDaisy/
DAISYSP_DIR = ./DaisySP/

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile
