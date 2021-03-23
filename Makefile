#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := mpu9250

EXTRA_COMPONENT_DIRS := components/ahrs
						components/mpu9250
						components/wifi
						components/ble

include $(IDF_PATH)/make/project.mk
