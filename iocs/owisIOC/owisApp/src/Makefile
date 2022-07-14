TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# The following are used for debugging messages.
#!USR_CXXFLAGS += -DDEBUG

#=============================
# Build the IOC application

PROD_IOC = owis
# owis.dbd will be created and installed
DBD += owis.dbd

# owis.dbd will be made up from these files:
owis_DBD += base.dbd

# Include dbd files from all support applications:
#ifdef ASYN
owis_DBD += asyn.dbd
owis_DBD += drvAsynSerialPort.dbd
#endif
owis_DBD += motorSupport.dbd
owis_DBD += OwisSupport.dbd

# Add all the support libraries needed by this IOC
owis_LIBS += Owis
owis_LIBS += motor
#ifdef ASYN
owis_LIBS += asyn
#endif

# owis_registerRecordDeviceDriver.cpp derives from owis.dbd
owis_SRCS += owis_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
owis_SRCS_DEFAULT += owisMain.cpp
owis_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#owis_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
owis_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

