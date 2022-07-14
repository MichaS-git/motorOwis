# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

# The following are used for debugging messages.
#!USR_CXXFLAGS += -DDEBUG

DBD += OwisSupport.dbd

LIBRARY_IOC = Owis

# Motion Controller PS 10
SRCS += PS10Driver.cpp

Owis_LIBS += motor asyn
Owis_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

