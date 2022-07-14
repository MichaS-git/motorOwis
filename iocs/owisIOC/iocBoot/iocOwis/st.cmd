#!../../bin/linux-x86_64/owis

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/owis.dbd"
owis_registerRecordDeviceDriver pdbbase

cd "${TOP}/iocBoot/${IOC}"

## motorUtil (allstop & alldone)
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=owis:")

## 
< motor.cmd.ps10

iocInit

## motorUtil (allstop & alldone)
motorUtilInit("owis:")

# Boot complete
