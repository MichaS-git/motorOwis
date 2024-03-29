/*
FILENAME... PS10Driver.cpp
USAGE...    Motor driver support for the Owis PS 10 controller.

Michael Sintschuk
July 11, 2022

*/


#include <stdio.h>
#include <string.h>
#include <sstream>
#include <stdlib.h>
#include <math.h>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include "PS10Driver.h"
#include <epicsExport.h>

#define NINT(f) (int)((f)>0 ? (f)+0.5 : (f)-0.5)

/** Creates a new PS10Controller object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] PS10PortName      The name of the drvAsynSerialPort that was created previously to connect to the PS10 controller
  * \param[in] numAxes           The number of axes that this controller supports
  * \param[in] slaveIDs          List of SlaveIDs when using the PS 10-32 in CAN-Daysy-Chain, separated by coma
  * \param[in] actLimits         String for Limit-activation (y) and -deactivation (n), e.g. "ny" for two axes
  * \param[in] movingPollPeriod  The time between polls when any axis is moving
  * \param[in] idlePollPeriod    The time between polls when no axis is moving
  */
PS10Controller::PS10Controller(const char *portName, const char *PS10PortName, int numAxes, const char *slaveIDs, const char *actLimits,
                               double movingPollPeriod, double idlePollPeriod)
    :  asynMotorController(portName, numAxes, NUM_PS10_PARAMS,
                           0, // No additional interfaces beyond those in base class
                           0, // No additional callback interfaces beyond those in base class
                           ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                           1, // autoconnect
                           0, 0)  // Default priority and stack size
{
    int axis;
    int* slaveIDsArr = new int[numAxes];
    asynStatus status;
    PS10Axis *pAxis;
    static const char *functionName = "PS10Controller::PS10Controller";

    // convert the slaveIDs to an integer array
    std::string str = slaveIDs;
    std::stringstream text_stream(slaveIDs);
    std::string item;

    int i = 0;
    while (std::getline(text_stream, item, ','))
    {
        slaveIDsArr[i] = std::atoi(item.c_str());
        i++;
    }

    std::string limits = actLimits;

    /* Connect to PS10 controller */
    status = pasynOctetSyncIO->connect(PS10PortName, 0, &pasynUserController_, NULL);
    if (status)
    {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s: cannot connect to PS 10 controller\n",
                  functionName);
    }
    for (axis=0; axis<numAxes; axis++)
    {
        pAxis = new PS10Axis(this, axis, slaveIDsArr[axis], limits[axis]);
    }

    startPoller(movingPollPeriod, idlePollPeriod, 2);
}


/** Creates a new PS10Controller object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] PS10PortName      The name of the drvSerialPort that was created previously to connect to the PS10 controller
  * \param[in] numAxes           The number of axes that this controller supports
  * \param[in] slaveIDs          List of SlaveIDs when using the PS 10-32 in CAN-Daysy-Chain, separated by coma
  * \param[in] actLimits         String for Limit-activation (y) and -deactivation (n), e.g. "ny" for two axes
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving
  */
extern "C" int PS10CreateController(const char *portName, const char *PS10PortName, int numAxes, const char *slaveIDs, const char *actLimits,
                                    int movingPollPeriod, int idlePollPeriod)
{
    PS10Controller *pPS10Controller
        = new PS10Controller(portName, PS10PortName, numAxes, slaveIDs, actLimits, movingPollPeriod/1000., idlePollPeriod/1000.);
    pPS10Controller = NULL;
    return(asynSuccess);
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */
void PS10Controller::report(FILE *fp, int level)
{
    fprintf(fp, "PS 10 motor driver\n");
    fprintf(fp, "  port name=%s\n", this->portName);
    fprintf(fp, "  moving poll period=%f\n", movingPollPeriod_);
    fprintf(fp, "  idle poll period=%f\n", idlePollPeriod_);

    // Call the base class method
    asynMotorController::report(fp, level);
}

/** Writes a string to the controller and reads the response.
  * Calls writeReadController() with default locations of the input and output strings
  * and default timeout. */
asynStatus PS10Controller::writeReadController()
{
    size_t nread;
    return writeReadController(outString_, inString_, sizeof(inString_), &nread, DEFAULT_CONTROLLER_TIMEOUT);
}

/** Writes a string to the controller and reads a response.
  * \param[in] output Pointer to the output string.
  * \param[out] input Pointer to the input string location.
  * \param[in] maxChars Size of the input buffer.
  * \param[out] nread Number of characters read.
  * \param[out] timeout Timeout before returning an error.*/
asynStatus PS10Controller::writeReadController(const char *output, char *input,
        size_t maxChars, size_t *nread, double timeout)
{
    size_t nwrite;
    asynStatus status;
    int eomReason;
    // const char *functionName="writeReadController";

    status = pasynOctetSyncIO->writeRead(pasynUserController_, output,
                                         strlen(output), input, maxChars, timeout,
                                         &nwrite, nread, &eomReason);

     // We need to halt the communication because of the controller's command processing time, 20~40ms
    epicsThreadSleep(0.05);

    return status;
}

/** Writes a string to the controller.
  * Calls writeController() with a default location of the string to write and a default timeout. */
asynStatus PS10Controller::writeController()
{
    return writeController(outString_, DEFAULT_CONTROLLER_TIMEOUT);
}

/** Writes a string to the controller.
  * \param[in] output The string to be written.
  * \param[in] timeout Timeout before returning an error.*/
asynStatus PS10Controller::writeController(const char *output, double timeout)
{
    size_t nwrite;
    asynStatus status;
    // const char *functionName="writeController";

    status = pasynOctetSyncIO->write(pasynUserController_, output,
                                     strlen(output), timeout, &nwrite);

    // We need to halt the communication because of the controller's command processing time, 20~40ms
    epicsThreadSleep(0.05);

    return status ;
}

// These are the PS10Axis methods

/** Creates a new PS10Axis object.
  * \param[in] pC Pointer to the PS10Controller to which this axis belongs.
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * \param[in] slaveID when operating in daisy chain.
  * Initializes register numbers, etc.
  */
PS10Axis::PS10Axis(PS10Controller *pC, int axisNo, int slaveID, const char actLimit)
    : asynMotorAxis(pC, axisNo),
      pC_(pC),
      velocityMode(false)
{
    int errorFlag = 0;
    asynStatus status;
    static const char *functionName = "PS10Axis::PS10Axis";

    // controller axes are numbered from 1
    //axisIndex_ = axisNo + 1;

    // Slave-ID for CAN-Daysy-Chain, 00 to 99
    slaveID_ = slaveID;

    // Flush I/O in case there is lingering garbage
    pC_->writeReadController();

    // Read the status of this motor and initialize the axis if not initialized
    sprintf(pC_->outString_, "%02d?ASTAT", slaveID_);
    status = pC_->writeReadController();
    if (status != asynSuccess)
        errorFlag = 1;

    if (strncmp(pC_->inString_, "I", 1) == 0)
        sprintf(pC_->outString_, "%02dINIT1", slaveID_);
        status = pC_->writeController();
        if (status != asynSuccess)
            errorFlag = 1;

    // Read the version string and the motor type
    sprintf(pC_->outString_, "%02d?MOTYPE1", slaveID_);
    status = pC_->writeReadController();
    if (status != asynSuccess)
        errorFlag = 1;
    // Store motor type
    motorType_ = atoi(&pC_->inString_[1]);

    // Set limit switch mask, use MAXSTOP and MINSTOP if actLimit is 'y'
    if (actLimit == 'y')
    {
        sprintf(pC_->outString_, "%02dSMK1=9", slaveID_);
    }
    else
    {
        sprintf(pC_->outString_, "%02dSMK1=0", slaveID_);
    }
    status = pC_->writeController();
    if (status != asynSuccess)
        errorFlag = 1;

    // What should happen if the controller doesn't respond?
    // For now put the controller in an error state
    if (errorFlag == 1)
    {
        setIntegerParam(pC_->motorStatusProblem_, 1);
    }

    callParamCallbacks();
}

/** Reports on status of the axis
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * After printing device-specific information calls asynMotorAxis::report()
  */
void PS10Axis::report(FILE *fp, int level)
{
    if (level > 0)
    {
        //fprintf(fp, "  axis %d\n", axisNo_);
        fprintf(fp, "  slave ID=%d\n", slaveID_);
        fprintf(fp, "  motor type %d\n", motorType_);
    }

    // Call the base class method
    asynMotorAxis::report(fp, level);
}

asynStatus PS10Axis::sendAccelAndVelocity(double acceleration, double velocity)
{
    asynStatus status;
    // static const char *functionName = "PS10::sendAccelAndVelocity";

    // Send the velocity
    sprintf(pC_->outString_, "%02dPVEL1=%d", slaveID_, NINT(velocity));
    status = pC_->writeController();

    // Send the acceleration
    sprintf(pC_->outString_, "%02dACC1=%d", slaveID_, NINT(acceleration));
    status = pC_->writeController();

    return status;
}

asynStatus PS10Axis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
    asynStatus status;
    // static const char *functionName = "PS10Axis::move";

    status = sendAccelAndVelocity(acceleration, maxVelocity);

    velocityMode = false;
    if (relative)
    {
        sprintf(pC_->outString_, "%02dRELAT1", slaveID_);
    }
    else
    {
        sprintf(pC_->outString_, "%02dABSOL1", slaveID_);
    }
    status = pC_->writeController();

    sprintf(pC_->outString_, "%02dPSET1=%d", slaveID_, NINT(position));
    status = pC_->writeController();

    sprintf(pC_->outString_, "%02dPGO1", slaveID_);
    status = pC_->writeController();

    return status;
}

asynStatus PS10Axis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
    asynStatus status;
    // static const char *functionName = "PS10Axis::home";

    status = sendAccelAndVelocity(acceleration, maxVelocity);

    velocityMode = false;
    if (forwards)
    {
        sprintf(pC_->outString_, "%02dRMK1=8", slaveID_);
    }
    else
    {
        sprintf(pC_->outString_, "%02dRMK1=1", slaveID_);
    }
    status = pC_->writeController();

    // we use the mode 4 only so far
    sprintf(pC_->outString_, "%02dREF1=4", slaveID_);
    status = pC_->writeController();

    return status;
}

asynStatus PS10Axis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
    asynStatus status;
    static const char *functionName = "PS10Axis::moveVelocity";

    asynPrint(pasynUser_, ASYN_TRACE_FLOW,
              "%s: minVelocity=%f, maxVelocity=%f, acceleration=%f\n",
              functionName, minVelocity, maxVelocity, acceleration);

    //status = sendAccelAndVelocity(acceleration, maxVelocity);

    velocityMode = true;

    sprintf(pC_->outString_, "%02dVVEL1=%f", slaveID_, minVelocity);
    status = pC_->writeController();

    sprintf(pC_->outString_, "%02dVGO1", slaveID_);
    status = pC_->writeController();

    return status;
}

asynStatus PS10Axis::stop(double acceleration )
{
    asynStatus status;
    //static const char *functionName = "PS10Axis::stop";
    if(velocityMode)
        sprintf(pC_->outString_, "%02dVSTP1", slaveID_);
    else
        sprintf(pC_->outString_, "%02dSTOP1", slaveID_);

    status = pC_->writeController();
    velocityMode = false;
    return status;
}

/** Polls the axis.
  * This function reads the motor position and the moving status.
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */
asynStatus PS10Axis::poll(bool *moving)
{
    int done = 0;
    int limit = 0;
    int position;
    asynStatus comStatus;

    /*
    // Read out the message exit buffer
    sprintf(pC_->outString_, "%02d?MSG", slaveID_);
    comStatus = pC_->writeReadController();
    if (comStatus) goto skip;

    // Read out the error messages
    sprintf(pC_->outString_, "%02d?ERR", slaveID_);
    comStatus = pC_->writeReadController();
    if (comStatus) goto skip;

    // Read out the limit switch mask
    sprintf(pC_->outString_, "%02d?SMK1", slaveID_);
    comStatus = pC_->writeReadController();
    if (comStatus) goto skip;
    */

    // Read the current motor position
    sprintf(pC_->outString_, "%02d?CNT1", slaveID_);
    comStatus = pC_->writeReadController();
    if (comStatus) goto skip;
    // The response string is of the form "1000"
    sscanf(pC_->inString_, "%d", &position);
    setDoubleParam(pC_->motorPosition_, position);

    // Read the limit status
    sprintf(pC_->outString_, "%02d?ESTAT1", slaveID_);
    comStatus = pC_->writeReadController();
    if (comStatus) goto skip;
    // The response string is of the form "10101"
    sscanf(pC_->inString_, "%d", &limit);
    if (limit == 8)
    {
        setIntegerParam(pC_->motorStatusHighLimit_, 1);
    }
    if (limit == 1)
    {
        setIntegerParam(pC_->motorStatusLowLimit_, 1);
    }
    if (limit == 0)
    {
        setIntegerParam(pC_->motorStatusHighLimit_, 0);
        setIntegerParam(pC_->motorStatusLowLimit_, 0);
    }

    // Read the axis state inquiry, response in letters: IIORRTTJV
    sprintf(pC_->outString_, "%02d?ASTAT", slaveID_);
    comStatus = pC_->writeReadController();
    if (comStatus) goto skip;
    // R means axis initialized and ready
    if (strncmp(pC_->inString_, "R", 1) == 0)
    {
        done = 1;
    }
    // L means axis has been disabled after approaching a hardware limit switch
    if (strncmp(pC_->inString_, "L", 1) == 0)
    {
        // initialize the axis again and release the limit switch
        // afterwards the user should reference the axis manually
        sprintf(pC_->outString_, "%02dINIT1", slaveID_);
        comStatus = pC_->writeController();
        if (comStatus) goto skip;
        sprintf(pC_->outString_, "%02dEFREE1", slaveID_);
        comStatus = pC_->writeController();
        if (comStatus) goto skip;
    }
    setIntegerParam(pC_->motorStatusDone_, done);
    setIntegerParam(pC_->motorStatusMoving_, !done);
    *moving = done ? false:true;

skip:
    setIntegerParam(pC_->motorStatusProblem_, comStatus ? 1:0);
    callParamCallbacks();
    return comStatus ? asynError : asynSuccess;
}

/** Code for iocsh registration */
static const iocshArg PS10CreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg PS10CreateControllerArg1 = {"PS 10 port name", iocshArgString};
static const iocshArg PS10CreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg PS10CreateControllerArg3 = {"List with slaveIDs", iocshArgString};
static const iocshArg PS10CreateControllerArg4 = {"List for Limit-activation", iocshArgString};
static const iocshArg PS10CreateControllerArg5 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg PS10CreateControllerArg6 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const PS10CreateControllerArgs[] = {&PS10CreateControllerArg0,
                                                            &PS10CreateControllerArg1,
                                                            &PS10CreateControllerArg2,
                                                            &PS10CreateControllerArg3,
                                                            &PS10CreateControllerArg4,
                                                            &PS10CreateControllerArg5,
                                                            &PS10CreateControllerArg6
                                                           };
static const iocshFuncDef PS10CreateControllerDef = {"PS10CreateController", 7, PS10CreateControllerArgs};
static void PS10CreateContollerCallFunc(const iocshArgBuf *args)
{
    PS10CreateController(args[0].sval, args[1].sval, args[2].ival, args[3].sval, args[4].sval, args[5].ival, args[6].ival);
}

static void PS10Register(void)
{
    iocshRegister(&PS10CreateControllerDef, PS10CreateContollerCallFunc);
}

extern "C" {
    epicsExportRegistrar(PS10Register);
}
