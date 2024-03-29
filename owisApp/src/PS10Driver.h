/*
FILENAME...   PS10Driver.h
USAGE...      Motor driver support for the Owis PS 10 controller.

Michael Sintschuk
July 11, 2022

*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"

// No controller-specific parameters yet
#define NUM_PS10_PARAMS 0

class PS10Axis : public asynMotorAxis
{
public:
    /* These are the methods we override from the base class */
    PS10Axis(class PS10Controller *pC, int axis, int slaveID, const char actLimit);
    void report(FILE *fp, int level);
    asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
    asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
    asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
    asynStatus stop(double acceleration);
    asynStatus poll(bool *moving);

private:
    PS10Controller *pC_;          /**< Pointer to the asynMotorController to which this axis belongs.
                                   *   Abbreviated because it is used very frequently */
    int axisIndex_;    /* Numbered from 1 */
    int slaveID_;      /* Slave-ID for CAN-Daysy-Chain, 00 to 99*/
    int motorType_;    /* Motor Type: 0 = DC brush; 1 = stepper motor open-loop */
    bool velocityMode;
    asynStatus sendAccelAndVelocity(double accel, double velocity);

    friend class PS10Controller;
};

class PS10Controller : public asynMotorController
{
public:
    PS10Controller(const char *portName, const char *PS10PortName, int numAxes, const char *slaveIDs, const char *actLimits, double movingPollPeriod, double idlePollPeriod);

    /* These are the methods that we override from asynMotorDriver */
    void report(FILE *fp, int level);
    asynStatus writeReadController();
    asynStatus writeReadController(const char *output, char *input, size_t maxChars, size_t *nread, double timeout);
    asynStatus writeController();
    asynStatus writeController(const char *output, double timeout);

    friend class PS10Axis;
};
