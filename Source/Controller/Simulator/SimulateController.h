/*
 * SimulateController.h
 *
 *  Created on: Jul 4, 2017
 *      Author: cybernik
 */

#ifndef CONTROLLER_SIMULATECONTROLLER_SIMULATECONTROLLER_H_
#define CONTROLLER_SIMULATECONTROLLER_SIMULATECONTROLLER_H_

#include <DataSet/Dispitcher.h>
#include <Service/ServiceType/RequestBase.h>
#include <Service/ServiceType/ResponseBase.h>
#include <boost/thread/thread.hpp>
#include <DataSet/DataType/Odometry.h>
#include <boost/thread/thread.hpp>

#include "../../Application/Application.h"

namespace NS_Controller
{
  
  typedef enum
  {
    SE_USER_ROLL_STOP = 0, SE_USER_ROLL_FORWARD, SE_USER_ROLL_BACKWARD,
  } SeUserRollStates;
  
  typedef struct
  {
    unsigned short nEncode;
    unsigned short nSpeed;
    SeUserRollStates tState;
  } SeUserRollInfo;
  
  typedef struct
  {
    double fX;
    double fY;
    double fTheta;
    double angularVel;
    double linearVel;
  } SeUserOdometry;
  
  typedef struct
  {
    double fLinear;
    double fAngular;
  } SeUserVelocity;
  
  class SimulateController
  {
  public:
    SimulateController ()
    {
    }
    ;

    ~SimulateController ()
    {
    }
    ;

  private:
    SeUserRollInfo tLeftRollInfo, tRightRollInfo;
    SeUserOdometry tCurrentOdometry;
    SeUserVelocity tCurVel, tNewVel;
    NS_DataType::Odometry current_odometry;

    boost::mutex odom_lock;

    double fBaseTicksPerMeter;
    double fBaseWheelTrack;
    double fBaseWheelDiameter;
    double fBaseEncoderResolution;
    double fBaseGearReduction;

    double fTickDistance;
    double fReciprocalOfWheelTrack;

    int nCntlDuration;

    double fAccelLimit;

    short fUserLeftSpeed;
    short fUserRightSpeed;

    short tUserLeftEncoder;
    short tUserRightEncoder;

    int period; //ms
    double fJiffies;

    boost::thread simulate_controller_thread;

    void
    controllerPolling (void);
    void
    SeUserMotionCalcRollSpeed (SeUserVelocity tVel, short *tLeftSpeed,
                               short *tRightSpeed);
    SeUserOdometry
    SeUserOdometryCalculate (int nLeftEncoderDelta, int nRightEncoderDelta,
                             unsigned int nDeltaTime);
    //void SeUserMotionCalcRollSpeed(SeUserVelocity tVel, short *tLeftSpeed, short *tRightSpeed);
    
    bool running;
  public:
    
    void
    setBaseTicksPerMeter (double ticksPerMeter);
    void
    setBaseWheelTrack (double wheelTrack);
    void
    setBaseWheelDiameter (double wheelDiameter);
    void
    setBaseEncoderResolution (double encoderResolution);
    void
    setBaseGearReduction (double gearReduction);
    void
    setAccelLimit (double accelLimit);
    void
    setDuration (int duration);
    void
    setLinearVel (double angularVel);
    void
    setAngularVel (double linearVel);

    double
    getAngularVel (void);
    double
    getLinearVel (void);
    double
    getX (void);
    double
    getY (void);
    double
    getTheta (void);

    void
    motionCallback (void);

  public:
    void
    initialize ();
    void
    run ();
    void
    quit ();
  };
}

#endif /* CONTROLLER_SIMULATECONTROLLER_SIMULATECONTROLLER_H_ */
