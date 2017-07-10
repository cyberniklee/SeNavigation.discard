/*
 * SimulateController.cpp
 *
 *  Created on: Jul 4, 2017
 *      Author: cybernik
 */

#include "SimulateController.h"
#include <Time/Utils.h>
#include <Service/ServiceType/RequestOdometry.h>
#include <Service/ServiceType/ResponseOdometry.h>
#include <Service/ServiceType/RequestTransform.h>
#include <Service/ServiceType/ResponseTransform.h>
#include <Console/Console.h>
#include <Service/Service.h>

namespace NS_Controller
{
  
  //SimulateController::SimulateController ()
  //{
  // TODO Auto-generated constructor stub
  
  //}
  
  //SimulateController::~SimulateController ()
  //{
  // TODO Auto-generated destructor stub
  //}
  
  void
  SimulateController::setBaseTicksPerMeter (double ticksPerMeter)
  {
    fBaseTicksPerMeter = ticksPerMeter;
    if (fBaseTicksPerMeter != 0)
      fTickDistance = 1 / fBaseTicksPerMeter;
    else fTickDistance = 0;
  }
  
  void
  SimulateController::setBaseWheelTrack (double wheelTrack)
  {
    fBaseWheelTrack = wheelTrack;
    if (wheelTrack == 0)
      return;
    fReciprocalOfWheelTrack = 1 / fBaseWheelTrack;
  }
  
  void
  SimulateController::setBaseWheelDiameter (double wheelDiameter)
  {
    fBaseWheelDiameter = wheelDiameter;
  }
  
  void
  SimulateController::setBaseEncoderResolution (double encoderResolution)
  {
    fBaseEncoderResolution = encoderResolution;
  }
  
  void
  SimulateController::setBaseGearReduction (double gearReduction)
  {
    fBaseGearReduction = gearReduction;
  }
  
  void
  SimulateController::setAccelLimit (double accelLimit)
  {
    fAccelLimit = accelLimit;
  }
  
  void
  SimulateController::setLinearVel (double linearVel)
  {
    tNewVel.fLinear = linearVel;
  }
  
  void
  SimulateController::setAngularVel (double angularVel)
  {
    tNewVel.fAngular = angularVel;
  }
  
  double
  SimulateController::getLinearVel (void)
  {
    return tCurrentOdometry.linearVel;
  }
  
  double
  SimulateController::getAngularVel (void)
  {
    return tCurrentOdometry.angularVel;
  }
  
  double
  SimulateController::getX (void)
  {
    return tCurrentOdometry.fX;
  }
  
  double
  SimulateController::getY (void)
  {
    return tCurrentOdometry.fY;
  }
  
  double
  SimulateController::getTheta (void)
  {
    return tCurrentOdometry.fTheta;
  }
  
  void
  SimulateController::SeUserMotionCalcRollSpeed (SeUserVelocity tVel,
                                                 short *tLeftSpeed,
                                                 short *tRightSpeed)
  {
    double fLeftDist, fRightDist;
    
    //double fDeltaDist = (tVel.fAngular*fBaseWheelTrack*fBaseGearReduction)/2.0f;
    //double fVelScale = (fBaseTicksPerMeter/nCntlDuration);
    double fDeltaDist = (tVel.fAngular * fBaseWheelTrack) / 2;
    double fVelScale = fBaseTicksPerMeter * nCntlDuration / 1000;
    
    if (tVel.fLinear == 0)
    {
      fRightDist = fDeltaDist;
      fLeftDist = -fRightDist;
    }
    else if (tVel.fAngular == 0)
    {
      fLeftDist = fRightDist = tVel.fLinear;
    }
    else
    {
      fLeftDist = tVel.fLinear - fDeltaDist;
      fRightDist = tVel.fLinear + fDeltaDist;
    }
    
    *tLeftSpeed = (short) (fLeftDist * fVelScale);
    *tRightSpeed = (short) (fRightDist * fVelScale);
    
  }
  
  SeUserOdometry
  SimulateController::SeUserOdometryCalculate (int nLeftEncoderDelta,
                                               int nRightEncoderDelta,
                                               unsigned int nDeltaTime)
  {
    
    double fDeltaRight, fDeltaLeft;
    double fDistAve;
    double fDeltaTheta;
    double fDeltaX, fDeltaY;
    double fDX, fDY;
    
    fDeltaLeft = nLeftEncoderDelta * fTickDistance;
    fDeltaRight = nRightEncoderDelta * fTickDistance;
    
    fDistAve = (fDeltaLeft + fDeltaRight) * 0.5f;
    
    fDeltaTheta = (fDeltaRight - fDeltaLeft) * fReciprocalOfWheelTrack;
    
    fDeltaX = cos (fDeltaTheta) * fDistAve;
    fDeltaY = -sin (fDeltaTheta) * fDistAve;
    
    double fThetaSinVal = sin (tCurrentOdometry.fTheta);
    double fThetaCosVal = cos (tCurrentOdometry.fTheta);
    
    fDX = fThetaCosVal * fDeltaX - fThetaSinVal * fDeltaY;
    fDY = fThetaSinVal * fDeltaX + fThetaCosVal * fDeltaY;
    
    tCurrentOdometry.fX += fDX;
    tCurrentOdometry.fY += fDY;
    tCurrentOdometry.fTheta += fDeltaTheta;
    
    //NS_NaviCommon::console.message ("DX=%f,DY=%f, DeltaTheta=%f, fDistAve=%f", fDX,fDY,fDeltaTheta, fDistAve);
    //double fReciprocalOfDeltaTime = (1.0f / nDeltaTime);
    double fReciprocalOfDeltaTime = 1000 / nDeltaTime;
    
    tCurrentOdometry.linearVel = (fDistAve * fReciprocalOfDeltaTime);
    tCurrentOdometry.angularVel = (fDeltaTheta * fReciprocalOfDeltaTime);
    
    return tCurrentOdometry;
  }
  
  void
  SimulateController::controllerPolling (void)
  {
    int nLeftCount = tLeftRollInfo.nEncode;
    int nRightCount = tRightRollInfo.nEncode;
    
    short fLeftDestVel = 0;
    short fRightDestVel = 0;
    SeUserOdometry tCurOdom;
    unsigned long tCurTime;
    unsigned long deltaTime;
    
    tCurTime = NS_NaviCommon::getMs ();
    
    if (fJiffies + nCntlDuration > tCurTime)
      return;
    //deltaTime = tCurTime-fJiffies;
    fJiffies = tCurTime;
    //NS_NaviCommon::console.message ("simulate controller time:%d", tCurTime);
    
    tCurVel = tNewVel;
    
    tUserLeftEncoder = nLeftCount;
    tUserRightEncoder = nRightCount;
    //NS_NaviCommon::console.message ("left count=%d, right count=%d", nLeftCount, nRightCount);
    if (nLeftCount != 0 && nRightCount != 0)
    {
      tCurOdom = SeUserOdometryCalculate (nLeftCount, nRightCount,
                                          nCntlDuration);
    }
    
    //NS_NaviCommon::console.message ("X=%f,Y=%f,Theta=%f,linearVel=%f,angularVel=%f", getX(),getY(),getTheta(),getLinearVel(),getAngularVel());
    SeUserMotionCalcRollSpeed (tCurVel, (short *) &fLeftDestVel,
                               (short *) &fRightDestVel);
    /*
     if(tLeftRollInfo.nSpeed < fLeftDestVel)
     {
     tLeftRollInfo.nSpeed += fAccelLimit;
     if(tLeftRollInfo.nSpeed > fLeftDestVel)
     {
     tLeftRollInfo.nSpeed = fLeftDestVel;
     }
     }else{
     tLeftRollInfo.nSpeed -= fAccelLimit;
     if(tLeftRollInfo.nSpeed < fLeftDestVel)
     {
     tLeftRollInfo.nSpeed = fLeftDestVel;
     }
     }

     if(tRightRollInfo.nSpeed < fRightDestVel)
     {
     tRightRollInfo.nSpeed += fAccelLimit;
     if(tRightRollInfo.nSpeed > fRightDestVel)
     {
     tRightRollInfo.nSpeed = fRightDestVel;
     }
     }else{
     tRightRollInfo.nSpeed -= fAccelLimit;
     if(tRightRollInfo.nSpeed < fRightDestVel)
     {
     tRightRollInfo.nSpeed = fRightDestVel;
     }
     }
     */
    tLeftRollInfo.nSpeed = fLeftDestVel;
    tRightRollInfo.nSpeed = fRightDestVel;
    if (tLeftRollInfo.nSpeed > 0)
    {
      tLeftRollInfo.tState = SE_USER_ROLL_FORWARD;
    }
    else if (tLeftRollInfo.nSpeed < 0)
    {
      tLeftRollInfo.nSpeed = -tLeftRollInfo.nSpeed;
      tLeftRollInfo.tState = SE_USER_ROLL_BACKWARD;
    }
    else
    {
      tLeftRollInfo.tState = SE_USER_ROLL_STOP;
    }
    
    if (tRightRollInfo.nSpeed > 0)
    {
      tRightRollInfo.tState = SE_USER_ROLL_FORWARD;
    }
    else if (tRightRollInfo.nSpeed < 0)
    {
      tRightRollInfo.nSpeed = -tRightRollInfo.nSpeed;
      tRightRollInfo.tState = SE_USER_ROLL_BACKWARD;
    }
    else
    {
      tRightRollInfo.tState = SE_USER_ROLL_STOP;
    }
    
    tLeftRollInfo.nEncode = 0;
    tRightRollInfo.nEncode = 0;
    
    tLeftRollInfo.nEncode = tLeftRollInfo.nSpeed;
    tRightRollInfo.nEncode = tRightRollInfo.nSpeed;
  }
  
  void
  SimulateController::motionCallback (void)
  {
    NS_NaviCommon::Rate rate (1000 / nCntlDuration);
    while (1)
    {
      controllerPolling ();
      rate.sleep ();
      if (running != true)
        break;
    }
  }
  
  void
  SimulateController::initialize ()
  {
    
    tLeftRollInfo.nEncode = 0;
    tLeftRollInfo.nSpeed = 0;
    tLeftRollInfo.tState = SE_USER_ROLL_STOP;
    tRightRollInfo.nEncode = 0;
    tRightRollInfo.nSpeed = 0;
    tRightRollInfo.tState = SE_USER_ROLL_STOP;
    
    tCurrentOdometry.angularVel = 0;
    tCurrentOdometry.fTheta = 0;
    tCurrentOdometry.fX = 0;
    tCurrentOdometry.fY = 0;
    tCurrentOdometry.linearVel = 0;
    
    tCurVel.fAngular = 0;
    tCurVel.fLinear = 0;
    tNewVel.fAngular = 0;
    tNewVel.fLinear = 0;
    
    fBaseWheelTrack = 0.265f;
    fBaseWheelDiameter = 0.068f;
    fBaseEncoderResolution = 16;
    fBaseGearReduction = 62;
    
    fBaseTicksPerMeter = (fBaseEncoderResolution * fBaseGearReduction)
        / (fBaseWheelDiameter * M_PI);
    nCntlDuration = 33;
    
    fAccelLimit = 1.0f;
    
    if (fBaseWheelDiameter != 0)
      fReciprocalOfWheelTrack = 1 / fBaseWheelTrack;
    else fReciprocalOfWheelTrack = 0;
    
    setLinearVel (0.5);
    setAngularVel (0.1);
    
    if (fBaseTicksPerMeter != 0)
      fTickDistance = 1 / fBaseTicksPerMeter;
    else fTickDistance = 0;
    
    fUserLeftSpeed = 0;
    fUserRightSpeed = 0;
    
    tUserLeftEncoder = 0;
    tUserRightEncoder = 0;
    
    fJiffies = NS_NaviCommon::getMs ();
    
    NS_NaviCommon::console.message ("simulate controller is initialized...");
  }
  
  void
  SimulateController::run ()
  {
    running = true;
    NS_NaviCommon::console.message ("simulate controller is running...");
    simulate_controller_thread = boost::thread (
        boost::bind (&SimulateController::motionCallback, this));
    
  }
  
  void
  SimulateController::quit ()
  {
    NS_NaviCommon::console.message ("simulate controller is quitting!");
    running = false;
    simulate_controller_thread.join ();
  }

}
