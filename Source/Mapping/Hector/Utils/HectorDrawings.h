#ifndef _HECTOR_DRAWINGS_H_
#define _HECTOR_DRAWINGS_H_

#include "DrawInterface.h"
#include "UtilFunctions.h"

#include <Eigen/Dense>

#include <DataSet/DataType/Pose.h>
#include <DataSet/DataType/Vector3.h>

#include <Console/Console.h>

class HectorDrawings: public DrawInterface
{
public:
  
  HectorDrawings ()
  {
    this->setScale (1.0);
    this->setColor (1.0, 1.0, 1.0);
  }
  ;

  virtual void
  drawPoint (const Eigen::Vector2f& pointWorldFrame)
  {
    double x = pointWorldFrame.x ();
    double y = pointWorldFrame.y ();
    
    NS_NaviCommon::console.debug ("Draw Point (%f, %f)", x, y);
  }
  
  virtual void
  drawArrow (const Eigen::Vector3f& poseWorld)
  {
    NS_DataType::Pose p;
    
    p.position.x = poseWorld.x ();
    p.position.y = poseWorld.y ();
    
    p.orientation.w = cos (poseWorld.z () * 0.5f);
    p.orientation.z = sin (poseWorld.z () * 0.5f);
    
    NS_NaviCommon::console.debug (
        "Draw Pose position->(%f, %f) orientation->(%f, %f)", p.position.x,
        p.position.y, p.orientation.w, p.orientation.z);
  }
  
  virtual void
  drawCovariance (const Eigen::Vector2f& mean, const Eigen::Matrix2f& covMatrix)
  {
    
    NS_DataType::Pose p;
    
    p.position.x = mean[0];
    p.position.y = mean[1];
    
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig (covMatrix);
    
    const Eigen::Vector2f& eigValues (eig.eigenvalues ());
    const Eigen::Matrix2f& eigVectors (eig.eigenvectors ());
    
    float angle = (atan2 (eigVectors (1, 0), eigVectors (0, 0)));
    
    double lengthMajor = sqrt (eigValues[0]);
    double lengthMinor = sqrt (eigValues[1]);
    
    NS_DataType::Vector3 v;
    
    v.x = lengthMajor;
    v.y = lengthMinor;
    v.z = 0.001;
    
    p.orientation.w = cos (angle * 0.5);
    p.orientation.z = sin (angle * 0.5);
    
    //drawLine(Eigen::Vector3f(0,0,0), Eigen::Vector3f(lengthMajor ,0,0));
    //drawLine(Eigen::Vector3f(0,0,0), Eigen::Vector3f(0,lengthMinor,0));
    
    //glScalef(lengthMajor, lengthMinor, 0);
    //glCallList(dlCircle);
    //this->popCS();
  }
  
  virtual void
  setScale (double scale)
  {
    
  }
  
  virtual void
  setColor (double r, double g, double b, double a = 1.0)
  {
    
  }
  
  virtual void
  sendAndResetData ()
  {
    
  }
  
  void
  setTime (const NS_NaviCommon::Time& time)
  {
    
  }
};

#endif
