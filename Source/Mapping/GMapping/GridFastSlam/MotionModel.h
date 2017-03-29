#ifndef _MOTIONMODEL_H_
#define _MOTIONMODEL_H_

#include "../Utils/Point.h"
#include "../Utils/Stat.h"
#include "../Utils/MacroParams.h"

namespace NS_GMapping
{
  
  struct MotionModel
  {
    OrientedPoint
    drawFromMotion (const OrientedPoint& p, double linearMove,
                    double angularMove) const;
    OrientedPoint
    drawFromMotion (const OrientedPoint& p, const OrientedPoint& pnew,
                    const OrientedPoint& pold) const;
    Covariance3
    gaussianApproximation (const OrientedPoint& pnew,
                           const OrientedPoint& pold) const;
    double srr, str, srt, stt;
  };

}
;

#endif
