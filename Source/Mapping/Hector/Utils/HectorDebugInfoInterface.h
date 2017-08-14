#ifndef _HECTOR_DEBUG_INFO_INTERFACE_H_
#define _HECTOR_DEBUG_INFO_INTERFACE_H_

class HectorDebugInfoInterface
{
public:
  
  virtual void
  sendAndResetData () = 0;
  virtual void
  addHessianMatrix (const Eigen::Matrix3f& hessian) = 0;
  virtual void
  addPoseLikelihood (float lh) = 0;
};

#endif
