/*
 * RecoveryBehavior.h
 *
 *  Created on: 2016年12月28日
 *      Author: seeing
 */

#ifndef _RECOVERYBEHAVIOR_H_
#define _RECOVERYBEHAVIOR_H_

class RecoveryBehavior
{
public:
  RecoveryBehavior ()
  {
  }
  ;
  virtual
  ~RecoveryBehavior ()
  {
  }
  ;
public:
  void
  initialize ();

  virtual void
  onInitialize () = 0;

  virtual void
  run () = 0;
};

#endif /* NAVIGATION_PLANNER_BASE_RECOVERYBEHAVIOR_H_ */
