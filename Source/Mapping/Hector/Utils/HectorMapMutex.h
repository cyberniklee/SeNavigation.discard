#ifndef _HECTOR_MAP_MUTEX_
#define _HECTOR_MAP_MUTEX_

#include "MapLockerInterface.h"

#include <boost/thread/mutex.hpp>

class HectorMapMutex: public MapLockerInterface
{
public:
  virtual void
  lockMap ()
  {
    mapModifyMutex_.lock ();
  }
  
  virtual void
  unlockMap ()
  {
    mapModifyMutex_.unlock ();
  }
  
  boost::mutex mapModifyMutex_;
};

#endif
