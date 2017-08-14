#ifndef _MAP_LOCK_INTERFACE_
#define _MAP_LOCK_INTERFACE_

class MapLockerInterface
{
public:
  virtual void
  lockMap () = 0;
  virtual void
  unlockMap () = 0;
};

#endif
