
#ifndef _SELIDAR_TYPES_H_
#define _SELIDAR_TYPES_H_


namespace NS_Selidar{

enum ResultCode
{
  BadCRC = -5,
  Denied = -4,
  Invalid = -3,
  Timeout = -2,
  Failure = -1,
  Success = 0,
};

#define IS_OK(x)    ( (x) == 0 )
#define IS_FAIL(x)  ( (x) < 0 )

}

#endif
