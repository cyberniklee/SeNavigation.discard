#ifndef _BASE_LOCAL_PLANNER_TRAJECTORY_INC_H_
#define _BASE_LOCAL_PLANNER_TRAJECTORY_INC_H_

#include <limits>

#ifndef DBL_MAX   /* Max decimal value of a double */
#define DBL_MAX   std::numeric_limits<double>::max()  // 1.7976931348623157e+308
#endif

#ifndef DBL_MIN //Min decimal value of a double
#define DBL_MIN   std::numeric_limits<double>::min()  // 2.2250738585072014e-308
#endif

#endif
