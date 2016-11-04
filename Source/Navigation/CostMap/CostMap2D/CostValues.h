#ifndef _COSTMAP_COST_VALUES_H_
#define _COSTMAP_COST_VALUES_H_
/** Provides a mapping for often used cost values */
namespace NS_CostMap
{
static const unsigned char NO_INFORMATION = 255;
static const unsigned char LETHAL_OBSTACLE = 254;
static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
static const unsigned char FREE_SPACE = 0;
}
#endif  // COSTMAP_2D_COST_VALUES_H_
