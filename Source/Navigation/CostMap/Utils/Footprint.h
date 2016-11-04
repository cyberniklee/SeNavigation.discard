#ifndef _COSTMAP_FOOTPRINT_H_
#define _COSTMAP_FOOTPRINT_H_

#include <DataSet/DataType/Polygon.h>
#include <DataSet/DataType/PolygonStamped.h>
#include <DataSet/DataType/Point.h>
#include <DataSet/DataType/Point32.h>

namespace NS_CostMap
{

/**
 * @brief Calculate the extreme distances for the footprint
 *
 * @param footprint The footprint to examine
 * @param min_dist Output parameter of the minimum distance
 * @param max_dist Output parameter of the maximum distance
 */
void calculateMinAndMaxDistances(const std::vector<NS_DataType::Point>& footprint,
                                 double& min_dist, double& max_dist);

/**
 * @brief Convert Point32 to Point
 */
NS_DataType::Point              toPoint(NS_DataType::Point32 pt);

/**
 * @brief Convert Point to Point32
 */
NS_DataType::Point32            toPoint32(NS_DataType::Point   pt);

/**
 * @brief Convert vector of Points to Polygon msg
 */
NS_DataType::Polygon            toPolygon(std::vector<NS_DataType::Point> pts);

/**
 * @brief Convert Polygon msg to vector of Points.
 */
std::vector<NS_DataType::Point> toPointVector(NS_DataType::Polygon polygon);

/**
 * @brief  Given a pose and base footprint, build the oriented footprint of the robot (list of Points)
 * @param  x The x position of the robot
 * @param  y The y position of the robot
 * @param  theta The orientation of the robot
 * @param  footprint_spec Basic shape of the footprint
 * @param  oriented_footprint Will be filled with the points in the oriented footprint of the robot
*/
void transformFootprint(double x, double y, double theta, const std::vector<NS_DataType::Point>& footprint_spec,
                        std::vector<NS_DataType::Point>& oriented_footprint);

/**
 * @brief  Given a pose and base footprint, build the oriented footprint of the robot (PolygonStamped)
 * @param  x The x position of the robot
 * @param  y The y position of the robot
 * @param  theta The orientation of the robot
 * @param  footprint_spec Basic shape of the footprint
 * @param  oriented_footprint Will be filled with the points in the oriented footprint of the robot
*/
void transformFootprint(double x, double y, double theta, const std::vector<NS_DataType::Point>& footprint_spec,
		NS_DataType::PolygonStamped & oriented_footprint);

/**
 * @brief Adds the specified amount of padding to the footprint (in place)
 */
void padFootprint(std::vector<NS_DataType::Point>& footprint, double padding);

/**
 * @brief Create a circular footprint from a given radius
 */
std::vector<NS_DataType::Point> makeFootprintFromRadius(double radius);

/**
 * @brief Make the footprint from the given string.
 *
 * Format should be bracketed array of arrays of floats, like so: [[1.0, 2.2], [3.3, 4.2], ...]
 *
 */
bool makeFootprintFromString(const std::string& footprint_string, std::vector<NS_DataType::Point>& footprint);

}  // end namespace costmap_2d

#endif  // COSTMAP_2D_FOOTPRINT_H
