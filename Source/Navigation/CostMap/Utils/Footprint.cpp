#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include "Footprint.h"
#include "ArrayParser.h"
#include <DataSet/DataType/Point32.h>
#include <Console/Console.h>
#include "Math.h"

namespace NS_CostMap
{

void calculateMinAndMaxDistances(const std::vector<NS_DataType::Point>& footprint, double& min_dist, double& max_dist)
{
  min_dist = std::numeric_limits<double>::max();
  max_dist = 0.0;

  if (footprint.size() <= 2)
  {
    return;
  }

  for (unsigned int i = 0; i < footprint.size() - 1; ++i)
  {
    // check the distance from the robot center point to the first vertex
    double vertex_dist = distance(0.0, 0.0, footprint[i].x, footprint[i].y);
    double edge_dist = distanceToLine(0.0, 0.0, footprint[i].x, footprint[i].y,
                                      footprint[i + 1].x, footprint[i + 1].y);
    min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
    max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
  }

  // we also need to do the last vertex and the first vertex
  double vertex_dist = distance(0.0, 0.0, footprint.back().x, footprint.back().y);
  double edge_dist = distanceToLine(0.0, 0.0, footprint.back().x, footprint.back().y,
                                      footprint.front().x, footprint.front().y);
  min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
  max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
}

NS_DataType::Point32 toPoint32(NS_DataType::Point pt)
{
  NS_DataType::Point32 point32;
  point32.x = pt.x;
  point32.y = pt.y;
  point32.z = pt.z;
  return point32;
}

NS_DataType::Point toPoint(NS_DataType::Point32 pt)
{
  NS_DataType::Point point;
  point.x = pt.x;
  point.y = pt.y;
  point.z = pt.z;
  return point;
}

NS_DataType::Polygon toPolygon(std::vector<NS_DataType::Point> pts)
{
  NS_DataType::Polygon polygon;
  for (int i = 0; i < pts.size(); i++){
    polygon.points.push_back(toPoint32(pts[i]));
  }
  return polygon;
}

std::vector<NS_DataType::Point> toPointVector(NS_DataType::Polygon polygon)
{
  std::vector<NS_DataType::Point> pts;
  for (int i = 0; i < polygon.points.size(); i++)
  {
    pts.push_back(toPoint(polygon.points[i]));
  }
  return pts;
}

void transformFootprint(double x, double y, double theta, const std::vector<NS_DataType::Point>& footprint_spec,
                        std::vector<NS_DataType::Point>& oriented_footprint)
{
  // build the oriented footprint at a given location
  oriented_footprint.clear();
  double cos_th = cos(theta);
  double sin_th = sin(theta);
  for (unsigned int i = 0; i < footprint_spec.size(); ++i)
  {
    NS_DataType::Point new_pt;
    new_pt.x = x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
    new_pt.y = y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
    oriented_footprint.push_back(new_pt);
  }
}

void transformFootprint(double x, double y, double theta, const std::vector<NS_DataType::Point>& footprint_spec,
		NS_DataType::PolygonStamped& oriented_footprint)
{
  // build the oriented footprint at a given location
  oriented_footprint.polygon.points.clear();
  double cos_th = cos(theta);
  double sin_th = sin(theta);
  for (unsigned int i = 0; i < footprint_spec.size(); ++i)
  {
    NS_DataType::Point32 new_pt;
    new_pt.x = x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
    new_pt.y = y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
    oriented_footprint.polygon.points.push_back(new_pt);
  }
}

void padFootprint(std::vector<NS_DataType::Point>& footprint, double padding)
{
  // pad footprint in place
  for (unsigned int i = 0; i < footprint.size(); i++)
  {
    NS_DataType::Point& pt = footprint[ i ];
    pt.x += sign0(pt.x) * padding;
    pt.y += sign0(pt.y) * padding;
  }
}


std::vector<NS_DataType::Point> makeFootprintFromRadius(double radius)
{
  std::vector<NS_DataType::Point> points;

  // Loop over 16 angles around a circle making a point each time
  int N = 16;
  NS_DataType::Point pt;
  for (int i = 0; i < N; ++i)
  {
    double angle = i * 2 * M_PI / N;
    pt.x = cos(angle) * radius;
    pt.y = sin(angle) * radius;

    points.push_back(pt);
  }

  return points;
}


bool makeFootprintFromString(const std::string& footprint_string, std::vector<NS_DataType::Point>& footprint)
{
  std::string error;
  std::vector<std::vector<float> > vvf = parseVVF(footprint_string, error);

  if (error != "")
  {
    NS_NaviCommon::console.error("Error parsing footprint parameter: '%s'", error.c_str());
    NS_NaviCommon::console.error("  Footprint string was '%s'.", footprint_string.c_str());
    return false;
  }

  // convert vvf into points.
  if (vvf.size() < 3)
  {
    NS_NaviCommon::console.error("You must specify at least three points for the robot footprint, reverting to previous footprint.");
    return false;
  }
  footprint.reserve(vvf.size());
  for (unsigned int i = 0; i < vvf.size(); i++)
  {
    if (vvf[ i ].size() == 2)
    {
      NS_DataType::Point point;
      point.x = vvf[ i ][ 0 ];
      point.y = vvf[ i ][ 1 ];
      point.z = 0;
      footprint.push_back(point);
    }
    else
    {
      NS_NaviCommon::console.error("Points in the footprint specification must be pairs of numbers.  Found a point with %d numbers.",
                 int(vvf[ i ].size()));
      return false;
    }
  }

  return true;
}

}  // end namespace costmap_2d
