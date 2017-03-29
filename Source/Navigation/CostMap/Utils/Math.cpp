#include "Math.h"

double
distanceToLine (double pX, double pY, double x0, double y0, double x1,
                double y1)
{
  double A = pX - x0;
  double B = pY - y0;
  double C = x1 - x0;
  double D = y1 - y0;
  
  double dot = A * C + B * D;
  double len_sq = C * C + D * D;
  double param = dot / len_sq;
  
  double xx, yy;
  
  if (param < 0)
  {
    xx = x0;
    yy = y0;
  }
  else if (param > 1)
  {
    xx = x1;
    yy = y1;
  }
  else
  {
    xx = x0 + param * C;
    yy = y0 + param * D;
  }
  
  return distance (pX, pY, xx, yy);
}

bool
intersects (std::vector<NS_DataType::Point>& polygon, float testx, float testy)
{
  bool c = false;
  int i, j, nvert = polygon.size ();
  for (i = 0, j = nvert - 1; i < nvert; j = i++)
  {
    float yi = polygon[i].y, yj = polygon[j].y, xi = polygon[i].x, xj =
        polygon[j].x;
    
    if (((yi > testy) != (yj > testy))
        && (testx < (xj - xi) * (testy - yi) / (yj - yi) + xi))
      c = !c;
  }
  return c;
}

bool
intersects_helper (std::vector<NS_DataType::Point>& polygon1,
                   std::vector<NS_DataType::Point>& polygon2)
{
  for (unsigned int i = 0; i < polygon1.size (); i++)
    if (intersects (polygon2, polygon1[i].x, polygon1[i].y))
      return true;
  return false;
}

bool
intersects (std::vector<NS_DataType::Point>& polygon1,
            std::vector<NS_DataType::Point>& polygon2)
{
  return intersects_helper (polygon1, polygon2)
      || intersects_helper (polygon2, polygon1);
}
