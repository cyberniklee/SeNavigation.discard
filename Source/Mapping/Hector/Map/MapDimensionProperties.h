#ifndef _MapDimensionProperties_h_
#define _MapDimensionProperties_h_

class MapDimensionProperties
{
public:
  MapDimensionProperties ()
      : topLeftOffset (-1.0f, -1.0f), mapDimensions (-1, -1), cellLength (-1.0f)
  {
  }
  
  MapDimensionProperties (const Eigen::Vector2f& topLeftOffsetIn,
                          const Eigen::Vector2i& mapDimensionsIn,
                          float cellLengthIn)
      : topLeftOffset (topLeftOffsetIn), mapDimensions (mapDimensionsIn),
          cellLength (cellLengthIn)
  {
    mapLimitsf = (mapDimensionsIn.cast<float> ()).array () - 1.0f;
  }
  
  bool
  operator== (const MapDimensionProperties& other) const
  {
    return (topLeftOffset == other.topLeftOffset)
        && (mapDimensions == other.mapDimensions)
        && (cellLength == other.cellLength);
  }
  
  bool
  hasEqualDimensionProperties (const MapDimensionProperties& other) const
  {
    return (mapDimensions == other.mapDimensions);
  }
  
  bool
  hasEqualTransformationProperties (const MapDimensionProperties& other) const
  {
    return (topLeftOffset == other.topLeftOffset)
        && (cellLength == other.cellLength);
  }
  
  bool
  pointOutOfMapBounds (const Eigen::Vector2f& coords) const
  {
    return ((coords[0] < 0.0f) || (coords[0] > mapLimitsf[0])
        || (coords[1] < 0.0f) || (coords[1] > mapLimitsf[1]));
  }
  
  void
  setMapCellDims (const Eigen::Vector2i& newDims)
  {
    mapDimensions = newDims;
    mapLimitsf = (newDims.cast<float> ()).array () - 2.0f;
  }
  
  void
  setTopLeftOffset (const Eigen::Vector2f& topLeftOffsetIn)
  {
    topLeftOffset = topLeftOffsetIn;
  }
  
  void
  setSizeX (int sX)
  {
    mapDimensions[0] = sX;
  }
  ;
  void
  setSizeY (int sY)
  {
    mapDimensions[1] = sY;
  }
  ;
  void
  setCellLength (float cl)
  {
    cellLength = cl;
  }
  ;

  const Eigen::Vector2f&
  getTopLeftOffset () const
  {
    return topLeftOffset;
  }
  ;
  const Eigen::Vector2i&
  getMapDimensions () const
  {
    return mapDimensions;
  }
  ;
  int
  getSizeX () const
  {
    return mapDimensions[0];
  }
  ;
  int
  getSizeY () const
  {
    return mapDimensions[1];
  }
  ;
  float
  getCellLength () const
  {
    return cellLength;
  }
  ;

protected:
  Eigen::Vector2f topLeftOffset;
  Eigen::Vector2i mapDimensions;
  Eigen::Vector2f mapLimitsf;
  float cellLength;
};

#endif

