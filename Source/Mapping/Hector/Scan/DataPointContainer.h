#ifndef _DATAPOINTCONTAINER_H_
#define _DATAPOINTCONTAINER_H_

#include <vector>
#include <Eigen/Dense>

namespace NS_HectorMapping
{
  
  template<typename DataPointType>
    class DataPointContainer
    {
    public:
      
      DataPointContainer (int size = 1000)
      {
        dataPoints.reserve (size);
      }
      
      void
      setFrom (const DataPointContainer& other, float factor)
      {
        origo = other.getOrigo () * factor;
        
        dataPoints = other.dataPoints;
        
        unsigned int size = dataPoints.size ();
        
        for (unsigned int i = 0; i < size; ++i)
        {
          dataPoints[i] *= factor;
        }
        
      }
      
      void
      add (const DataPointType& dataPoint)
      {
        dataPoints.push_back (dataPoint);
      }
      
      void
      clear ()
      {
        dataPoints.clear ();
      }
      
      int
      getSize () const
      {
        return dataPoints.size ();
      }
      
      const DataPointType&
      getVecEntry (int index) const
      {
        return dataPoints[index];
      }
      
      DataPointType
      getOrigo () const
      {
        return origo;
      }
      
      void
      setOrigo (const DataPointType& origoIn)
      {
        origo = origoIn;
      }
      
    protected:
      
      std::vector<DataPointType> dataPoints;
      DataPointType origo;
    };
  
  typedef DataPointContainer<Eigen::Vector2f> DataContainer;

}

#endif
