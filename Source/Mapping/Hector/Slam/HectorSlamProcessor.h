#ifndef _HECTOR_SLAM_PROCESSOR_H_
#define _HECTOR_SLAM_PROCESSOR_H_

#include <float.h>
#include "../Map/GridMap.h"
#include "../Map/OccGridMapUtilConfig.h"
#include "../Matcher/ScanMatcher.h"
#include "../Scan/DataPointContainer.h"
#include "../Slam/MapRepMultiMap.h"
#include "../Slam/MapRepresentationInterface.h"
#include "../Utils/DrawInterface.h"
#include "../Utils/HectorDebugInfoInterface.h"
#include "../Utils/MapLockerInterface.h"
#include "../Utils/UtilFunctions.h"

namespace NS_HectorMapping
{
  
  class HectorSlamProcessor
  {
  public:
    
    HectorSlamProcessor (float mapResolution, int mapSizeX, int mapSizeY,
                         const Eigen::Vector2f& startCoords, int multi_res_size,
                         DrawInterface* drawInterfaceIn = 0,
                         HectorDebugInfoInterface* debugInterfaceIn = 0)
        : drawInterface (drawInterfaceIn), debugInterface (debugInterfaceIn)
    {
      mapRep = new MapRepMultiMap (mapResolution, mapSizeX, mapSizeY,
                                   multi_res_size, startCoords, drawInterfaceIn,
                                   debugInterfaceIn);
      
      this->reset ();
      
      this->setMapUpdateMinDistDiff (0.4f * 1.0f);
      this->setMapUpdateMinAngleDiff (0.13f * 1.0f);
    }
    
    ~HectorSlamProcessor ()
    {
      delete mapRep;
    }
    
    void
    update (const DataContainer& dataContainer,
            const Eigen::Vector3f& poseHintWorld, bool map_without_matching =
                false)
    {
      //std::cout << "\nph:\n" << poseHintWorld << "\n";
      
      Eigen::Vector3f newPoseEstimateWorld;
      
      if (!map_without_matching)
      {
        newPoseEstimateWorld = (mapRep->matchData (poseHintWorld, dataContainer,
                                                   lastScanMatchCov));
      }
      else
      {
        newPoseEstimateWorld = poseHintWorld;
      }
      
      lastScanMatchPose = newPoseEstimateWorld;
      
      //std::cout << "\nt1:\n" << newPoseEstimateWorld << "\n";
      
      //std::cout << "\n1";
      //std::cout << "\n" << lastScanMatchPose << "\n";
      if (NS_HectorMapping::poseDifferenceLargerThan (
          newPoseEstimateWorld, lastMapUpdatePose,
          paramMinDistanceDiffForMapUpdate, paramMinAngleDiffForMapUpdate)
          || map_without_matching)
      {

        mapRep->updateByScan (dataContainer, newPoseEstimateWorld);
        
        mapRep->onMapUpdated ();
        lastMapUpdatePose = newPoseEstimateWorld;
      }
      
      if (drawInterface)
      {
        const GridMap& gridMapRef (mapRep->getGridMap ());
        drawInterface->setColor (1.0, 0.0, 0.0);
        drawInterface->setScale (0.15);
        
        drawInterface->drawPoint (
            gridMapRef.getWorldCoords (Eigen::Vector2f::Zero ()));
        drawInterface->drawPoint (
            gridMapRef.getWorldCoords (
                (gridMapRef.getMapDimensions ().array () - 1).cast<float> ()));
        drawInterface->drawPoint (Eigen::Vector2f (1.0f, 1.0f));
        
        drawInterface->sendAndResetData ();
      }
      
      if (debugInterface)
      {
        debugInterface->sendAndResetData ();
      }
    }
    
    void
    reset ()
    {
      lastMapUpdatePose = Eigen::Vector3f (FLT_MAX, FLT_MAX, FLT_MAX);
      lastScanMatchPose = Eigen::Vector3f::Zero ();
      //lastScanMatchPose.x() = -10.0f;
      //lastScanMatchPose.y() = -15.0f;
      //lastScanMatchPose.z() = M_PI*0.15f;
      
      mapRep->reset ();
    }
    
    const Eigen::Vector3f&
    getLastScanMatchPose () const
    {
      return lastScanMatchPose;
    }
    ;
    const Eigen::Matrix3f&
    getLastScanMatchCovariance () const
    {
      return lastScanMatchCov;
    }
    ;
    float
    getScaleToMap () const
    {
      return mapRep->getScaleToMap ();
    }
    ;

    int
    getMapLevels () const
    {
      return mapRep->getMapLevels ();
    }
    ;
    const GridMap&
    getGridMap (int mapLevel = 0) const
    {
      return mapRep->getGridMap (mapLevel);
    }
    ;

    void
    addMapMutex (int i, MapLockerInterface* mapMutex)
    {
      mapRep->addMapMutex (i, mapMutex);
    }
    ;
    MapLockerInterface*
    getMapMutex (int i)
    {
      return mapRep->getMapMutex (i);
    }
    ;

    void
    setUpdateFactorFree (float free_factor)
    {
      mapRep->setUpdateFactorFree (free_factor);
    }
    ;
    void
    setUpdateFactorOccupied (float occupied_factor)
    {
      mapRep->setUpdateFactorOccupied (occupied_factor);
    }
    ;
    void
    setMapUpdateMinDistDiff (float minDist)
    {
      paramMinDistanceDiffForMapUpdate = minDist;
    }
    ;
    void
    setMapUpdateMinAngleDiff (float angleChange)
    {
      paramMinAngleDiffForMapUpdate = angleChange;
    }
    ;

  protected:
    
    MapRepresentationInterface* mapRep;

    Eigen::Vector3f lastMapUpdatePose;
    Eigen::Vector3f lastScanMatchPose;
    Eigen::Matrix3f lastScanMatchCov;

    float paramMinDistanceDiffForMapUpdate;
    float paramMinAngleDiffForMapUpdate;

    DrawInterface* drawInterface;
    HectorDebugInfoInterface* debugInterface;
  };

}

#endif
