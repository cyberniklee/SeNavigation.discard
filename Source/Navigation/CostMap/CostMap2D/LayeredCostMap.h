#ifndef _COSTMAP_LAYERED_COSTMAP_H_
#define _COSTMAP_LAYERED_COSTMAP_H_

#include "CostValues.h"
#include "Layer.h"
#include "CostMap2D.h"
#include <vector>
#include <string>

namespace NS_CostMap
{
  class Layer;
  
  /**
   * @class LayeredCostmap
   * @brief Instantiates different layer plugins and aggregates them into one score
   */
  class LayeredCostmap
  {
  public:
    /**
     * @brief  Constructor for a costmap
     */
    LayeredCostmap (bool track_unknown);

    /**
     * @brief  Destructor
     */
    ~LayeredCostmap ();

    /**
     * @brief  Update the underlying costmap with new data.
     * If you want to update the map outside of the update loop that runs, you can call this.
     */
    void
    updateMap (double robot_x, double robot_y, double robot_yaw);

    void
    resizeMap (unsigned int size_x, unsigned int size_y, double resolution,
               double origin_x, double origin_y, bool size_locked = false);

    void
    getUpdatedBounds (double& minx, double& miny, double& maxx, double& maxy)
    {
      minx = minx_;
      miny = miny_;
      maxx = maxx_;
      maxy = maxy_;
    }
    
    bool
    isCurrent ();

    Costmap2D*
    getCostmap ()
    {
      return &costmap_;
    }
    
    bool
    isTrackingUnknown ()
    {
      return costmap_.getDefaultValue () == NS_CostMap::NO_INFORMATION;
    }
    
    std::vector<boost::shared_ptr<Layer> >*
    getPlugins ()
    {
      return &plugins_;
    }
    
    void
    addPlugin (boost::shared_ptr<Layer> plugin)
    {
      plugins_.push_back (plugin);
    }
    
    bool
    isSizeLocked ()
    {
      return size_locked_;
    }
    
    void
    getBounds (unsigned int* x0, unsigned int* xn, unsigned int* y0,
               unsigned int* yn)
    {
      *x0 = bx0_;
      *xn = bxn_;
      *y0 = by0_;
      *yn = byn_;
    }
    
    bool
    isInitialized ()
    {
      return initialized_;
    }
    
    /** @brief Updates the stored footprint, updates the circumscribed
     * and inscribed radii, and calls onFootprintChanged() in all
     * layers. */
    void
    setFootprint (const std::vector<NS_DataType::Point>& footprint_spec);

    /** @brief Returns the latest footprint stored with setFootprint(). */
    const std::vector<NS_DataType::Point>&
    getFootprint ()
    {
      return footprint_;
    }
    
    /** @brief The radius of a circle centered at the origin of the
     * robot which just surrounds all points on the robot's
     * footprint.
     *
     * This is updated by setFootprint(). */
    double
    getCircumscribedRadius ()
    {
      return circumscribed_radius_;
    }
    
    /** @brief The radius of a circle centered at the origin of the
     * robot which is just within all points and edges of the robot's
     * footprint.
     *
     * This is updated by setFootprint(). */
    double
    getInscribedRadius ()
    {
      return inscribed_radius_;
    }
    
  private:
    Costmap2D costmap_;

    bool current_;
    double minx_, miny_, maxx_, maxy_;
    unsigned int bx0_, bxn_, by0_, byn_;

    std::vector<boost::shared_ptr<Layer> > plugins_;

    bool initialized_;
    bool size_locked_;
    double circumscribed_radius_, inscribed_radius_;
    std::vector<NS_DataType::Point> footprint_;
  };

}  // namespace costmap_2d

#endif  // COSTMAP_2D_LAYERED_COSTMAP_H_
