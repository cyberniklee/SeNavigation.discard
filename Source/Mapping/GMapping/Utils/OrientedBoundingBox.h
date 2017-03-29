#ifndef  _ORIENTENDBOUNDINGBOX_H_
#define  _ORIENTENDBOUNDINGBOX_H_

#include <stdio.h>
#include <math.h>

#include "../Utils/Point.h"

namespace NS_GMapping
{
  
  template<class NUMERIC>
    class OrientedBoundingBox
    {
      
    public:
      OrientedBoundingBox (std::vector<point<NUMERIC> > p);
      double
      area ();

    protected:
      Point ul;
      Point ur;
      Point ll;
      Point lr;
    };

#include "OrientedBoundingBox.hxx"

}
;
// end namespace

#endif

