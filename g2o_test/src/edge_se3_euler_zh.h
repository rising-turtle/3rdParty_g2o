/*
 *  Jan. 29, 2016 David Z
 *
 *  implement rotation with euler angle
 *
 * */

#ifndef EDGE_SE3_EULER_ZH_H
#define EDGE_SE3_EULER_ZH_H

#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/config.h"

using namespace std; 
using namespace g2o; 
using namespace Eigen;
namespace g2o{
  class EdgeSE3EulerZH: public EdgeSE3
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      virtual bool read(std::istream& is); 
      virtual bool write(std::ostream& os) const;
  };

}


#endif

