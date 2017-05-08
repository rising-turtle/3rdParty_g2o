/*
 *  Feb. 11, 2016, David Z
 *  
 *  TODO:  not finished 
 *  check Jacobian for edgeSE3PlaneSensorCalib, plane node 
 *
 * */

#ifndef EDGE_SE3_PLANE_SENSOR_CALIB_ZH_H
#define EDGE_SE3_PLANE_SENSOR_CALIB_ZH_H

#include "g2o/types/slam3d_addons/edge_se3_plane_calib.h"

namespace g2o{

  class EdgeSE3PlaneSensorCalibZH : public EdgeSE3PlaneSesnsorCalib
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      EdgeSE3PlaneSensorCalibZH();

      void linearizeOplus(); // compute the jacobian 
      void calJacobian(const Isometry3D& , Plane3D& ); // it works only offsetSE3 = SE3::Identity() 
  };

}


#endif

