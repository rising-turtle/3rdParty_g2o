#include "edge_se3_calib_zh.h"

namespace g2o{

  EdgeSE3PlaneSensorCalibZH::EdgeSE3PlaneSensorCalibZH(){}  
  
  void EdgeSE3PlaneSensorCalibZH::linearizeOplus()
  {
    const VertexSE3* v1            = static_cast<const VertexSE3*>(_vertices[0]);
    const VertexPlane* planeVertex = static_cast<const VertexPlane*>(_vertices[1]);
    const VertexSE3* offset        = static_cast<const VertexSE3*>(_vertices[2]);
    const Plane3D& plane           = planeVertex->estimate();     
    const Isometry3D vp = v1->estimate(); 
    calJacobian(vp, plane);
  }

  void EdgeSE3PlaneSensorCalibZH::calJacobian(const Isometry3D& T, Plane3D& plane)
  {
    
  }
}

