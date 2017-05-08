/*
 *  Jan. 29, 2016 David Z
 *  
 *  Implement se3 euler 
 *
 * */

#ifndef VERTEX_SE3_EULER_ZH_H
#define VERTEX_SE3_EULER_ZH_H

#include "g2o/config.h"
#include "g2o/types/slam3d_addons/vertex_se3_euler.h"

namespace g2o{

  class VertexSE3EulerZH : public VertexSE3Euler
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW; 
      VertexSE3EulerZH(); 
      virtual ~VertexSE3EulerZH(); 
       virtual bool read(std::istream& is);
       virtual bool write(std::ostream& os) const; 
      virtual bool setEstimateDataImpl(const double * est){
        Eigen::Map<const Vector6d> v(est); 
        _estimate = internal::fromVectorET(v);
        return true;
      }
      virtual bool getEstimateData(double* est) const{
        Eigen::Map<Vector6d> v(est); 
        v = internal::toVectorET(_estimate); 
      }
      virtual int estimateDimension() const {return 6; } // will this cause any problem? 
      virtual bool setMinimalEstimateDataImpl(const double * est){
        return setEstimateDataImpl(est); 
      }
      virtual bool getMinimalEstimateData(double *est) const{
        return getEstimateData(est);
      }
      virtual int minimalEstimateDimension() const {return 6;}
      
      /**
       *  update the position of this vertex. the update is in the form
       *  {x, y, z, roll, pitch, yaw} whereas (x, y, z) represents the translational update
       *  and (r, p, y) corresponds to the respective rotational euler angles. 
       */
      virtual void oplusImpl(const double * update)
      {
        Eigen::Map<const Vector6d> v(update); 
        Isometry3D increment = internal::fromVectorET(v); 
        _estimate = _estimate * increment; 
        if(++_numOplusCalls > orthogonalizeAfter){
          _numOplusCalls = 0; 
          internal::approximateNearestOrthogonalMatrix(_estimate.matrix().topLeftCorner<3,3>());
        }
      }
  };

}

#endif
