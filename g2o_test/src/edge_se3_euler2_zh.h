/*
 * Jan. 29, 2016 David Z
 *
 *  reimplement an euler based SE3 class 
 *
 * */

#ifndef EDGE_SE3_EULER2_ZH_H
#define EDGE_SE3_EULER2_ZH_H

#include "edge_se3_euler_zh.h"
#include "g2o/config.h"

namespace g2o{

  class EdgeSE3Euler2ZH : public EdgeSE3EulerZH
  {
    public:
      using EdgeSE3::setMeasurement;
    public: 
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      virtual bool read(std::istream & is); 
      virtual bool write(std::ostream & os) const; 
      virtual bool setMeasurement(const double *d){
        Eigen::Map<const Vector6d> v(d); 
        setMeasurement(internal::fromVectorET(v));
        return true;
      }
      virtual bool getMeasurementData(double *d) const{
        Eigen::Map<Vector6d> v(d); 
        v = internal::toVectorET(_measurement); 
        return true;
      }
      
      virtual int measurementDimension() const {return 6;}

      // these two main functions need to be overloaded 
      void linearizeOplus(); // calls computeError to compute numeric Jacobian 
      void computeError();  

  };

}

#endif
