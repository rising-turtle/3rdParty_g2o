#include "edge_se3_euler2_zh.h"
#include "g2o/core/factory.h"
#include <iostream>

namespace g2o{
  
  bool EdgeSE3Euler2ZH::read(std::istream& is)
  {
    Vector6d meas; 
    for(int i=0; i<6; i++)
      is >> meas[i];
    setMeasurement(internal::fromVectorET(meas)); 
    if(is.bad())
      return false;
    for(int i=0; i<information().rows() && is.good(); ++i)
      for(int j=i; j<information().cols() && is.good(); ++j)
      {
        is>>information()(i, j); 
        if(i != j)
          information()(j,i) = information()(i,j);
      }
    if(is.bad())
      information().setIdentity();
    return true;
  }

  bool EdgeSE3Euler2ZH::write(std::ostream& os) const
  {
    Vector6d v = internal::toVectorET(_measurement); 
    for(int i=0; i<6; i++)
      os << v[i]<<" ";
    for(int i=0; i<information().rows(); i++)
      for(int j=i; j<information().cols(); j++)
        os<<information()(i,j)<<" ";
    return os.good();
  }

  void EdgeSE3Euler2ZH::computeError(){
    VertexSE3* from = static_cast<VertexSE3*>(_vertices[0]); 
    VertexSE3* to = static_cast<VertexSE3*>(_vertices[1]); 
    Isometry3D delta = _inverseMeasurement * from->estimate().inverse() * to->estimate(); 
    _error = internal::toVectorET(delta);
  }

  void EdgeSE3Euler2ZH::linearizeOplus()
  {
    // use the numeric jacobian 
    BaseBinaryEdge<6, Isometry3D, VertexSE3, VertexSE3>::linearizeOplus(); 
    return ;
  }

}
