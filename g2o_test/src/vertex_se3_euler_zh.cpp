#include "vertex_se3_euler_zh.h"
#include "g2o/core/factory.h"

#include <iostream>
#include "g2o/core/cache.h"

using namespace Eigen; 

namespace g2o{
  
  VertexSE3EulerZH::VertexSE3EulerZH(){}
  VertexSE3EulerZH::~VertexSE3EulerZH(){}
  
  bool VertexSE3EulerZH::read(std::istream& is)
  {
    Vector6d est; 
    for(int i=0; i<6; i++)
    {
      is >> est[i]; 
    }
    setEstimate(internal::fromVectorET(est)); 
    updateCache();
    return true;
  }
  
  bool VertexSE3EulerZH::write(std::ostream& os) const
  {
    Vector6d est = internal::toVectorET(_estimate); 
    for(int i=0; i<6; i++)
      os << est[i]<<" ";
    return os.good();
  }
  
}

