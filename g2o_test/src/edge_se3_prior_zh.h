/*  
 *  Jan. 25, 2016 David Z 
 *  
 *  Try to test the jacobian function of EdgeSE3Prior, 
 *  use Euler angle to represent Rotation part
 *
 * */
#ifndef EDGE_SE3_PRIOR_ZH_H
#define EDGE_SE3_PRIOR_ZH_H

#include "g2o/types/slam3d/edge_se3_prior.h"
#include "g2o/config.h"

using namespace std; 
using namespace g2o; 
using namespace Eigen;

namespace g2o{
  
  class EdgeSE3PriorZH : public EdgeSE3Prior
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      EdgeSE3PriorZH(); 
      
      virtual bool read(std::istream& is); 
      virtual bool write(std::ostream& os) const; 

    public:
      using EdgeSE3Prior::resolveCaches;
      virtual ~EdgeSE3PriorZH(); 
      void numericJacobian(); 
      bool resolveCacheAndParameters(); 

      // critical functions need to be overloaded
      virtual void linearizeOplus();
      void computeError(); 

      // this function now is not compatible with the class
      void analyticJacobian();
  };

}




#endif 
