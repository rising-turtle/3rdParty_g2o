/*
 *  Jan. 15, 2016 David Z 
 *  A derived class of EdgeSE2Line2D, that reimplement the Jacobian function 
 *
 * */

#ifndef EDGE_SE2_LINE2D_ZH_H
#define EDGE_SE2_LINE2D_ZH_H

#include "g2o/types/slam2d_addons/edge_se2_line2d.h"
#include "g2o/config.h"

using namespace std; 
using namespace Eigen;
using namespace g2o; 

namespace g2o{

  class EdgeSE2Line2DZH : public EdgeSE2Line2D
  {
    public:
      EdgeSE2Line2DZH(); 
      virtual ~EdgeSE2Line2DZH(); 
      virtual void linearizeOplus(); 
  };

   #ifdef G2O_HAVE_OPENGL 
         class EdgeSE2Line2DZHDrawAction: public DrawAction{ 
         public: 
           EdgeSE2Line2DZHDrawAction(); 
           virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
                   HyperGraphElementAction::Parameters* params_);
         }; 
       #endif 




}


#endif
