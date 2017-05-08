#include "edge_se2_line2d_zh.h"

#ifdef G2O_HAVE_OPENGL
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#endif

namespace g2o{

  EdgeSE2Line2DZH::EdgeSE2Line2DZH() : 
    EdgeSE2Line2D(){}

  EdgeSE2Line2DZH::~EdgeSE2Line2DZH() {}

  /*
  void EdgeSE2Line2DZH::linearizeOplus()
  {
    const VertexSE2* vi     = static_cast<const VertexSE2*>(_vertices[0]);
    const VertexLine2D* vj = static_cast<const VertexLine2D*>(_vertices[1]);
    const double& x1        = vi->estimate().translation()[0];
    const double& y1        = vi->estimate().translation()[1];
    const double& th1       = vi->estimate().rotation().angle();
    const double& x2        = vj->estimate()[0];
    const double& y2        = vj->estimate()[1];

    double aux_1 = cos(th1) ;
    double aux_2 = -aux_1 ;
    double aux_3 = sin(th1) ;

    _jacobianOplusXi( 0 , 0 ) = aux_2 ;
    _jacobianOplusXi( 0 , 1 ) = -aux_3 ;
    _jacobianOplusXi( 0 , 2 ) = aux_1*y2-aux_1*y1-aux_3*x2+aux_3*x1 ;
    _jacobianOplusXi( 1 , 0 ) = aux_3 ;
    _jacobianOplusXi( 1 , 1 ) = aux_2 ;
    _jacobianOplusXi( 1 , 2 ) = -aux_3*y2+aux_3*y1-aux_1*x2+aux_1*x1 ;

    _jacobianOplusXj( 0 , 0 ) = aux_1 ;
    _jacobianOplusXj( 0 , 1 ) = aux_3 ;
    _jacobianOplusXj( 1 , 0 ) = -aux_3 ;
    _jacobianOplusXj( 1 , 1 ) = aux_1 ;
  }*/
  void EdgeSE2Line2DZH::linearizeOplus()
  {
    // compute as the original way 
    // EdgeSE2Line2D::linearizeOplus(); 
    // cout<<"edge_se2_line2d_zh.cpp: numeric jacobian matrix: "<<endl;
    // cout<<"jacobianXi: "<<endl
    //    <<_jacobianOplusXi<<endl; 
    // cout<<"jacobianXj: "<<endl
    //    <<_jacobianOplusXj<<endl;

    const VertexSE2* vi     = static_cast<const VertexSE2*>(_vertices[0]);
    const VertexLine2D* vj = static_cast<const VertexLine2D*>(_vertices[1]);
    const double& x        = vi->estimate().translation()[0];
    const double& y        = vi->estimate().translation()[1];
    const double& theta       = vi->estimate().rotation().angle();
    const double& alpha_j        = vj->estimate()[0];
    const double& d_j        = vj->estimate()[1];

    // cout<<"edge_se2_line2d_zh.cpp: line angle = "<<(alpha_j*180.)/M_PI<<" x= "<<x<<" y = "<<y <<endl;
    double cos_alpha_j = cos(alpha_j) ;
    double sin_alpha_j = sin(alpha_j) ;
    
    _jacobianOplusXi(0, 0) = 0; // 
    _jacobianOplusXi(0, 1) = 0; // 
    _jacobianOplusXi(0, 2) = -1; 
    _jacobianOplusXi(1, 0) = -cos_alpha_j; 
    _jacobianOplusXi(1, 1) = -sin_alpha_j; 
    _jacobianOplusXi(1, 2) = 0; 

     _jacobianOplusXj(0, 0) = 1; 
     _jacobianOplusXj(0, 1) = 0; 
     _jacobianOplusXj(1, 0) = sin_alpha_j*x - cos_alpha_j*y; 
     _jacobianOplusXj(1, 1) = 1; 

    // the derived way of computing jacobian
    // cout<<"edge_se2_line2d_zh.cpp: equation jacobian matrix: "<<endl;
    // cout<<"jacobianXi: "<<endl
    //    <<_jacobianOplusXi<<endl; 
    // cout<<"jacobianXj: "<<endl
    //    <<_jacobianOplusXj<<endl;
  }

  #ifdef G2O_HAVE_OPENGL
   EdgeSE2Line2DZHDrawAction::EdgeSE2Line2DZHDrawAction(): DrawAction(typeid(EdgeSE2Line2DZH).name()){}

     HyperGraphElementAction* EdgeSE2Line2DZHDrawAction::operator()(HyperGraph::HyperGraphElement* element,
                   HyperGraphElementAction::Parameters*  params_){
       if (typeid(*element).name()!=_typeName)
         return 0;
 
       refreshPropertyPtrs(params_);
      if (! _previousParams)
        return this;
  
       if (_show && !_show->value())
        return this;
 
  
       EdgeSE2Line2DZH* e =  static_cast<EdgeSE2Line2DZH*>(element);
       VertexSE2* fromEdge = static_cast<VertexSE2*>(e->vertex(0));
       VertexLine2D* toEdge   = static_cast<VertexLine2D*>(e->vertex(1));
       glColor3f(0.4f,0.4f,0.2f);
       glPushAttrib(GL_ENABLE_BIT);
       glDisable(GL_LIGHTING);
       glBegin(GL_LINES);
       glVertex3f((float)fromEdge->estimate().translation().x(),(float)fromEdge->estimate().translation().y(),0.f);
       glVertex3f((float)toEdge->estimate().x(),(float)toEdge->estimate().y(),0.f);
       glEnd();
       glPopAttrib();
       return this;
     }
   #endif
  

}
