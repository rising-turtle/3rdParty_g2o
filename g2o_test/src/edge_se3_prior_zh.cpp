#include "edge_se3_prior_zh.h"
#include "g2o/types/slam3d/isometry3d_gradients.h"
#include "vertex_se3_euler_zh.h"
#include <iostream>

namespace g2o{
  
  EdgeSE3PriorZH::EdgeSE3PriorZH() : EdgeSE3Prior(){}
  EdgeSE3PriorZH::~EdgeSE3PriorZH(){}

  bool EdgeSE3PriorZH::read(std::istream& is)
  {
    int pid; 
    is >> pid; 
    if(!setParameterId(0, pid)) 
      return false; 
    Vector6d v; 
    for(int i=0; i<6; i++) is >> v[i]; 
    setMeasurement(g2o::internal::fromVectorET(v)); 
    if (is.bad()) {
      return false;
    }
    for ( int i=0; i<information().rows() && is.good(); i++)
      for (int j=i; j<information().cols() && is.good(); j++){
        is >> information()(i,j);
        if (i!=j)
          information()(j,i)=information()(i,j);
      }
    if (is.bad()) {
      //  we overwrite the information matrix
      information().setIdentity();
    } 
    return true;
  }
  
  bool EdgeSE3PriorZH::write(std::ostream& os) const
  {
    os<< _offsetParam->id()<<" "; 
    double buf[6]; 
    Eigen::Map<Vector6d> v(buf); 
    v = g2o::internal::toVectorET(_measurement); 
    for(int i=0; i<6; i++)
      os << v[i] << " ";
    for (int i=0; i<information().rows(); i++)
      for (int j=i; j<information().cols(); j++) {
        os <<  information()(i,j) << " ";
      }
    return os.good();
  }

  void EdgeSE3PriorZH::computeError()
  {
    // Isometry3D delta = _inverseMeasurement * _cache->n2w(); 
    VertexSE3 * v = (VertexSE3*)(vertices()[0]);
    Isometry3D delta = _inverseMeasurement * v->estimate();
    _error = internal::toVectorET(delta); 
  }

  void EdgeSE3PriorZH::linearizeOplus()
  {
    // cout<<"edge_se3_prior_zh.cpp: in EdgeSE3PriorZH::linearizeOplus()"<<endl;
    numericJacobian(); 
  }

  void EdgeSE3PriorZH::analyticJacobian()
  {
    cout<<"edge_se3_prior_zh.cpp: EdgeSE3PriorZH::analyticJacobian()"<<endl;
    VertexSE3 *from = static_cast<VertexSE3*>(_vertices[0]);
    Isometry3D E;
    Isometry3D Z, X, P;
    X=from->estimate();
    P=_cache->offsetParam()->offset();
    Z=_measurement;
    internal::computeEdgeSE3PriorGradient(E, _jacobianOplusXi, Z, X, P);
  }

  void EdgeSE3PriorZH::numericJacobian()
  {
    // cout<<"edge_se3_prior_zh.cpp: EdgeSE3PriorZH::numericJacobian()"<<endl;
    // VertexSE3 * vi = static_cast<VertexSE3*>(_vertices[0]); 
    VertexSE3EulerZH * vi = static_cast<VertexSE3EulerZH*>(_vertices[0]);

    if(vi->fixed()) return; 
    
    const double delta = 1e-9; 
    const double scalar = 1./(2*delta); 
    ErrorVector error1; 
    ErrorVector errorBeforeNumeric = _error; 
    
    double add_vi[VertexSE3::Dimension]; 
    std::fill(add_vi, add_vi+VertexSE3::Dimension, 0.0); 
    
    // cout<<"edge_se3_prior_zh.cpp: VertexSE3::Dimension: "<<VertexSE3::Dimension<<endl;
    for(int d = 0; d<VertexSE3::Dimension; ++d)
    {
      vi->push(); 
      add_vi[d] = delta; 
      vi->oplus(add_vi);
      computeError(); 
      error1 = _error;
      vi->pop(); 
      vi->push(); 
      add_vi[d] = -delta; 
      vi->oplus(add_vi); 
      computeError(); 
      vi->pop();
      add_vi[d] = 0; 
      
      _jacobianOplusXi.col(d) = scalar * (error1 - _error);
    }

    _error = errorBeforeNumeric; 
  }
  
  bool EdgeSE3PriorZH::resolveCacheAndParameters()
  {
    if(_offsetParam == 0)
    {
      _offsetParam = new ParameterSE3Offset; 
      installParameter(_offsetParam, 0, 0); 
    }

    if(!resolveCaches())
    {
      cerr<<"edge_se3_prior_zh.cpp: failed to resolveCaches()"<<endl;
      if(_cache == 0)
      {
        cerr<<"yes, _cache = 0"<<endl;
      }else
      {
        cerr<<"what? _cache != 0"<<endl;
      }
      return false; 
    }
    return true;
  }

}
