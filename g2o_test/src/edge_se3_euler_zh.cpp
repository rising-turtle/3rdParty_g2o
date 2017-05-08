#include "edge_se3_euler_zh.h"
#include "g2o/core/factory.h"
#include <iostream>

namespace g2o{

  void jac_quat3_euler3(Eigen::Matrix<double, 6, 6, Eigen::ColMajor>& J, const Isometry3D& t)
  {
    Vector7d t0 = g2o::internal::toVectorQT(t);

    double delta=1e-6;
    double idelta= 1. / (2. * delta);

    Vector7d ta = t0;
    Vector7d tb = t0;
    for (int i=0; i<6; i++){
      ta=tb=t0;
      ta[i]-=delta;
      tb[i]+=delta;
      Vector6d ea = g2o::internal::toVectorET(g2o::internal::fromVectorQT(ta));
      Vector6d eb = g2o::internal::toVectorET(g2o::internal::fromVectorQT(tb));
      // J.col(3)=(eb-ea)*idelta;
      J.col(i) = (eb-ea)*idelta;
    }
  }

  bool EdgeSE3EulerZH::read(std::istream& is)
  {
    Vector6d meas;
    for (int i=0; i<6; i++)
      is  >> meas[i];
    Isometry3D transf= g2o::internal::fromVectorET(meas);
    Matrix<double, 6, 6, Eigen::ColMajor> infMatEuler;
    for (int i=0; i<6; i++)
      for (int j=i; j<6; j++) {
        is >> infMatEuler(i,j);
        if (i!=j)
          infMatEuler(j,i) = infMatEuler(i,j);
      }
    Matrix<double, 6, 6, Eigen::ColMajor> J;
    jac_quat3_euler3(J, transf);
    // Matrix<double, 6, 6, Eigen::ColMajor> infMat = J.transpose() * infMatEuler * J;
    Matrix<double, 6, 6, Eigen::ColMajor> infMat = J * infMatEuler * J.transpose();
    setMeasurement(transf);
    setInformation(infMat);
    return true;
  }

  bool EdgeSE3EulerZH::write(std::ostream& os) const
  {
    Vector6d meas = g2o::internal::toVectorET(_measurement);
    for (int i=0; i<6; i++)
      os << meas[i] << " ";

    Matrix<double, 6, 6, Eigen::ColMajor> J;
    jac_quat3_euler3(J, measurement());
    // cout<<"edge_se3_euler_zh.cpp: compute J: "<<endl<<J<<endl;
    //HACK: invert the jacobian to simulate the inverse derivative
    J=J.inverse();
    // Matrix<double, 6, 6, Eigen::ColMajor> infMatEuler = J.transpose()*information()*J;
    Matrix<double, 6, 6, Eigen::ColMajor> infMatEuler = J*information()*J.transpose();
    // cout<<"edge_se3_euler_zh.cpp: compute infMatEuler: "<<endl<<infMatEuler<<endl;
    for (int i=0; i<6; i++)
      for (int j=i; j<6; j++){
        os << " " <<  infMatEuler(i,j);
      }
    return os.good();
  }
}
