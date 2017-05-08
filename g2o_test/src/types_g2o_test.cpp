#include "types_g2o_test.h"

using namespace g2o; 

void init_types_g2o_test(void)
{
  Factory * factory = Factory::instance(); 
  factory->registerType("EDGE_SE2_LINE2D_ZH", new HyperGraphElementCreator<EdgeSE2Line2DZH>); 
  factory->registerType("EDGE_SE3_PRIOR_ZH", new HyperGraphElementCreator<EdgeSE3PriorZH>);
  factory->registerType("EDGE_SE3_EULER_ZH", new HyperGraphElementCreator<EdgeSE3EulerZH>);
  factory->registerType("EDGE_SE3_EULER2_ZH", new HyperGraphElementCreator<EdgeSE3Euler2ZH>); 
  factory->registerType("VERTEX3_EULER_ZH", new HyperGraphElementCreator<VertexSE3EulerZH>); 
}

G2O_REGISTER_TYPE(EDGE_SE2_LINE2D_ZH, EdgeSE2Line2DZH); 
G2O_REGISTER_TYPE(EDGE_SE3_PRIOR_ZH, EdgeSE3PriorZH);
G2O_REGISTER_TYPE(EDGE_SE3_EULER_ZH, EdgeSE3EulerZH);
G2O_REGISTER_TYPE(EDGE_SE3_EULER2_ZH, EdgeSE3Euler2ZH); 
G2O_REGISTER_TYPE(VERTEX3_EULER_ZH, VertexSE3EulerZH);



