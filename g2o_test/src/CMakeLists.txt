CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#message(STATUS "g2o include directory is ${G2O_INCLUDE_DIR}")

ADD_LIBRARY(edges_zh SHARED
	    types_g2o_test.cpp types_g2o_test.h
            edge_se2_line2d_zh.cpp edge_se2_line2d_zh.h
            edge_se3_prior_zh.cpp edge_se3_prior_zh.h
            edge_se3_euler_zh.cpp edge_se3_euler_zh.h
            edge_se3_euler2_zh.cpp edge_se3_euler2_zh.h
            vertex_se3_euler_zh.cpp vertex_se3_euler_zh.h
            ) 

target_link_libraries(edges_zh ${G2O_ALL_LIBS} ${EIGEN3_LIBRARIES})

SET_TARGET_PROPERTIES(edges_zh PROPERTIES OUTPUT_NAME 
  ${EDGE_ZH_LIB_PREFIX})


ADD_LIBRARY(edges_zh2 SHARED
	    types_g2o_test.cpp types_g2o_test.h
            edge_se3_prior_zh.cpp edge_se3_prior_zh.h
            edge_se3_euler_zh.cpp edge_se3_euler_zh.h
            edge_se3_euler2_zh.cpp edge_se3_euler2_zh.h
            vertex_se3_euler_zh.cpp vertex_se3_euler_zh.h
            ) 
          #target_link_libraries(edges_zh2 ${G2O_ALL_LIBS} ${EIGEN3_LIBRARIES})
          
