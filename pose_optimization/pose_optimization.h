// #ifdef POSE_OPTIMIZATION
#define POSE_OPTIMIZATION

#include "reading_data/data_read.cpp"
#include <gtsam/geometry/Pose3.h>
// We will use simple integer Keys to refer to the robot poses.
#include <gtsam/inference/Key.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <cstring>
#include <fstream>
#include <iostream>
#include <math.h>
#include <cmath>
#include "reading_data/matplotlib-cpp/matplotlibcpp.h"

using namespace std;
using namespace gtsam;
using namespace Eigen; 

namespace plt = matplotlibcpp;

using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)



// #endif