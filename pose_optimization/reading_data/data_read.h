// #ifdef DATA_READ
#define DATA_READ
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>
// GTSAM related includes.
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>


using namespace std;
using namespace Eigen; 
using namespace gtsam;

class VECTOR_SE3{
    public:
        long long int time;
        int idx;
        double x;
        double y;
        double z;
        Eigen::Quaterniond q;
        Eigen::Matrix3d rotationMatrix;
};

class VECTOR_SE2{
    public:
        long long int time;
        int idx;
        double x;
        double y;
        double theta;
};


void read_vector_se3_data(VECTOR_SE3 &vertex_se3, VECTOR_SE2 &vertex_se2, string line);
bool read_se_3_data(vector<VECTOR_SE3> &vertices, string filname);

 struct EDGE_SE3 {
    public: 
      string type; 
      int idx; 
      long long int time; 
      Eigen::Vector3d gyro;
      Eigen::Vector3d accel; 
      Eigen::Quaterniond q; 
      Eigen::Matrix3d rotationMatrix; 
      Eigen::Matrix3d qCov;
      Eigen::Matrix3d gyroCov; 
      Eigen::Matrix3d accelCov;     
 };

  struct EDGE_SE2 {
    public: 
      int idx; 
      long long int time; 
      double dx;
      double dy;
      double dtheta;
 };

 struct KittiCalibration {
  double body_ptx;
  double body_pty;
  double body_ptz;
  double body_prx;
  double body_pry;
  double body_prz;
  double accelerometer_sigma;
  double gyroscope_sigma;
  double integration_sigma;
  double accelerometer_bias_sigma;
  double gyroscope_bias_sigma;
  double average_delta_t;
};


//  #endif