#define READ_TF
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>



using namespace std;
using namespace Eigen; 

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


void read_vector_se3_data(VECTOR_SE3 &vertex_se3, string line);
bool read_se_3_data(vector<VECTOR_SE3> &vertices, string filname);

 struct ImuMeasurement {
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