#include "pose_optimization.h"
#include "../../matplotlib-cpp/matplotlibcpp.h"

using namespace std; 
using namespace gtsam; 
using namespace Eigen; 
namespace plt = matplotlibcpp; 

int main(){

    /*vector<VECTOR_SE3> vertices;
    vector<VECTOR_SE2> slamPoses;
    string slam_data = "/home/hjpa/Documents/eecs568-group17-project/pose_optimization/data/refined_tf.txt";
    
    vector<EDGE_SE3> imuMeasurements_SE3;
    vector<EDGE_SE2> imuMeasurements;
       
    string imu_data = "./home/hjpa/Documents/eecs568-group17-project/pose_optimization/data/imu.txt";

    KittiCalibration kittiCalibration;
    string imu_metadata = "/home/hjpa/Documents/eecs568-group17-project/pose_optimization/data/KittiEquivBiasedImu_metadata.txt";

    int i = 10; 
    if (read_se_3_data(vertices, slamPoses, slam_data))
    {
        // cout << vertices.at(i).q.x() << ", " << vertices.at(i).q.y() << ", " << vertices.at(i).q.z() << ", " << vertices.at(i).q.w() << endl;
        // cout << "IDX = " << vertices_se2.at(i).idx << ", Time = " << vertices_se2.at(i).time << ", X = " << vertices_se2.at(i).x << ", Y = " << vertices_se2.at(i).y << ", Theta = " << vertices_se2.at(i).theta << endl;
    }
    else exit(1);
    
    if (loadKittiData(imuMeasurements_SE3, kittiCalibration, imu_data, imu_metadata))
    {
        cout << "IMU Data Read Successfully!" << endl;
    } 
    else exit(1);*/

    plt::plot({1,2,3,4});
    plt::show(); 

}