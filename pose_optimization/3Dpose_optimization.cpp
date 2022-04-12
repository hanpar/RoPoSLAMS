#include "pose_optimization.h"


using namespace std; 
using namespace gtsam; 
using namespace Eigen; 

namespace plt = matplotlibcpp;

void runBatch(vector<VECTOR_SE3> slamPoses, vector<EDGE_SE3> imuMeasurements){
    NonlinearFactorGraph graph;
    Values initial;
    
    // Add prior
    auto priorModel = noiseModel::Diagonal::Variances(
      (Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
    graph.add(PriorFactor<Pose3>(1, Pose3(), priorModel));
    
    // Add vertices and edges
    double prev_vel_x = 5;
    double prev_vel_y = 0;
    double current_pitch = 0;
    double current_roll = 0;
    double current_yaw = 0;
    double prev_pitch = 0, prev_roll = 0, prev_yaw = 1.8; 
    double current_vel_x = 0;
    double dt, dx, dy, dz;
    double droll, dpitch, dyaw;

    int limit = imuMeasurements.size();
    cout << "Imu measurement = " << limit;
    cout << "   SLAM measurement = " << slamPoses.size() << endl;

    for (int i = 2; i < limit - 1; i++){ 
        auto euler = imuMeasurements.at(i).q.toRotationMatrix().eulerAngles(0,1,2);
        current_roll = euler[0]; 
        current_pitch = euler[1];
        current_yaw = euler[2];
        dt = (imuMeasurements.at(i).time - imuMeasurements.at(i-1).time) * pow(10, -9);
        current_vel_x = prev_vel_x + imuMeasurements.at(i).accel(0) * dt;
        dx = current_vel_x * dt;
        dy = 0; dz = 0; 
        droll = (current_roll - prev_roll) * dt; 
        dpitch = (current_pitch - prev_pitch) * dt;
        dyaw = (current_yaw - prev_yaw) * dt;

    	EDGE_SE3 tempEdge = imuMeasurements.at(i);
    	noiseModel::Diagonal::shared_ptr model = noiseModel::Diagonal::Variances(
      (Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
        
        Point3 t(dx,dy,dz);
    	graph.add(BetweenFactor<Pose3>(tempEdge.idx-1,tempEdge.idx, Pose3(Rot3(tempEdge.rotationMatrix),t), model)); 
        prev_vel_x = current_vel_x;
        prev_roll = current_roll; 
        prev_pitch = current_pitch;
        prev_yaw = current_yaw;
    	
    }
    cout << "EDGES added " << endl;
        
    vector<double> floam_x, floam_y, floam_z;
    limit = slamPoses.size();
    for(int i = 1; i < limit; i++){
    	auto tempPose = slamPoses.at(i);
        Point3 t(tempPose.x,tempPose.y,tempPose.z);
    	initial.insert(i, Pose3(Rot3(tempPose.rotationMatrix),t));
        floam_x.push_back(tempPose.x);
        floam_y.push_back(tempPose.y);
        floam_z.push_back(tempPose.z);
    }
    cout << "Poses added " << endl;
    GaussNewtonParams parameters; 
    parameters.relativeErrorTol = 1e-5; 
    parameters.maxIterations = 10000; 
    GaussNewtonOptimizer optimizer(graph,initial,parameters);
    // LevenbergMarquardtOptimizer optimizer(graph, initial);
    
    Values result = optimizer.optimize();
    //result.print(); 
    vector<double> post_x, post_y, post_z;
    //   Save results to file
    printf("\nWriting results to file...\n");
    string output_filename = "optimized_poses_without_imu.txt";
    FILE* fp_out = fopen(output_filename.c_str(), "w+");
    fprintf(fp_out,
            "#time(s),x(m),y(m),theta(m)\n");

    for (size_t i = 1; i < limit - 1; i++) {

        auto pose = result.at<Pose3>(i);

        //cout << "State at #" << i << endl;
        //cout << "Pose:" << endl << pose << endl;

        fprintf(fp_out, "%lld, %f,%f,%f\n",
                slamPoses[i].time, pose.x(), pose.y(), pose.z());
        post_x.push_back(pose.x());
        post_y.push_back(pose.y());
        post_z.push_back(pose.z());
    }
    //cout << "slamePose size" << slamPoses.size() << endl;

    cout << "Size of post x = " << post_x.size() << " y = " << post_y.size() << " z = " << post_z.size() << endl; 
    cout << "Size of post x = " << floam_x.size() << " y = " << floam_y.size() << " z = " << floam_z.size() << endl;
    fclose(fp_out);
    plt::figure(1);
    plt::plot3(post_x,post_y,post_z);
    plt::title("Post Process");
    plt::figure(2);
    plt::plot3(floam_x,floam_y,floam_z);
    plt::title("Floam Initial");
    plt::show();
}

int main(const int argc, const char *argv[]) {
    vector<VECTOR_SE3> vertices;
    vector<VECTOR_SE2> slamPoses;
    string slam_data = "../data/refined_tf.txt";
    
    vector<EDGE_SE3> imuMeasurements_SE3;
    vector<EDGE_SE2> imuMeasurements;
       
    string imu_data = "../data/imu.txt";

    KittiCalibration kittiCalibration;
    string imu_metadata = "../data/KittiEquivBiasedImu_metadata.txt";

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
    else exit(1);

    //integrateIMUData(imuMeasurements_SE3, imuMeasurements);
    runBatch(vertices, imuMeasurements_SE3); 
}
    
