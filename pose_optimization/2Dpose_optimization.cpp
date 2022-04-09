#include "pose_optimization.h"

using namespace std; 
using namespace gtsam; 
using namespace Eigen; 

void runBatch(vector<VECTOR_SE2> slamPoses, vector<EDGE_SE2> imuMeasurements){
    NonlinearFactorGraph graph;
    Values initial;
    
    // Add prior
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances(Vector3(1e-6,1e-6,1e-8)); 
    graph.add(PriorFactor<Pose2>(1, Pose2(0, 0, 0), priorNoise));
    
    // Add vertices and edges
    int limit = imuMeasurements.size();
    cout << "Imu measurement = " << limit;
    cout << "SLAM measurement = " << slamPoses.size();
    
    // int limit = 100;
    for (int i = 2; i < limit - 1; i++){ 
    	EDGE_SE2 tempEdge = imuMeasurements.at(i);
    	noiseModel::Gaussian::shared_ptr model = noiseModel::Diagonal::Variances(Vector3(4e-6,4e-6,4e-8)); 
    	graph.add(BetweenFactor<Pose2>(tempEdge.idx-1,tempEdge.idx, Pose2(tempEdge.dx,tempEdge.dy,tempEdge.dtheta), model)); 
    	
    }
    cout << "EDGES added" << endl;

    limit = slamPoses.size();
    // limit = 50;
    for(int i = 1; i < limit; i++){
    	auto tempPose = slamPoses.at(i);
    	initial.insert(i, Pose2(tempPose.x,tempPose.y,tempPose.theta));
    }
    cout << "Poses added" << endl;
    GaussNewtonParams parameters; 
    parameters.relativeErrorTol = 1e-5; 
    parameters.maxIterations = 10000; 
    GaussNewtonOptimizer optimizer(graph,initial,parameters);
    // LevenbergMarquardtOptimizer optimizer(graph, initial);
    
    Values result = optimizer.optimize();
    result.print(); 

    //   Save results to file
    printf("\nWriting results to file...\n");
    string output_filename = "optimized_poses_without_imu.txt";
    FILE* fp_out = fopen(output_filename.c_str(), "w+");
    fprintf(fp_out,
            "#time(s),x(m),y(m),theta(m)\n");

    for (size_t i = 1; i < slamPoses.size() - 1; i++) {

        auto pose = result.at<Pose2>(i);

        cout << "State at #" << i << endl;
        cout << "Pose:" << endl << pose << endl;

        fprintf(fp_out, "%lld, %f,%f,%f\n",
                slamPoses[i].time, pose.x(), pose.y(), pose.theta());
    }

    fclose(fp_out);
}

void runISAM(vector<VECTOR_SE2> slamPoses, vector<EDGE_SE2> imuMeasurements){

    Values result; 
    ISAM2Params isam_params; 
    isam_params.factorization = ISAM2Params::CHOLESKY; 
    isam_params.relinearizeSkip = 10; 
    ISAM2 isam(isam_params); 
    
    NonlinearFactorGraph graph;
    // Add vertices and edges
    for (int i = 0; i < slamPoses.size() - 1; i++){
    	graph.resize(0); 
    	Values initial; 
    	if (i == 0){
    	noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances(Vector3(1e-6,1e-6,1e-8)); 
    	graph.add(PriorFactor<Pose2>(slamPoses.at(0).idx, Pose2(slamPoses.at(0).x,slamPoses.at(0).y,slamPoses.at(0).theta), priorNoise));
    		initial.insert(slamPoses.at(0).idx, Pose2(slamPoses.at(0).x,slamPoses.at(0).y,slamPoses.at(0).theta));
    	}
    	else
        {
    	   initial.insert(i,result.at(i-1)); 
    	//    for(int j = 0; j < imuMeasurements.size(); j++){
    	   	EDGE_SE2 tempEdge = imuMeasurements.at(i);
            
    	   	// if (i==tempEdge.idx+1){
	    	noiseModel::Gaussian::shared_ptr model = noiseModel::Gaussian::Covariance(Vector3(0.05,0.05,0.05));
	    	graph.add(BetweenFactor<Pose2>(i-1, i, Pose2(tempEdge.dx,tempEdge.dy,tempEdge.dtheta), model)); 
    	   	// }
    	//    }
    	}    	
    	isam.update(graph,initial); 
    	result = isam.calculateEstimate(); 
    }
    result.print(); 
}

void integrateIMUData(vector<EDGE_SE3> &imuMeasurements_SE3, vector<EDGE_SE2> &imuMeasurements){

    double prev_vel_x = 0;
    double prev_vel_y = 0;
    double current_vel_x = 0;
    double current_vel_y = 0;
    double dt;

    EDGE_SE2 imuMeasurement;
    // int limit = imuMeasurements_SE3.size() ;
    int limit = 100;
    for(int i = 1; i < limit; i++){
        dt = (imuMeasurements_SE3.at(i).time - imuMeasurements_SE3.at(i-1).time) * pow(10, -9);
        current_vel_x = prev_vel_x + imuMeasurements_SE3.at(i).accel(0) * dt;
        current_vel_y = prev_vel_y + imuMeasurements_SE3.at(i).accel(1) * dt;

        imuMeasurement.dx = current_vel_x * dt;
        imuMeasurement.dy = current_vel_y * dt;

        imuMeasurement.dtheta =  (double) atan2(2 * (imuMeasurements_SE3.at(i).q.w() * imuMeasurements_SE3.at(i).q.z() + imuMeasurements_SE3.at(i).q.y() * imuMeasurements_SE3.at(i).q.x()), 1 - 2 * (pow(imuMeasurements_SE3.at(i).q.x(), 2) + pow(imuMeasurements_SE3.at(i).q.z(), 2))) * dt;
        imuMeasurement.time = imuMeasurements_SE3.at(i).time;
        imuMeasurement.idx = imuMeasurements_SE3.at(i).idx;

        // cout << "dx = " << imuMeasurement.dx << " dy = " << imuMeasurement.dy << " dtheta = " << imuMeasurement.dtheta <<endl;

        imuMeasurements.push_back(imuMeasurement);
        prev_vel_x = current_vel_x;
        prev_vel_y = current_vel_y;
    }

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

    integrateIMUData(imuMeasurements_SE3, imuMeasurements);
    cout << "Inetegrated IMU" <<endl;
    runBatch(slamPoses, imuMeasurements); 
}
    
