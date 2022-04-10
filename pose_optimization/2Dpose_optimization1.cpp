#include "pose_optimization.h"
#include <../reading_data/matplotlib-cpp/matplotlibcpp.h>
#include <cmath>

using namespace std; 
using namespace gtsam; 
using namespace Eigen; 

namespace plt = matplotlibcpp;

void runBatch(vector<VECTOR_SE2> slamPoses, vector<EDGE_SE2> imuMeasurements){
    NonlinearFactorGraph graph;
    Values initial;
    
    // Add prior
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances(Vector3(1e-6,1e-6,1e-8)); 
    graph.add(PriorFactor<Pose2>(slamPoses.at(0).idx, Pose2(slamPoses.at(0).x,slamPoses.at(0).y,slamPoses.at(0).theta), priorNoise));
    int limit = 20;
    //int limit = imuMeasurements.size();
    // Add vertices and edges
    for (int i = 1; i < limit - 1; i++){ 
    	EDGE_SE2 tempEdge = imuMeasurements.at(i);
    	noiseModel::Gaussian::shared_ptr model = noiseModel::Diagonal::Variances(Vector3(1e-6,1e-6,1e-8)); 
    	graph.add(BetweenFactor<Pose2>(tempEdge.idx-1,tempEdge.idx, Pose2(tempEdge.dx,tempEdge.dy,tempEdge.dtheta), model)); 
    	
    }
    cout << "EDGES added" << endl;
    vector<double> floam_x, floam_y;
    for(int i = 0; i < limit; i++){
    	auto tempPose = slamPoses.at(i);
    	initial.insert(i,Pose2(tempPose.x,tempPose.y,tempPose.theta));
        floam_x.push_back(tempPose.x);
        floam_y.push_back(tempPose.y);
    }
    cout << "Poses added" << endl;
    GaussNewtonParams parameters; 
    parameters.relativeErrorTol = 1e-2; 
    parameters.maxIterations = 100; 
    GaussNewtonOptimizer optimizer(graph,initial,parameters);
    
    Values result = optimizer.optimize();
    // result.print(); 

      // Save results to file
    cout << "finish" << endl;
    printf("\nWriting results to file...\n");
    string output_filename = "optimized_poses.txt";
    FILE* fp_out = fopen(output_filename.c_str(), "w+");
    fprintf(fp_out,
            "#time(s),x(m),y(m),theta(m)\n");
    vector<double> post_x, post_y;
    for (size_t i = 0; i < limit - 1; i++) {

        auto pose = result.at<Pose2>(i);

        cout << "State at #" << i << endl;
        cout << "Pose:" << endl << pose << endl;

        fprintf(fp_out, "%lld, %f,%f,%f\n",
                slamPoses[i].time, pose.x(), pose.y(), pose.theta());
        post_x.push_back(pose.x());
        post_y.push_back(pose.y());
    }
    fclose(fp_out);
    plt::figure(1);
    plt::plot(floam_x,floam_y);
    plt::plot(post_x,post_y);
    plt::title("result");
    //plt::save("Initial_F-LOAN_Point.png");
    
}

void runISAM(vector<VECTOR_SE2> slamPoses, vector<EDGE_SE2> imuMeasurements){

    Values result; 
    ISAM2Params isam_params; 
    isam_params.factorization = ISAM2Params::CHOLESKY; 
    isam_params.relinearizeSkip = 10; 
    ISAM2 isam(isam_params); 
    
    NonlinearFactorGraph graph;
    // Add vertices and edges
    for (int i = 0; i < slamPoses.size(); i++){
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
    double prev_angle = 0;
    double current_angle = 0;
    double current_vel_x = 0;
    double current_vel_y = 0;
    double corr_x = 0;
    double corr_y = 0;
    double dt;

    EDGE_SE2 imuMeasurement;
    string output_filename = "IMU_integrate.txt";
    FILE* fp_out = fopen(output_filename.c_str(), "w+");
    fprintf(fp_out,
            "#idx, dt(s),dx(m),dy(m),dtheta(m)\n");
    vector<double> IMU_x, IMU_y;
    IMU_x.push_back(corr_x);
    IMU_y.push_back(corr_y);
        
    for(int i = 1; i < imuMeasurements_SE3.size() - 1; i++){
        dt = (imuMeasurements_SE3.at(i).time - imuMeasurements_SE3.at(i-1).time) * pow(10, -9);
        current_angle =  (double) atan2(2 * (imuMeasurements_SE3.at(i).q.w() * imuMeasurements_SE3.at(i).q.z() + imuMeasurements_SE3.at(i).q.y() * imuMeasurements_SE3.at(i).q.x()), 1 - 2 * (pow(imuMeasurements_SE3.at(i).q.x(), 2) + pow(imuMeasurements_SE3.at(i).q.z(), 2)));
        if (i == 1) {
            cout << "current angle = " <<  current_angle << endl;
        }
        current_vel_x = prev_vel_x + imuMeasurements_SE3.at(i).accel(0) * dt;
        current_vel_y = prev_vel_y + imuMeasurements_SE3.at(i).accel(1) * dt;

        imuMeasurement.dx = current_vel_x * dt * cos(current_angle) - current_vel_y * dt * sin(current_angle);
        imuMeasurement.dy = current_vel_x * dt * sin(current_angle) + current_vel_y * dt * cos(current_angle);

        
        if (i == 1) {
            imuMeasurement.dtheta = 0;
        } else { 
            imuMeasurement.dtheta = current_angle - prev_angle;
        } 
        imuMeasurement.time = imuMeasurements_SE3.at(i).time;
        imuMeasurement.idx = imuMeasurements_SE3.at(i).idx;

        imuMeasurements.push_back(imuMeasurement);
        prev_vel_x = current_vel_x;
        prev_vel_y = current_vel_y;
        prev_angle = current_angle;

        fprintf(fp_out, "%d, %f, %f, %f, %f\n",
                    imuMeasurement.idx, dt, imuMeasurement.dx, imuMeasurement.dy, imuMeasurement.dtheta);

        corr_x = corr_x + imuMeasurement.dx;
        corr_y = corr_y + imuMeasurement.dy;
        IMU_x.push_back(corr_x);
        IMU_y.push_back(corr_y);
    }
    plt::figure(2);
    plt::plot(IMU_x,IMU_y);
    plt::title("IMU Trajectory");
    //plt::show();
}

int main(const int argc, const char *argv[]) {
    vector<VECTOR_SE3> vertices;
    vector<VECTOR_SE2> slamPoses;
    string slam_data = "./data/refined_tf.txt";
    
    vector<EDGE_SE3> imuMeasurements_SE3;
    vector<EDGE_SE2> imuMeasurements;
       
    string imu_data = "./data/imu.txt";

    KittiCalibration kittiCalibration;
    string imu_metadata = "./data/KittiEquivBiasedImu_metadata.txt";

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
    plt::show();
}
    
