#include "pose_optimization.h"

using namespace std; 
using namespace gtsam; 
using namespace Eigen; 

void runBatch(vector<VECTOR_SE3> slamPoses, vector<EDGE_SE3> imuMeasurements){
    NonlinearFactorGraph graph;
    Values initial;
    
    // Add prior
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances(Vector3(1e-6,1e-6,1e-8)); 
    graph.add(PriorFactor<Pose2>(slamPoses.at(0).idx, Pose2(slamPoses.at(0).x,slamPoses.at(0).y,slamPoses.at(0).theta), priorNoise));
    
    // Add vertices and edges
    for (int i = 0; i < imuMeasurements.size(); i++){ 
    	EDGE_SE3 tempEdge = imuMeasurements.at(i);
    	noiseModel::Gaussian::shared_ptr model = noiseModel::Gaussian::Covariance(Vector3(1e-6,1e-6,1e-8)));
    	graph.add(BetweenFactor<Pose2>(tempEdge.idx,tempEdge.idx+1,Pose2(tempEdge.x,tempEdge.y,tempEdge.theta), model)); 
    	
    }

    for(int i = 0; i < slamPoses.size(); i++){
    	Pose tempPose = slamPoses.at(i);
    	initial.insert(tempPose.idx,Pose2(tempPose.x,tempPose.y,tempPose.theta));
    }
    
    GaussNewtonParams parameters; 
    parameters.relativeErrorTol = 1e-5; 
    parameters.maxIterations = 100; 
    GaussNewtonOptimizer optimizer(graph,initial,parameters);
    
    Values result = optimizer.optimize();
    result.print(); 
}

void runISAM(vector<VECTOR_SE3> slamPoses, vector<EDGE_SE3> imuMeasurements){

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
    	// Add prior
    	noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances(Vector3(1e-6,1e-6,1e-8)); 
    	graph.add(PriorFactor<Pose2>(slamPoses.at(0).idx, Pose2(slamPoses.at(0).x,slamPoses.at(0).y,slamPoses.at(0).theta), priorNoise));
    		initial.insert(slamPoses.at(0).idx,Pose2(slamPoses.at(0).x,slamPoses.at(0).y,slamPoses.at(0).theta));
    	}
    	else{
    	   initial.insert(i,result.at(i-1)); 
    	   for(int j = 0; j < imuMeasurements.size(); j++){
    	   	VECTOR_SE3 imuMeasurements = edges.at(j);
    	   	if (i==tempEdge.j){
	    	noiseModel::Gaussian::shared_ptr model = noiseModel::Gaussian::Covariance(Vector3(0.05,0.05,0.05));
	    	graph.add(BetweenFactor<Pose2>(tempEdge.idx,tempEdge.idx+1,Pose2(tempEdge.x,tempEdge.y,tempEdge.theta), model)); 
    	   	}
    	   }
    	}    	
    	isam.update(graph,initial); 
    	result = isam.calculateEstimate(); 
    }
    result.print(); 
}

int main(const int argc, const char *argv[]) {
    vector<VECTOR_SE3> slamPoses;
    string slam_data = "../data/refined_tf.txt";
    
    vector<EDGE_SE3> imuMeasurements;   
    string imu_data = "../data/imu.txt";

    KittiCalibration kittiCalibration;
    string imu_metadata = "../data/KittiEquivBiasedImu_metadata.txt";

    //int k = 0; 
    if (read_se_3_data(slamPoses, slam_data))
    {
        cout << "SLAM Data Read Successfully!" << endl; 
    }
    else exit(1);
    
    if (loadKittiData(imuMeasurements, kittiCalibration, imu_data, imu_metadata))
    {
        cout << "IMU Data Read Successfully!" << endl;
    } 
    else exit(1);
    
    runISAM(slamPoses, imuMeasurements); 
}
    
