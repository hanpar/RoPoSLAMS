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

    noiseModel::Gaussian::shared_ptr model = noiseModel::Diagonal::Variances(Vector3(4e-6,4e-6,4e-8)); 
    graph.add(BetweenFactor<Pose2>(imuMeasurements.at(0).idx-1,imuMeasurements.at(0).idx, Pose2(imuMeasurements.at(0).dx,imuMeasurements.at(0).dy,imuMeasurements.at(0).dtheta), model)); 
    noiseModel::Gaussian::shared_ptr model = noiseModel::Diagonal::Variances(Vector3(4e-6,4e-6,4e-8)); 
    graph.add(BetweenFactor<Pose2>(imuMeasurements.at(1).idx-1,imuMeasurements.at(1).idx, Pose2(imuMeasurements.at(1).dx,imuMeasurements.at(1).dy,imuMeasurements.at(1).dtheta), model)); 
