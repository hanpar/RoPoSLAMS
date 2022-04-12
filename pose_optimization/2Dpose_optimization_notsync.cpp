#include "pose_optimization.h"
#include "reading_data/matplotlib-cpp/matplotlibcpp.h"
#include <cmath>

using namespace std; 
using namespace gtsam; 
using namespace Eigen; 

namespace plt = matplotlibcpp;

void runBatch(vector<VECTOR_SE2> slamPoses, vector<EDGE_SE2> imuMeasurements,  vector<VECTOR_SE2> gt, vector<IMU_CORR> imu_corr){
    NonlinearFactorGraph graph;
    Values initial;

    cout << "Imu measurement = " << imuMeasurements.size() << " GPS measurement = " << gt.size();
    cout << " SLAM measurement = " << slamPoses.size() << endl;
    
    // Add prior
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances(Vector3(1e-6,1e-6,1e-6)); 
    graph.add(PriorFactor<Pose2>(0, Pose2(0.0, 0.0, 0.0), priorNoise));

    vector<double> floam_x, floam_y;
    int limit = slamPoses.size();
    // limit = 50;
    for(int i = 0; i < limit; i++){
    	auto tempPose = slamPoses.at(i);
        // cout << tempPose.idx << endl;
    	initial.insert(tempPose.idx, Pose2(tempPose.x,tempPose.y,tempPose.theta));
        floam_x.push_back(tempPose.x);
        floam_y.push_back(tempPose.y);
    }
    cout << "Poses added! Last Pose idx = " << slamPoses.at(limit-1).idx <<  endl;
    
    //limit = gt.size();
    //VECTOR_SE2 prior = slamPoses.at(0);
    //graph.add(PriorFactor<Pose2>(prior.at(0).idx, Pose2(prior), priorNoise));
    int slamIdx = 0;
    for(int i = 0; i < limit; i++){
        auto gpsPose = slamPoses.at(i);
        if(slamPoses.at(slamIdx).time <= gpsPose.time){
            // cout << slamIdx;
            graph.add(PriorFactor<Pose2>(slamPoses.at(slamIdx).idx, Pose2(gpsPose.x, gpsPose.y, slamPoses.at(slamIdx).theta), priorNoise));
            slamIdx++;
            // break;
        }
    	// cout << tempPose.idx << endl;
        // graph.add(PriorFactor<Pose2>(i, Pose2(gpsPose.x, gpsPose.y, gpsPose.theta), priorNoise));
    	// initial.insert(tempPose.idx, Pose2(tempPose.x,tempPose.y,tempPose.theta));
        // floam_x.push_back(tempPose.x);
        // floam_y.push_back(tempPose.y);
    }
    cout << "Slampose priors added! " <<  endl;
    
    //  for(int i = 2; i < limit; i++){
    // 	auto tempPose = slamPoses.at(i);
    //     graph.add(PriorFactor<Pose2>(i, Pose2(tempPose.x,tempPose.y,tempPose.theta), priorNoise));
    // }   
    
    limit = imuMeasurements.size();
    for (int i = 1; i < limit; i++){ 
    	EDGE_SE2 tempEdge = imuMeasurements.at(i);
        //double num_integration = (imuMeasurements.at(i).time - imuMeasurements.at(i-1).time) / 0.1;
        double num_integration = 0.1;
        // cout << num_integration << endl;
        // break;
        // The more numbers of integrations, the higher covariance value
    	noiseModel::Gaussian::shared_ptr model = noiseModel::Diagonal::Variances(Vector3(1e-3 * num_integration, 1e-3 * num_integration, 1e-4 * num_integration)); 
    	//graph.add(BetweenFactor<Pose2>(tempEdge.idx-1,tempEdge.idx, Pose2(tempEdge.dx,tempEdge.dy,tempEdge.dtheta), model)); 
        graph.add(BetweenFactor<Pose2>(i,i - 1, Pose2(tempEdge.dx,tempEdge.dy,tempEdge.dtheta), model)); 
    }

    cout << "EDGES added! Last Pose idx = " << imuMeasurements.at(limit-1).idx << endl;

    GaussNewtonParams parameters; 
    parameters.relativeErrorTol = 1e-4; 
    parameters.maxIterations = 10000; 
    GaussNewtonOptimizer optimizer(graph,initial,parameters);
    // LevenbergMarquardtOptimizer optimizer(graph, initial);
    
    Values result = optimizer.optimize();
    //result.print(); 
    vector<double> post_x, post_y;
    //   Save results to file
    printf("\nWriting results to file...\n");
    string output_filename = "optimized_poses_with_imu.txt";
    FILE* fp_out = fopen(output_filename.c_str(), "w+");
    fprintf(fp_out,
            "#time(s),x(m),y(m),theta(m)\n");

    for (size_t i = 1; i < limit - 1; i++) {

        auto pose = result.at<Pose2>(i);

        //cout << "State at #" << i << endl;
        //cout << "Pose:" << endl << pose << endl;

        fprintf(fp_out, "%lld, %f,%f,%f\n",
                slamPoses[i].time, pose.x(), pose.y(), pose.theta());
        post_x.push_back(pose.x());
        post_y.push_back(pose.y());
    }
    //cout << "slamePose size" << slamPoses.size() << endl;
    vector<double> gt_x,gt_y;
    for (int i = 0; i < gt.size(); i++) {
        gt_x.push_back(gt.at(i).x);
        gt_y.push_back(gt.at(i).y);
    }
    vector<double> imu_x,imu_y;
    for (int i = 0; i < imu_corr.size(); i++) {
        imu_x.push_back(imu_corr.at(i).IMU_x);
        imu_y.push_back(imu_corr.at(i).IMU_y);
    }
    fclose(fp_out);
    plt::figure(1);
    plt::plot(floam_x,floam_y,{{"label", "FLOAM"}});
    plt::plot(post_x,post_y,{{"label", "Batch Result"}});
    plt::plot(imu_x,imu_y,{{"label", "IMU"}});
    plt::plot(gt_x,gt_y,{{"label", "Ground Truth"}});
    plt::title("FLOAM Post Process with IMU");
    plt::legend();
    plt::save("FLOAM_Post_Process_result.png");
    plt::figure(4);
    plt::plot(floam_x,floam_y,{{"label", "FLOAM"}});
    plt::plot(post_x,post_y,{{"label", "Batch Result"}});
    plt::title("FLOAM vs Batch");
    plt::save("result_FLOAMvsBatch.png");
    /*plt::figure(2);
    plt::plot(floam_x,floam_y,"-s");
    plt::title("Floam Initial");
    plt::show();*/
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
    	noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances(Vector3(1e-8,1e-8,1e-8)); 
    	graph.add(PriorFactor<Pose2>(slamPoses.at(0).idx, Pose2(slamPoses.at(0).x,slamPoses.at(0).y,slamPoses.at(0).theta), priorNoise));
    		initial.insert(slamPoses.at(0).idx, Pose2(slamPoses.at(0).x,slamPoses.at(0).y,slamPoses.at(0).theta));
    	}
    	else
        {
    	   initial.insert(i,result.at(i-1)); 
    	//    for(int j = 0; j < imuMeasurements.size(); j++){
    	   	EDGE_SE2 tempEdge = imuMeasurements.at(i);
            
    	   	// if (i==tempEdge.idx+1){
	    	noiseModel::Gaussian::shared_ptr model = noiseModel::Gaussian::Covariance(Vector3(0.5,0.5,0.001));
	    	graph.add(BetweenFactor<Pose2>(i-1, i, Pose2(tempEdge.dx,tempEdge.dy,tempEdge.dtheta), model)); 
    	   	// }
    	//    }
    	}    	
    	isam.update(graph,initial); 
    	result = isam.calculateEstimate(); 
    }
    //result.print(); 
}

void integrateIMUData(vector<VECTOR_SE3> &imuMeasurements_SE3, vector<EDGE_SE2> &imuMeasurements, vector<IMU_CORR> &imu_corr,vector<VECTOR_SE2> &slamPoses){

    double prev_x = 0;
    double prev_y = 0;
    double prev_angle = 0;
    //double move_x = 0;
    //double move_y = 0;
    double current_angle = 0.01;
    double current_move_x = 0;
    double current_move_y = 0;
    double corr_x = 0;
    double corr_y = 0;
    double dt;
    double dx = 0;
    double dy = 0;
    double dtheta = 0;

    EDGE_SE2 imuMeasurement;
    string output_filename = "IMU_integrate.txt";
    FILE* fp_out = fopen(output_filename.c_str(), "w+");
    fprintf(fp_out,
            "#idx, dt(s),dx(m),dy(m),dtheta(m)\n");
    vector<double> IMU_x, IMU_y;
    IMU_CORR imu;
    IMU_x.push_back(corr_x);
    IMU_y.push_back(corr_y);
    // int limit = imuMeasurements_SE3.size();
    int limit = slamPoses.size();
    //int limit = 200;
    int imuIdx = 1;
    for(int i = 0; i < limit - 1; i++){
        while(slamPoses.at(i).time >= imuMeasurements_SE3.at(imuIdx).time && imuIdx < imuMeasurements_SE3.size()){
            //dt = (imuMeasurements_SE3.at(imuIdx).time - imuMeasurements_SE3.at(imuIdx-1).time) * pow(10, -9);
            //current_angle =  (double) atan2(2 * (imuMeasurements_SE3.at(imuIdx).q.w() * imuMeasurements_SE3.at(imuIdx).q.z() + imuMeasurements_SE3.at(imuIdx).q.y() * imuMeasurements_SE3.at(imuIdx).q.x()), 1 - 2 * (pow(imuMeasurements_SE3.at(imuIdx).q.x(), 2) + pow(imuMeasurements_SE3.at(imuIdx).q.z(), 2)));
            current_move_x = imuMeasurements_SE3.at(imuIdx).x - prev_x;
            current_move_y = imuMeasurements_SE3.at(imuIdx).y - prev_y;
            imuIdx++;
        }
        dx = (double) sqrt(pow(current_move_x,2) + pow(current_move_y,2));
        //current_vel_y = prev_vel_y + imuMeasurements_SE3.at(i).accel(1) * dt;
        // prev_vel_y = current_vel_y;
        dy = 0;
        current_angle = (double) atan2(current_move_y,current_move_x);
        imuMeasurement.dx = dx;// * cos(current_angle); - current_vel_y * dt * sin(current_angle);
        imuMeasurement.dy = dy;//current_vel_x * dt// * sin(current_angle); //+ current_vel_y * dt * cos(current_angle);
        //imuMeasurement.dtheta =  current_angle - prev_angle;//(double) atan2(2 * (imuMeasurements_SE3.at(i).q.w() * imuMeasurements_SE3.at(i).q.z() + imuMeasurements_SE3.at(i).q.y() * imuMeasurements_SE3.at(i).q.x()), 1 - 2 * (pow(imuMeasurements_SE3.at(i).q.x(), 2) + pow(imuMeasurements_SE3.at(i).q.z(), 2))) * dt;
        imuMeasurement.dtheta = current_angle - prev_angle;
        imuMeasurement.time = imuMeasurements_SE3.at(imuIdx).time;
        imuMeasurement.idx = i;
        //prev_angle = current_angle;

        corr_x += dx * cos(current_angle);
        corr_y += dx * sin(current_angle);
        IMU_x.push_back(corr_x);
        IMU_y.push_back(corr_y);
        imu.IMU_x = corr_x;
        imu.IMU_y = corr_y;
        imu_corr.push_back(imu);
        dx = 0;
        prev_x = imuMeasurements_SE3.at(imuIdx-1).x;
        prev_y = imuMeasurements_SE3.at(imuIdx-1).y;
        // cout << "dx = " << imuMeasurement.dx << " dy = " << imuMeasurement.dy << " dtheta = " << imuMeasurement.dtheta <<endl;

        imuMeasurements.push_back(imuMeasurement);
        
        fprintf(fp_out, "%d, %f, %f, %f, %f\n",
                    imuMeasurement.idx, dt, imuMeasurement.dx, imuMeasurement.dy, imuMeasurement.dtheta);
    }
    // cout << "In IMU Integrate \n";
    // cout << "IMU Mesurement size = " << imuMeasurements.size() << " SLAM Measurement Size = " << slamPoses.size() << endl;
    plt::figure(2);
    plt::plot(IMU_x,IMU_y);
    plt::title("IMU Trajectory");
}

void Calculate_GroundTrue(vector<GPS_DATA> &gpsMeasurements, vector<GROUND_TRUE> &gt) {
    double pre_x = 0;
    double pre_y = 0;
    double pre_z = 0;
    GROUND_TRUE gt_point;
    vector<double> x,y,z;
    bool First = true;
    for (int i = 0; i < gpsMeasurements.size(); i++) {
        if (i == 0) {
            //pre_x = 6371007.2*sin(gpsMeasurements.at(i).latitude)*sin(gpsMeasurements.at(i).longitude);
            //pre_y = 6371007.2*cos(gpsMeasurements.at(i).latitude)*sin(gpsMeasurements.at(i).longitude);
            pre_x = gpsMeasurements.at(i).longitude;
            pre_y = gpsMeasurements.at(i).latitude;
            pre_z = gpsMeasurements.at(i).altitude;
            gt_point.time = gpsMeasurements.at(i).time;
            gt_point.idx = gpsMeasurements.at(i).idx;
            gt_point.gt_x = 0;
            gt_point.gt_y = 0;
            gt_point.gt_z = 0;
            First = false;
        } else {
            //gt_point.gt_x = 6371007.2*sin(gpsMeasurements.at(i).latitude)*sin(gpsMeasurements.at(i).longitude) - pre_x;
            //gt_point.gt_y = 6371007.2*cos(gpsMeasurements.at(i).latitude)*sin(gpsMeasurements.at(i).longitude) - pre_y;
            gt_point.gt_x = (gpsMeasurements.at(i).longitude - pre_x) * 72880;
            gt_point.gt_y = (gpsMeasurements.at(i).latitude - pre_y) * 111111;
            gt_point.gt_z = gpsMeasurements.at(i).altitude - pre_z;
            gt_point.time = gpsMeasurements.at(i).time;
            gt_point.idx = gpsMeasurements.at(i).idx;
        }
        gt.push_back(gt_point);
        x.push_back(gt_point.gt_x);
        y.push_back(gt_point.gt_y);
        z.push_back(gt_point.gt_z);
    }
    plt::figure(3);
    plt::plot(x,y);
    plt::title("GPS Ground Ture");
}

int main(const int argc, const char *argv[]) {
    vector<VECTOR_SE3> vertices;
    vector<VECTOR_SE2> slamPoses;
    //string slam_data = "./data/aloam_kitti_0018.txt";
    string slam_data = "./data/floam_pose_kitti_2011_09_30_drive_notsync.txt";

    vector<VECTOR_SE3> imuMeasurements_SE3;
    vector<VECTOR_SE2> imuMeasurements;
    vector<EDGE_SE2> imu_edge;
    vector<IMU_CORR> IMU_corr;
       
    string imu_data = "./data/floam_imu_kitti_2011_09_30_drive_notsync.txt";

    //KittiCalibration kittiCalibration;
    //string imu_metadata = "./data/old_data/data_latest_runs/KittiEquivBiasedImu_metadata.txt";

    vector<VECTOR_SE3> GT_SE3;
    vector<VECTOR_SE2> GT_SE2;
    string gt_data = "./data/floam_tf_kitti_2011_09_30_drive_notsync.txt";

     bool useALOAMCorrection = false;

    if (read_se_3_data_new(vertices, slamPoses, slam_data, useALOAMCorrection))
    {   
        cout << "SLAM Poses Read Successfully! " << slamPoses.size() <<endl;
        // cout << vertices.at(i).q.x() << ", " << vertices.at(i).q.y() << ", " << vertices.at(i).q.z() << ", " << vertices.at(i).q.w() << endl;
        // cout << "IDX = " << vertices_se2.at(i).idx << ", Time = " << vertices_se2.at(i).time << ", X = " << vertices_se2.at(i).x << ", Y = " << vertices_se2.at(i).y << ", Theta = " << vertices_se2.at(i).theta << endl;
    }
    else exit(1);
    
    if (read_se_3_data_new(imuMeasurements_SE3, imuMeasurements, imu_data))
    {
        cout << "IMU Data Read Successfully! " << imuMeasurements_SE3.size() <<  endl;
    } 
    else exit(1);

    if (read_se_3_data_new(GT_SE3, GT_SE2, gt_data))
    {
        cout << "GT Read Successfully! " << GT_SE3.size() << endl;
    } 
    else exit(1);

    //Calculate_GroundTrue(GPSMeasurements, gt);
    integrateIMUData(imuMeasurements_SE3, imu_edge, IMU_corr, slamPoses);
    runBatch(slamPoses, imu_edge, GT_SE2,IMU_corr);


    plt::show();
}
    
