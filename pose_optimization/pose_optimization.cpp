#include "pose_optimization.h"

int main(const int argc, const char *argv[]) {
    vector<VECTOR_SE3> slamPoses;
    vector<VECTOR_SE2> vertices;
    string slam_data = "/home/vishrut/ros_workspaces/eecs568-group17-project/pose_optimization/data/refined_tf.txt";
    
    vector<EDGE_SE3> imuMeasurements;
    // vector<EDGE_SE2> imuMeasurements;
       
    string imu_data = "/home/vishrut/ros_workspaces/eecs568-group17-project/pose_optimization/data/imu.txt";

    KittiCalibration kittiCalibration;
    string imu_metadata = "/home/vishrut/ros_workspaces/eecs568-group17-project/pose_optimization/data/KittiEquivBiasedImu_metadata.txt";

    int i = 10; 
    if (read_se_3_data(slamPoses, vertices, slam_data))
    {
        // cout << vertices.at(i).q.x() << ", " << vertices.at(i).q.y() << ", " << vertices.at(i).q.z() << ", " << vertices.at(i).q.w() << endl;
        // cout << "IDX = " << vertices_se2.at(i).idx << ", Time = " << vertices_se2.at(i).time << ", X = " << vertices_se2.at(i).x << ", Y = " << vertices_se2.at(i).y << ", Theta = " << vertices_se2.at(i).theta << endl;
    }
    else exit(1);
    
    if (loadKittiData(imuMeasurements, kittiCalibration, imu_data, imu_metadata))
    {
        cout << "IMU Data Read Successfully!" << endl;
    } 
    else exit(1);

    Vector6 BodyP = (Vector6() << kittiCalibration.body_ptx, kittiCalibration.body_pty,
                    kittiCalibration.body_ptz, kittiCalibration.body_prx,
                    kittiCalibration.body_pry, kittiCalibration.body_prz)
                        .finished();

    auto body_T_imu = Pose3::Expmap(BodyP);
    if (!body_T_imu.equals(Pose3(), 1e-5)) {
        printf(
            "Currently only support IMUinBody is identity, i.e. IMU and body frame "
            "are the same");
        exit(-1);
    }

    // Set initial conditions for the estimated trajectory

    // initial pose is the reference frame (navigation frame)
    auto current_pose_global = Pose3(Rot3(1.0, 0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
    
    // the vehicle is stationary at the beginning at position 0,0,0
    Vector3 current_velocity_global = Vector3::Zero();
    
    // init with zero bias
    auto current_bias = imuBias::ConstantBias();  

    auto sigma_init_x = noiseModel::Diagonal::Precisions((Vector6() << Vector3::Constant(0), Vector3::Constant(1.0)).finished());
    auto sigma_init_v = noiseModel::Diagonal::Sigmas(Vector3::Constant(1.0));
    auto sigma_init_b = noiseModel::Diagonal::Sigmas((Vector6() << Vector3::Constant(0.100), Vector3::Constant(5.00e-05)).finished());

    // Set IMU preintegration parameters
    Matrix33 measured_acc_cov = I_3x3 * pow(kittiCalibration.accelerometer_sigma, 2);
    Matrix33 measured_omega_cov = I_3x3 * pow(kittiCalibration.gyroscope_sigma, 2);
    
    // error committed in integrating position from velocities
    Matrix33 integration_error_cov = I_3x3 * pow(kittiCalibration.integration_sigma, 2);

    double g = 9.8;
    auto w_coriolis = Vector3::Zero();  

    auto imu_params = PreintegratedImuMeasurements::Params::MakeSharedU(g);
    imu_params->accelerometerCovariance = measured_acc_cov;  // acc white noise in continuous
    imu_params->integrationCovariance = integration_error_cov;  // integration uncertainty continuous
    imu_params->gyroscopeCovariance = measured_omega_cov;  // gyro white noise in continuous
    imu_params->omegaCoriolis = w_coriolis;

    std::shared_ptr<PreintegratedImuMeasurements> current_summarized_measurement = nullptr;

    // Set ISAM2 parameters and create ISAM2 solver object
    ISAM2Params isam_params;
    isam_params.factorization = ISAM2Params::CHOLESKY;
    isam_params.relinearizeSkip = 10;

    ISAM2 isam(isam_params);

    // Create the factor graph and values object that will store new factors and
    // values to add to the incremental graph
    NonlinearFactorGraph graph;
    Values initialEstimate;  // values storing the initial estimates of new nodes in
                    // the factor graph


    /// Main loop:
    /// (1) we read the measurements
    /// (2) we create the corresponding factors in the graph
    /// (3) we solve the graph to obtain and optimal estimate of robot trajectory
    // printf(
    //     "-- Starting main loop: inference is performed at each time step, but we "
    //     "plot trajectory every 10 steps\n");

    size_t j = 0;
    size_t included_imu_measurement_count = 0;

    size_t first_slam_pose = 1;

    // Something default code uses to skip vertices for GPS readings. Changing this greatly affects when the program fails.
    // slam_skip = 1 means use every SLAM reading 
    auto slam_skip = 1;

    // SUS
    auto slamNoiseModel = noiseModel::Diagonal::Variances((Vector(6) << 3e-6, 3e-6, 3e-6, 1e-6, 1e-6, 1e-6).finished());

    for (int i = first_slam_pose; i < slamPoses.size() - 1; i++) {
        // At each non=IMU measurement we initialize a new node in the graph
        auto current_pose_key = X(i);
        auto current_vel_key = V(i);
        auto current_bias_key = B(i);
        // auto current_pose_key = i;
        // auto current_vel_key = i;
        // auto current_bias_key = i;
        auto t = slamPoses[i].time;

        if (i == first_slam_pose) {
            // Create initial estimate and prior on initial pose, velocity, and biases
            cout << "Current Pose Key = " << current_pose_key << endl;
            initialEstimate.insert(current_pose_key, current_pose_global);
            initialEstimate.insert(current_vel_key, current_velocity_global);
            initialEstimate.insert(current_bias_key, current_bias);
            graph.emplace_shared<PriorFactor<Pose3>>(current_pose_key, current_pose_global, sigma_init_x);
            graph.emplace_shared<PriorFactor<Vector3>>(current_vel_key, current_velocity_global, sigma_init_v);
            graph.emplace_shared<PriorFactor<imuBias::ConstantBias>>(current_bias_key, current_bias, sigma_init_b);
        } 
        else
        {
            cout << "Current Pose Key = " << current_pose_key << endl;
            double dt = 0;
            auto t_previous = slamPoses[i - 1].time;

            // Summarize IMU data between the previous GPS measurement and now
            current_summarized_measurement = std::make_shared<PreintegratedImuMeasurements>(imu_params,
                                                                                            current_bias);
            
            while (j < imuMeasurements.size() && imuMeasurements[j].time <= t) {
                if (imuMeasurements[j].time >= t_previous) {
                    dt = (imuMeasurements[j+1].time - imuMeasurements[j].time)*pow(10,-9);
                    current_summarized_measurement->integrateMeasurement(
                        imuMeasurements[j].accel, imuMeasurements[j].gyro,
                        dt);
                    included_imu_measurement_count++;
                }
                j++;
            }
            // Create IMU factor
            auto previous_pose_key = X(i - 1);
            auto previous_vel_key = V(i - 1);
            auto previous_bias_key = B(i - 1);
            // auto previous_pose_key = i;
            // auto previous_vel_key = i;
            // auto previous_bias_key = i;

            graph.emplace_shared<ImuFactor>(previous_pose_key, previous_vel_key, current_pose_key,
                current_vel_key, previous_bias_key, *current_summarized_measurement);


            // Bias evolution as given in the IMU metadata
            auto sigma_between_b = noiseModel::Diagonal::Sigmas((Vector6() << Vector3::Constant(sqrt(included_imu_measurement_count) * kittiCalibration.accelerometer_bias_sigma),
                                                                Vector3::Constant(sqrt(included_imu_measurement_count) * kittiCalibration.gyroscope_bias_sigma)).finished());
            
            graph.emplace_shared<BetweenFactor<imuBias::ConstantBias>>(
                previous_bias_key, current_bias_key, imuBias::ConstantBias(),
                sigma_between_b);

            // Create GPS factor
            auto slamPose = Pose3(Rot3(slamPoses.at(i).rotationMatrix), Point3(slamPoses.at(i).x, slamPoses.at(i).y, slamPoses.at(i).z));
            
            if ((i % slam_skip) == 0) {

                graph.emplace_shared<PriorFactor<Pose3>>(
                    current_pose_key, slamPose, slamNoiseModel);
                initialEstimate.insert(current_pose_key, slamPose);

                // cout << "############ POSE INCLUDED AT TIME" << t << "############\n";
                cout << slamPose.translation();
                cout << endl;
            } 
            else {
                initialEstimate.insert(current_pose_key, current_pose_global);
            }
            // initialEstimate.insert(current_pose_key, slamPose);
            // Add initial values for velocity and bias based on the previous
            // estimates
            initialEstimate.insert(current_vel_key, current_velocity_global);
            initialEstimate.insert(current_bias_key, current_bias);

            // Update solver
            // =======================================================================
            if (i > (first_slam_pose + 2 * slam_skip)) {
                printf("############ NEW FACTORS AT TIME %lld ############\n",
                        t);
                // graph.print();

                isam.update(graph, initialEstimate);
                cout << "after isam update i = " << i << endl;
                // Reset the newFactors and newValues list
                graph.resize(0);
                initialEstimate.clear();

                // Extract the result/current estimates
                Values result = isam.calculateEstimate();

                current_pose_global = result.at<Pose3>(current_pose_key);
                current_velocity_global = result.at<Vector3>(current_vel_key);
                current_bias = result.at<imuBias::ConstantBias>(current_bias_key);

                cout << "\n############ POSE AT TIME "<< t << "############\n";
                current_pose_global.print();
                printf("\n\n");
            }
        }
    }

    string output_filename = "optimized_trajectory.txt";
    // Save results to file
    printf("\nWriting results to file...\n");
    FILE* fp_out = fopen(output_filename.c_str(), "w+");
    fprintf(fp_out,
            "#time(s),x(m),y(m),z(m),qx,qy,qz,qw,gt_x(m),gt_y(m),gt_z(m)\n");

    Values result = isam.calculateEstimate();
    // for (size_t i = first_slam_pose; i < slamPoses.size() - 1; i++) {
    //     auto pose_key = X(i);
    //     auto vel_key = V(i);
    //     auto bias_key = B(i);

    //     auto pose = result.at<Pose3>(pose_key);
    //     auto velocity = result.at<Vector3>(vel_key);
    //     auto bias = result.at<imuBias::ConstantBias>(bias_key);

    //     auto pose_quat = pose.rotation().toQuaternion();
    //     auto slamPose = Vector3({slamPoses.at(i).x, slamPoses.at(i).y, slamPoses.at(i).z});

    //     cout << "State at #" << i << endl;
    //     cout << "Pose:" << endl << pose << endl;
    //     cout << "Velocity:" << endl << velocity << endl;
    //     cout << "Bias:" << endl << bias << endl;

    //     fprintf(fp_out, "%lld,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
    //             slamPoses.at(i).time, pose.x(), pose.y(), pose.z(),
    //             pose_quat.x(), pose_quat.y(), pose_quat.z(), pose_quat.w(), slamPose(0),
    //             slamPose(1), slamPose(2));
    // }

    fclose(fp_out);

    return 0;
}

