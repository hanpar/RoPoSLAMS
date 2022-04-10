#include "data_read.h"


void get_se2_from_se3_data(VECTOR_SE3 &vertex_se3, VECTOR_SE2 &vertex_se2){
    vertex_se2.time = vertex_se3.time;
    vertex_se2.idx = vertex_se3.idx;    
    vertex_se2.x = vertex_se3.x;
    vertex_se2.y = vertex_se3.y;

    // auto y = (double) atan2(2 * (q.w() * q.y() + q.x() * q.z()), 1 - 2 * (pow(q.y(), 2) + pow(q.x(), 2)));
    // auto x = (double) asin(2 * (q.w() * q.x() - q.z() * q.y()));
    vertex_se2.theta = (double) atan2(2 * (vertex_se3.q.w() * vertex_se3.q.z() + vertex_se3.q.y() * vertex_se3.q.x()), 1 - 2 * (pow(vertex_se3.q.x(), 2) + pow(vertex_se3.q.z(), 2)));

    // auto euler_angles = vertex_se3.rotationMatrix.eulerAngles(0, 1, 2);
    // vertex_se2.theta = euler_angles[2];

}
void read_vector_se3_data(VECTOR_SE3 &vertex_se3, VECTOR_SE2 &vertex_se2, string line, int idx){
    int len;

    //Ignore 1st 3 Columns
    len = line.find(" ");
    line = line.erase(0, len + 1);
    len = line.find(" ");
    line = line.erase(0, len + 1);
    len = line.find(" ");
    line = line.erase(0, len + 1);

    len = line.find(" ");
    // cout << line.substr(0, len) << endl;
    vertex_se3.time = stoll(line.substr(0, len));
    line = line.erase(0, len + 1);

    //Ignore the next 2 Columns
    len = line.find(" ");
    line = line.erase(0, len + 1);
    len = line.find(" ");
    line = line.erase(0, len + 1);

    vertex_se3.idx = idx++;

    len = line.find(" ");
    vertex_se3.x = stod(line.substr(0, len));
    line = line.erase(0, len + 1);

    len = line.find(" ");
    vertex_se3.y = stod(line.substr(0, len));
    line = line.erase(0, len + 1);

    len = line.find(" ");
    vertex_se3.z = stod(line.substr(0, len));
    line = line.erase(0, len + 1);

    len = line.find(" ");
    vertex_se3.q.x() = stod(line.substr(0, len));
    line = line.erase(0, len + 1);

    len = line.find(" ");
    vertex_se3.q.y() = stod(line.substr(0, len));
    line = line.erase(0, len + 1);

    len = line.find(" ");
    vertex_se3.q.z() = stod(line.substr(0, len));
    line = line.erase(0, len + 1);

    len = line.find(" ");
    vertex_se3.q.w() = stod(line.substr(0, len));
    line = line.erase(0, len + 1);

    vertex_se3.rotationMatrix = vertex_se3.q.normalized().toRotationMatrix();

    get_se2_from_se3_data(vertex_se3, vertex_se2);

}

bool read_se_3_data(vector<VECTOR_SE3> &vertices, vector<VECTOR_SE2> &vertices_se2, string filname){

    ifstream g2o_file(filname);
    
    if (!g2o_file.good()){
        cout << "Invalid File\n";
        return 0;
    }
    int len = 0;

    VECTOR_SE3 vertex_se3;
    VECTOR_SE2 vertex_se2;
    int idx = 0;
    
    string line;

    //Ignore 1st line
    getline(g2o_file, line);

    for(line; getline(g2o_file, line); )
    {
        read_vector_se3_data(vertex_se3, vertex_se2, line, idx);
        vertices.push_back(vertex_se3);
        vertices_se2.push_back(vertex_se2);

    }

    g2o_file.close();
    return 1;

}

void fixKittiData(EDGE_SE3 &meas, string line, int &idx){
     int len; 
     double c0,c1,c2,c3,c4,c5,c6,c7,c8; 
     meas.idx = idx++; 
     
     // don't need time or seq
     len = line.find(" "); 
     line = line.erase(0, len + 1);
     len = line.find(" "); 
     line = line.erase(0, len + 1);  
     
     len = line.find(" "); 
     meas.time = stoll(line.substr(0, len)); 
     line = line.erase(0, len + 1); 
     
     // don't need frame ID
     len = line.find(" "); 
     line = line.erase(0, len + 1); 
     
     len = line.find(" "); 
     meas.q.x() = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1); 
     
     len = line.find(" "); 
     meas.q.y() = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1); 
     
     len = line.find(" "); 
     meas.q.z() = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1); 
     
     len = line.find(" "); 
     meas.q.w() = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1); 
     
     meas.rotationMatrix = meas.q.normalized().toRotationMatrix(); 
     
     len = line.find(" "); 
     meas.qCov(0,0) = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     len = line.find(" "); 
     meas.qCov(0,1) = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     len = line.find(" "); 
     meas.qCov(0,2) = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     meas.qCov(1,0) = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     len = line.find(" "); 
     meas.qCov(1,1)= stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     len = line.find(" "); 
     meas.qCov(1,2) = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     meas.qCov(2,0) = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     len = line.find(" "); 
     meas.qCov(2,1) = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     len = line.find(" "); 
     meas.qCov(2,2) = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     //Eigen::Matrix3d<double,-1,-1> m;
     // m = {{c0,c1,c2},{c3,c4,c5},{c6,c7,c8}};
     // meas.qCov = m; 
     
     len = line.find(" "); 
     double gyro_x = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     len = line.find(" "); 
     double gyro_y = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     len = line.find(" "); 
     double gyro_z = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     meas.gyro = Vector3d(gyro_x, gyro_y, gyro_z); 
     
     len = line.find(" "); 
     meas.gyroCov(0,0) = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     len = line.find(" "); 
     meas.gyroCov(0,1) = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     len = line.find(" "); 
     meas.gyroCov(0,2) = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     meas.gyroCov(1,0) = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     len = line.find(" "); 
     meas.gyroCov(1,1) = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     len = line.find(" "); 
     meas.gyroCov(1,2) = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     meas.gyroCov(2,0) = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     len = line.find(" "); 
     meas.gyroCov(2,1) = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     len = line.find(" "); 
     meas.gyroCov(2,2) = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     
     len = line.find(" "); 
     double accel_x = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     len = line.find(" "); 
     double accel_y = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     len = line.find(" "); 
     double accel_z = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     meas.accel = Vector3d(accel_x, accel_y, accel_z); 
     
     len = line.find(" "); 
     meas.accelCov(0,0) = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     len = line.find(" "); 
     meas.accelCov(0,1) = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     len = line.find(" "); 
     meas.accelCov(0,2) = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     meas.accelCov(1,0) = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     len = line.find(" "); 
     meas.accelCov(1,1) = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     len = line.find(" "); 
     meas.accelCov(1,2) = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     meas.accelCov(2,0) = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     len = line.find(" "); 
     meas.accelCov(2,1) = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     len = line.find(" "); 
     meas.accelCov(2,2) = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     
  }
  
  bool loadKittiData(vector<EDGE_SE3>& imu_measurements, KittiCalibration &kitti_calibration, string imu_data_file, string imu_metadata_file) {
    
    string line;

    ifstream imu_metadata(imu_metadata_file.c_str());

    if(!imu_metadata.good()){
        cout << "Invalid IMU Meta data file\n";
        return 0; 
    } 

    printf("Reading IMU Metadata\n");

    getline(imu_metadata, line, '\n');  // ignore the first line

  // Load Kitti calibration
    getline(imu_metadata, line, '\n');
    sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
            &kitti_calibration.body_ptx, &kitti_calibration.body_pty,
            &kitti_calibration.body_ptz, &kitti_calibration.body_prx,
            &kitti_calibration.body_pry, &kitti_calibration.body_prz,
            &kitti_calibration.accelerometer_sigma,
            &kitti_calibration.gyroscope_sigma,
            &kitti_calibration.integration_sigma,
            &kitti_calibration.accelerometer_bias_sigma,
            &kitti_calibration.gyroscope_bias_sigma,
            &kitti_calibration.average_delta_t);


    
    ifstream imu_file(imu_data_file);

    if(!imu_file.good()){
        cout << "Invalid IMU data file\n";
        return 0; 
    } 
    // ignore first line
    getline(imu_file,line); 
    int len = 0; 
    EDGE_SE3 measurement; 
    int idx = 0; 

    while(getline(imu_file, line)){
        fixKittiData(measurement, line, idx); 
        //cout << line << endl; 
        imu_measurements.push_back(measurement); 
    }
    imu_file.close(); 
    return 1; 
  }

void fixGPSData(GPS_DATA &gps, string line, int &idx) {
     int len; 
     gps.idx = idx++; 
     
     // don't need time or seq
     len = line.find(" "); 
     line = line.erase(0, len + 1);
     len = line.find(" "); 
     line = line.erase(0, len + 1);  
     
     len = line.find(" "); 
     gps.time = stoll(line.substr(0, len)); 
     line = line.erase(0, len + 1); 
     
     // don't need frame ID, status, service
     len = line.find(" "); 
     line = line.erase(0, len + 1);
     len = line.find(" "); 
     line = line.erase(0, len + 1); 
     len = line.find(" "); 
     line = line.erase(0, len + 1); 
     
     len = line.find(" "); 
     gps.latitude = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1); 
     
     len = line.find(" "); 
     gps.longitude = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1); 
     
     len = line.find(" "); 
     gps.altitude = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1); 
     
     len = line.find(" "); 
     gps.PositionCov(0,0) = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     len = line.find(" "); 
     gps.PositionCov(0,1) = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     len = line.find(" "); 
     gps.PositionCov(0,2) = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     gps.PositionCov(1,0) = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     len = line.find(" "); 
     gps.PositionCov(1,1)= stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     len = line.find(" "); 
     gps.PositionCov(1,2) = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     gps.PositionCov(2,0) = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     len = line.find(" "); 
     gps.PositionCov(2,1) = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     len = line.find(" "); 
     gps.PositionCov(2,2) = stod(line.substr(0, len)); 
     line = line.erase(0, len + 1);
     //Eigen::Matrix3d<double,-1,-1> m;
     // m = {{c0,c1,c2},{c3,c4,c5},{c6,c7,c8}};
     // meas.qCov = m; 
     // don't need COV type
     len = line.find(" "); 
     line = line.erase(0, len + 1);
  }

  bool loadGPSData(vector<GPS_DATA>& gps_measurements, string gps_data_file) {
    
    string line;
    
    ifstream gps_file(gps_data_file);

    if(!gps_file.good()){
        cout << "Invalid GPS data file\n";
        return 0; 
    } 
    // ignore first line
    getline(gps_file,line); 
    int len = 0;
    GPS_DATA measurement; 
    int idx = 0;
    
    while(getline(gps_file, line)){
        fixGPSData(measurement, line, idx);
        //cout << line << endl; 
        gps_measurements.push_back(measurement); 
    }
    gps_file.close(); 
    return 1; 
  }

// int main(const int argc, const char *argv[]) {
//     vector<VECTOR_SE3> vertices;
//     vector<VECTOR_SE2> vertices_se2;
//     string slam_data = "/home/vishrut/ros_workspaces/eecs568-group17-project/pose_optimization/data/refined_tf.txt";
    
//     vector<EDGE_SE3> edges;   
//     string imu_data = "/home/vishrut/ros_workspaces/eecs568-group17-project/pose_optimization/data/imu.txt";

//     KittiCalibration kittiCalibration;
//     string imu_metadata = "/home/vishrut/ros_workspaces/eecs568-group17-project/pose_optimization/data/KittiEquivBiasedImu_metadata.txt";

//     int i = 10; 
//     if (read_se_3_data(vertices, vertices_se2, slam_data))
//     {
//         cout << vertices.at(i).q.x() << ", " << vertices.at(i).q.y() << ", " << vertices.at(i).q.z() << ", " << vertices.at(i).q.w() << endl;
//         cout << "IDX = " << vertices_se2.at(i).idx << ", Time = " << vertices_se2.at(i).time << ", X = " << vertices_se2.at(i).x << ", Y = " << vertices_se2.at(i).y << ", Theta = " << vertices_se2.at(i).theta << endl;
//     }

//     //int j = 0; 
    
//     //  if (loadKittiData(edges, kittiCalibration, imu_data, imu_metadata))
//     //  {
//     //  	/*cout << "IDX = " << measurements.at(j).idx << ", Time = " << measurements.at(j).time << endl;
//     //  	cout << "Quaternion x = " <<  measurements.at(j).q.x() << " y = " <<  measurements.at(j).q.y() << " z = " <<  measurements.at(j).q.z() << " w = " <<  measurements.at(j).q.w() << endl;
//     //  	cout << "Quat Cov = " << measurements.at(j).qCov << endl; 
//     //  	cout << "Gyro = " << measurements.at(j).gyro << endl;
//     //  	cout << "Quat Cov = " << measurements.at(j).gyroCov << endl;
//     //  	cout << "Accel = " << measurements.at(j).accel << endl; 
//     //  	cout << "Quat Cov = " << measurements.at(j).accelCov << endl;
//     //  	cout << "Rotation Matrix = " << measurements.at(j).rotationMatrix << endl; */
     	
//     //  }


//     // // Integration
//     // std::shared_ptr<PreintegratedImuMeasurements> current_summarized_measurement = nullptr;
//     // auto imu_params = PreintegratedImuMeasurements::Params::MakeSharedU(9.8);
//     // imu_params->accelerometerCovariance = I_3x3;  // acc white noise in continuous
//     // imu_params->integrationCovariance = I_3x3;  // integration uncertainty continuous
//     // imu_params->gyroscopeCovariance = I_3x3;  // gyro white noise in continuous
//     // imu_params->omegaCoriolis = Vector3::Zero();

//     // auto current_bias = imuBias::ConstantBias();
//     // size_t included_imu_measurement_count = 0;
//     // int j = 0; 
//     // for(int i = 1; i < (vertices.size()-1); ++i)
//     // {
//     //     // testing integration
//     //     double t_previous = vertices[i - 1].time;

//     // // Summarize IMU data between the previous GPS measurement and now
//     //     current_summarized_measurement = std::make_shared<PreintegratedImuMeasurements>(imu_params,current_bias);
//     //     double dt = 0;
//     //     while (j < edges.size() && edges[j].time <= vertices[i].time) {
//     //         if (edges[j].time >= t_previous) {
//     //             dt = ( edges[j+1].time - edges[j].time)*pow(10,-9);
//     //             current_summarized_measurement->integrateMeasurement(
//     //                 edges[j].accel, edges[j].gyro, dt);
//     //             //included_imu_measurement_count++;
//     //         }
//     //         j++;
//     //     }
//     //     // current_summarized_measurement->print(); 
// 	// }

//     return 0;


// }
