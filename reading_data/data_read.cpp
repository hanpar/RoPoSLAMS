#include "data_read.h"

// GTSAM related includes.
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>


using namespace gtsam;

void read_vector_se3_data(VECTOR_SE3 &vertex_se3, string line, int &idx){
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
}

bool read_se_3_data(vector<VECTOR_SE3> &vertices, string filname){

    ifstream g2o_file(filname);
    
    if (!g2o_file.good()){
        cout << "Invalid File\n";
        return 0;
    }
    int len = 0;

    VECTOR_SE3 vertex_se3;
    int idx = 0;
    
    string line;

    //Ignore 1st line
    getline(g2o_file, line);

    for(line; getline(g2o_file, line); )
    {
        read_vector_se3_data(vertex_se3, line, idx);
        vertices.push_back(vertex_se3);
    }

    g2o_file.close();
    return 1;

}

void fixKittiData(ImuMeasurement &meas, string line, int &idx){
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
  
  bool loadKittiData(vector<ImuMeasurement>& imu_measurements, string imu_data_file) {
     ifstream imu_file(imu_data_file);
     
     if(!imu_file.good()){
     	cout << "Invalid File\n";
     	return 0; 
     } 
     // ignore first line
     string line; 
     getline(imu_file,line); 
     int len = 0; 
       ImuMeasurement measurement; 
       int idx = 0; 
       while(getline(imu_file, line)){
    	   fixKittiData(measurement, line, idx); 
    	   //cout << line << endl; 
    	   imu_measurements.push_back(measurement); 
     	}
     imu_file.close(); 
     return 1; 
  }




int main(const int argc, const char *argv[]) {
    vector<VECTOR_SE3> vertices;

    //EDIT: Enter your txt file
    string filename = "data/refined_tf.txt";
    vector<ImuMeasurement> measurements;   
    string filename2 = "data/imu.txt";

    //int k = 0; 
    if (read_se_3_data(vertices, filename))
    {
        /*cout << "IDX = " << vertices.at(i).idx << ", Time = " << vertices.at(i).time << ", X = " << vertices.at(i).x << ", Y = " << vertices.at(i).y << ", Z = " << vertices.at(i).z  << endl << "Rotation Matrix: \n" << vertices.at(i).rotationMatrix << endl;*/
    }

    //int j = 0; 
    
     if (loadKittiData(measurements,filename2))
     {
     	/*cout << "IDX = " << measurements.at(j).idx << ", Time = " << measurements.at(j).time << endl;
     	cout << "Quaternion x = " <<  measurements.at(j).q.x() << " y = " <<  measurements.at(j).q.y() << " z = " <<  measurements.at(j).q.z() << " w = " <<  measurements.at(j).q.w() << endl;
     	cout << "Quat Cov = " << measurements.at(j).qCov << endl; 
     	cout << "Gyro = " << measurements.at(j).gyro << endl;
     	cout << "Quat Cov = " << measurements.at(j).gyroCov << endl;
     	cout << "Accel = " << measurements.at(j).accel << endl; 
     	cout << "Quat Cov = " << measurements.at(j).accelCov << endl;
     	cout << "Rotation Matrix = " << measurements.at(j).rotationMatrix << endl; */
     	
     }


    // Integration
    std::shared_ptr<PreintegratedImuMeasurements> current_summarized_measurement = nullptr;
    auto imu_params = PreintegratedImuMeasurements::Params::MakeSharedU(9.8);
    imu_params->accelerometerCovariance = I_3x3;  // acc white noise in continuous
    imu_params->integrationCovariance = I_3x3;  // integration uncertainty continuous
    imu_params->gyroscopeCovariance = I_3x3;  // gyro white noise in continuous
    imu_params->omegaCoriolis = Vector3::Zero();

    auto current_bias = imuBias::ConstantBias();
    size_t included_imu_measurement_count = 0;
    int j = 0; 
    for(int i = 1; i < (vertices.size()-1); ++i)
    {
        // testing integration
        double t_previous = vertices[i - 1].time;

    // Summarize IMU data between the previous GPS measurement and now
        current_summarized_measurement = std::make_shared<PreintegratedImuMeasurements>(imu_params,current_bias);
        double dt = 0;
        while (j < measurements.size() && measurements[j].time <= vertices[i].time) {
            if (measurements[j].time >= t_previous) {
                dt = ( measurements[j+1].time - measurements[j].time)*pow(10,-9);
                current_summarized_measurement->integrateMeasurement(
                    measurements[j].accel, measurements[j].gyro, dt);
                //included_imu_measurement_count++;
            }
            j++;
        }
        current_summarized_measurement->print(); 
	}

    return 0;


}
