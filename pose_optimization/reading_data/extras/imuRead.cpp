#include <fstream>
#include <iostream>
#include <vector> 
#include <string> 
#include <eigen3/Eigen/Dense>

using namespace std; 
using namespace Eigen; 

 struct ImuMeasurement {
    public: 
    string type; 
    int idx; 
    long long int time; 
    Eigen::Vector3d gyro;
    Eigen::Vector3d accel; 
    Eigen::Quaterniond q; 
    Eigen::Matrix3d rotationMatrix; 
    Eigen::Matrix3d qCov;
    Eigen::Matrix3d gyroCov; 
    Eigen::Matrix3d accelCov;     
 };
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
     meas.time = meas.time*pow(10,-9);
 
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
  
  int main(){
     vector<ImuMeasurement> measurements;   
     string filename = "./data/imu.txt";
     
     int i = 0; 
     while (loadKittiData(measurements,filename)){
     	cout << "IDX = " << measurements.at(i).idx << ", Time = " << measurements.at(i).time << endl;
     	cout << "Quaternion x = " <<  measurements.at(i).q.x() << " y = " <<  measurements.at(i).q.y() << " z = " <<  measurements.at(i).q.z() << " w = " <<  measurements.at(i).q.w() << endl;
     	cout << "Quat Cov = " << measurements.at(i).qCov << endl; 
     	cout << "Gyro = " << measurements.at(i).gyro << endl;
     	cout << "Quat Cov = " << measurements.at(i).gyroCov << endl;
     	cout << "Accel = " << measurements.at(i).accel << endl; 
     	cout << "Quat Cov = " << measurements.at(i).accelCov << endl;
     	cout << "Rotation Matrix = " << measurements.at(i).rotationMatrix << endl; 
     	
     	i++; 
     }
     
     return 0;  
  }
  
