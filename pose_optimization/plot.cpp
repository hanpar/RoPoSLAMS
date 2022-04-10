#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <../reading_data/matplotlib-cpp/matplotlibcpp.h>

using namespace std;

namespace plt = matplotlibcpp;

int main(const int argc, const char *argv[]) {
    string poses_p_data = "./data/optimized_poses_without_imu.txt";
       
    string tf_data = "./data/optimized_poses.txt";

    vector<double> x_wo_IMU, y_wo_IMU, theta_wo_IMU;
    vector<double> x_w_IMU, y_w_IMU, theta_w_IMU;
    vector<long long int> t,t1;
    string line, line1;
    
    ifstream fin(poses_p_data);
    getline(fin, line);  // ignore the first line
    
    long long int time;
    double x,y,theta;
    while ((fin >> time) && (fin >> x) && (fin >> y) && (fin >> theta))
    {   
        cout << time << endl;
        t.push_back(time);
        x_wo_IMU.push_back(x);
        y_wo_IMU.push_back(y);
        theta_wo_IMU.push_back(theta);
    }
    fin.close();
    cout << x_wo_IMU.at(0) << endl; 
    ifstream fin1(tf_data);
    getline(fin1, line1);  // ignore the first line
    
    while (fin1 >> time >> x >> y >> theta)
    {
        t1.push_back(time);
        x_w_IMU.push_back(x);
        y_w_IMU.push_back(y);
        theta_w_IMU.push_back(theta);
        
    }
    cout << x_w_IMU.at(0) << endl;

    plt::plot(x_wo_IMU,y_wo_IMU);
    plt::plot(x_w_IMU,y_w_IMU);
    plt::title("Comparison");
    plt::show();
}
  
