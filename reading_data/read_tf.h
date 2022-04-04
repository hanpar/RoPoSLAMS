#ifndef READ_TF
#define READ_TF
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>

using namespace std;

class VECTOR_SE3{
    public:
        string type;
        int idx;
        double x;
        double y;
        double z;
        double qx;
        double qy;
        double qz;
        double qw;     
};


void read_vector_se3_data(VECTOR_SE3 &vertex_se3, string line);
bool read_se_3_data(vector<VECTOR_SE3> &vertex, string filname);


#endif