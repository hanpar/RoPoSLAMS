#include<iostream>
#include<Eigen/Geometry>
#include<math.h>

using namespace std;

int main(){

    Eigen::Quaterniond q;
    q.x() = 0;
    q.y() = 0.2;
    q.z() = 0.55;
    q.w() = 1;
    q = q.normalized();
    auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);


    auto y = (double) atan2(2 * (q.w() * q.y() + q.x() * q.z()), 1 - 2 * (pow(q.y(), 2) + pow(q.x(), 2)));
    auto x = (double) asin(2 * (q.w() * q.x() - q.z() * q.y()));
    auto z = (double) atan2(2 * (q.w() * q.z() + q.y() * q.x()), 1 - 2 * (pow(q.x(), 2) + pow(q.z(), 2)));


    cout << q.x() << " " << q.y() << " "  << q.z() << " " << q.w() << " " << endl;

    cout << x << " " << y << " " << z << "  \n " << endl;
    // cout << euler[0] << " " << euler[1] << " " << euler[2] << "  \n " << endl;
}