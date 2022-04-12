#include <gtsam/geometry/Pose3.h>

// We will use simple integer Keys to refer to the robot poses.
#include <gtsam/inference/Key.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include </mnt/c/Users/adam8/Desktop/Umich/Course/ROB530/Project/eecs568-group17-project/reading_data/matplotlib-cpp/matplotlibcpp.h>

#include "Pose3_tf.h"

using namespace std;
using namespace gtsam;
namespace plt = matplotlibcpp;

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
    vertex_se3.qx = vertex_se3.q.x();
    line = line.erase(0, len + 1);

    len = line.find(" ");
    vertex_se3.q.y() = stod(line.substr(0, len));
    vertex_se3.qy = vertex_se3.q.y();
    line = line.erase(0, len + 1);

    len = line.find(" ");
    vertex_se3.q.z() = stod(line.substr(0, len));
    vertex_se3.qz = vertex_se3.q.z();
    line = line.erase(0, len + 1);

    len = line.find(" ");
    vertex_se3.q.w() = stod(line.substr(0, len));
    vertex_se3.qw = vertex_se3.q.w();
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

    for(line; getline(g2o_file, line);)
    {
        read_vector_se3_data(vertex_se3, line, idx);
        vertices.push_back(vertex_se3);
    }

    g2o_file.close();
    return 1;

}

int main(const int argc, const char *argv[]) {
    vector<VECTOR_SE3> vertices;

    //EDIT: Enter your txt file
    string filename = "data/refined_tf.txt";

    int i = 1;

    if (read_se_3_data(vertices, filename)){
        cout << "IDX = " << vertices.at(i).idx << ", Time = " << vertices.at(i).time << endl << "Rotation Matrix: \n" << vertices.at(i).rotationMatrix << endl;
    }
    NonlinearFactorGraph graph;
    Values initialEstimate;
    vector<double> point_x, point_y;//, point_z;
    auto priorModel = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished());
    int vec_size = vertices.size();
    for(int i = 0; i < vec_size; i++) {
        point_x.push_back(vertices.at(i).x);
        point_y.push_back(vertices.at(i).y);
        //point_z.push_back(vertices.at(i).z);
        Rot3 Rot = Rot3(vertices.at(i).qw,vertices.at(i).qx,vertices.at(i).qy,vertices.at(i).qz);
        Point3 Trans = Point3(vertices.at(i).x,vertices.at(i).y,vertices.at(i).z);
        initialEstimate.insert(i, Pose3(Rot,Trans));
        if (i == 0) {
            graph.addPrior(1, Pose3(Rot,Trans), priorModel);
        }
    }
    initialEstimate.print("\nInitial Estimate:\n");
    graph.print("\nFactor Graph:\n");

    plt::plot(point_x,point_y);
    plt::title("Initial F-LOAN Point");
    plt::save("Initial_F-LOAN_Point.png");
    plt::show();

    //Batch Solution
    /*
    GaussNewtonParams params;
    params.setVerbosity("TERMINATION");  //  show info about stopping conditions
    GaussNewtonOptimizer optimizer(*graph, *initial, params);
    Values result = optimizer.optimize();
    result.print("result")
    */

    //Incremental Solution
    /*
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    parameters.cacheLinearizedFactors = false;
    parameters.enableDetailedResults = true;
    parameters.print();
    ISAM2 isam(parameters);
    int graph_size
    for (int i = 0; i < vec_size; i++) {
        NonlinearFactorGraph graph;
        Values initialEstimate;
        int id_p = vertices.at(i).idx;
        if (id_p == 0) {
            Rot3 Rot = Rot3(vertices.at(i).qw,vertices.at(i).qx,vertices.at(i).qy,vertices.at(i).qz);
            Point3 Trans = Point3(vertices.at(i).x,vertices.at(i).y,vertices.at(i).z);
            auto NoiseModel = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished());
            graph.addPrior(0, Pose3(Rot,Trans), NoiseModel);
            initialEstimate.insert(id_p, Pose3(Rot,Trans));
        } else {
            Pose3 PrevPose = currentEstimate.atPose3(id_p-1);
            initialEstimate.insert(id_p, PrevPose));
            for (int j == 0; j < graph_size; j++) {
                int id_e1 = 
                int id_e2 =
                Rot3 dRot = 
                Point3 dTrans = 
                vector info = 
                if (id_e2 == id_p) {
                    cov_model = noiseModel::Gaussian::Information(Info);
                    graph.emplace_shared<BetweenFactor<Pose3> >(id_e1,id_e2,Pose3(dRot,dTrans),cov_model)
                }
            }
        }
        ISAM2Result result = isam.update(graph, initialEstimate);
        Values currentEstimate = isam.calculateEstimate();
    } 
    */

    return 0;
}