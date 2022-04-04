#include "read_tf.h"

void read_vector_se3_data(VECTOR_SE3 &vertex_se3, string line, int &idx){
    int len;

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

    for(string line; getline(g2o_file, line); )
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
    string filename = "data/refined_tf_0018.txt";

    int i = 0;

    if (read_se_3_data(vertices, filename)){
        cout << "IDX = " << vertices.at(i).idx  << ", X = " << vertices.at(i).x << ", Y = " << vertices.at(i).y << ", Z = " << vertices.at(i).z  << endl << "Rotation Matrix: \n" << vertices.at(i).rotationMatrix << endl;
    }

    return 0;
}
