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
    vertex_se3.qx = stod(line.substr(0, len));
    line = line.erase(0, len + 1);

    len = line.find(" ");
    vertex_se3.qy = stod(line.substr(0, len));
    line = line.erase(0, len + 1);

    len = line.find(" ");
    vertex_se3.qz = stod(line.substr(0, len));
    line = line.erase(0, len + 1);

    len = line.find(" ");
    vertex_se3.qw = stod(line.substr(0, len));
    line = line.erase(0, len + 1);
}

bool read_se_3_data(vector<VECTOR_SE3> &vertex, string filname){

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
        vertex.push_back(vertex_se3);
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
        cout << vertices.at(i).idx  << " " << vertices.at(i).x << " " << vertices.at(i).y << " " << vertices.at(i).z  << " " << vertices.at(i).qx << " " << vertices.at(i).qy << " " << vertices.at(i).qz << " " << vertices.at(i).qw << endl;
    }

    return 0;
}
