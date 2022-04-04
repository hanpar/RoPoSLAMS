#include "read_tf.h"

void read_vector_se2_data(VECTOR_SE2 &vertex_se2, string line){
    int len;

    len = line.find(" ");
    vertex_se2.type = line.substr(0, len);
    line = line.erase(0, len + 1);

    len = line.find(" ");
    vertex_se2.idx = stoi(line.substr(0, len));
    line = line.erase(0, len + 1);

    len = line.find(" ");
    vertex_se2.x = stod(line.substr(0, len));
    line = line.erase(0, len + 1);

    len = line.find(" ");
    vertex_se2.y = stod(line.substr(0, len));
    line = line.erase(0, len + 1);

    len = line.find(" ");
    vertex_se2.theta = stod(line.substr(0, len));
    line = line.erase(0, len + 1);
}

void read_edge_se2_data(EDGE_SE2 &edge_se2, string line){
    int len;

    len = line.find(" ");
    edge_se2.type = line.substr(0, len);
    line = line.erase(0, len + 1);

    len = line.find(" ");
    edge_se2.i = stoi(line.substr(0, len));
    line = line.erase(0, len + 1);

    len = line.find(" ");
    edge_se2.j = stoi(line.substr(0, len));
    line = line.erase(0, len + 1);

    len = line.find(" ");
    edge_se2.x = stod(line.substr(0, len));
    line = line.erase(0, len + 1);

    len = line.find(" ");
    edge_se2.y = stod(line.substr(0, len));
    line = line.erase(0, len + 1);

    len = line.find(" ");
    edge_se2.theta = stod(line.substr(0, len));
    line = line.erase(0, len + 1); 

    for(int i = 0; i < 3; i++){
        len = line.find(" ");
        edge_se2.info.at(0).at(i) = stod(line.substr(0, len));   
        line = line.erase(0, len + 1); 
    }

    len = line.find(" ");
    edge_se2.info.at(1).at(1) = stod(line.substr(0, len));   
    line = line.erase(0, len + 1); 

    len = line.find(" ");
    edge_se2.info.at(1).at(2) = stod(line.substr(0, len));   
    line = line.erase(0, len + 1); 

    len = line.find(" ");
    edge_se2.info.at(2).at(2) = stod(line.substr(0, len));   
    line = line.erase(0, len + 1);

    edge_se2.info.at(1).at(0) = edge_se2.info.at(0).at(1);
    edge_se2.info.at(2).at(0) = edge_se2.info.at(0).at(2);
    edge_se2.info.at(2).at(1) = edge_se2.info.at(1).at(2);
}

bool read_se_2_data(vector<VECTOR_SE2> &vertex, vector<EDGE_SE2> &edges, string filname){

    ifstream g2o_file(filname);
    
    if (!g2o_file.good()){
        cout << "Invalid File\n";
        return 0;
    }
    int len = 0;

    VECTOR_SE2 vertex_se2;
    EDGE_SE2 edge_se2;

    for(string line; getline(g2o_file, line); )
    {
        len = line.find(" ");

        if (len > 8) 
        {
            read_vector_se2_data(vertex_se2, line);
            vertex.push_back(vertex_se2);
        }
        else
        {
            read_edge_se2_data(edge_se2, line);
            edges.push_back(edge_se2);
        }
    }
    g2o_file.close();
    return 1;

}


int main(const int argc, const char *argv[]) {
    vector<VECTOR_SE2> vertex;
    vector<EDGE_SE2> edges;

    string filename = "/home/vishrut/ROB Degree/ROB 530 Mobile Robotics/Assignment 7/data/input_INTEL_g2o.g2o";
    
    if (read_se_2_data(vertex, edges, filename)){
        cout << vertex.at(0).type << " " << vertex.at(10).idx  << " " << vertex.at(10).x << " " << vertex.at(10).y << " " << vertex.at(10).theta << endl;
        cout << edges.at(10).type << " " << edges.at(10).i << " " << edges.at(10).j  << " " << edges.at(10).x << " " << edges.at(10).y << " " << edges.at(10).theta << " " << edges.at(10).info.at(0).at(0) << endl;


        for(int i = 0; i < 3; i++){
            for(int j = 0; j < 3; j++)
                cout << edges.at(10).info.at(i).at(j) << " ";
            cout << endl;
        }
    }

    gtsam::Value::

  return 0;
}
