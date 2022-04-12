import numpy as np
import gtsam

def squareform_diagfill(input_array):
    #Credits: Stackoverflow (https://stackoverflow.com/questions/38747311/how-to-create-a-symmetric-matrix-from-a-numpy-1d-array-the-most-efficient-way)

    n = int(np.sqrt(input_array.size * 2))
    if (n*(n+1))//2 != input_array.size:
        print("Size of 1D array not suitable for creating a symmetric 2D array!")
        return None
    else:
        R,C = np.triu_indices(n)
        out = np.zeros((n,n),dtype=input_array.dtype)
        out[R,C] = input_array
        out[C,R] = input_array
    return out

def read_gto_file(path):
    edges = []
    vertices = []

    f = open(path, "r")
    lines = f.readlines()

    for line in lines:
        s = line.split()
        if s[0] == "VERTEX_SE2" :
            vertices.append([int(s[1]), (float(s[2]), float(s[3]), float(s[4]))])

        elif s[0] == "EDGE_SE2" :
            covariance = squareform_diagfill(np.array([float(i) for i in s[6:]]))
            edges.append([(int(s[1]), int(s[2])), (float(s[3]), float(s[4]), float(s[5])), covariance])

        elif s[0] == "VERTEX_SE3:QUAT":
            quaternion = [float(s[8])] + [float(i) for i in s[5:8]]
            transformation_matrix_se3 = gtsam.Rot3(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
            vertices.append([int(s[1]), (float(s[2]), float(s[3]), float(s[4])), transformation_matrix_se3])
        
        elif s[0] == "EDGE_SE3:QUAT":
            quaternion = [float(s[9])]+ [float(i) for i in s[6:9]]
            transformation_matrix_se3 = gtsam.Rot3(quaternion[0], quaternion[1], quaternion[2], quaternion[3])

            covariance = squareform_diagfill(np.array([float(i) for i in s[10:]]))
            edges.append([(int(s[1]), int(s[2])), (float(s[3]), float(s[4]), float(s[5])), transformation_matrix_se3, covariance])

    return vertices, edges

if __name__ == "__main__":
    # g2oFile = "/home/vishrut/ROB Degree/ROB 530 Mobile Robotics/Assignment 7/data/input_INTEL_g2o.g2o"
    g2oFile = "/home/vishrut/ROB Degree/ROB 530 Mobile Robotics/Assignment 7/data/parking-garage.g2o"

    vectors, edges = read_gto_file(g2oFile)

    # print(len(vectors))
    # print(vectors[0])
    # print(len(edges))
    print(edges[0][3])

