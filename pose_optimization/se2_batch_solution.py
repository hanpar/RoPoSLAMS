from read_data import *
import matplotlib.pyplot as plt
from gtsam.utils import plot


if __name__ == "__main__":
    g2oFile = "/home/vishrut/ROB Degree/ROB 530 Mobile Robotics/Assignment 7/data/input_INTEL_g2o.g2o"

    vertices, edges = read_gto_file(g2oFile)

    priorMean = gtsam.Pose2(0.0, 0.0, 0.0)
    priorNoise = gtsam.noiseModel.Diagonal.Variances(gtsam.Point3(1e-6, 1e-6, 1e-8))

    graph = gtsam.NonlinearFactorGraph()
    graph.add(gtsam.PriorFactorPose2(1, priorMean, priorNoise))

    for edge in edges:
        odometry = gtsam.Pose2(edge[1])
        odometryNoise = gtsam.noiseModel.Gaussian.Information(edge[2])
        graph.add(gtsam.BetweenFactorPose2(edge[0][0], edge[0][1], odometry, odometryNoise))

    initial = gtsam.Values()

    for vertex in vertices:
        initial.insert(vertex[0], gtsam.Pose2(vertex[1][0], vertex[1][1], vertex[1][2]))

    parameters = gtsam.GaussNewtonParams()

    parameters.setRelativeErrorTol(1e-5)
    parameters.setMaxIterations(100)

    optimizer = gtsam.GaussNewtonOptimizer(graph, initial, parameters)
    result = optimizer.optimize()

    initialPoses = gtsam.utilities.extractPose2(initial)
    resultPoses = gtsam.utilities.extractPose2(result)

    plt.plot(initialPoses[:,0], initialPoses[:,1],'tab:blue', label='Initial Poses')
    plt.plot(resultPoses[:,0], resultPoses[:,1], 'tab:orange', label="GTSAM Poses")
    plt.title("SE2 Batch Solution")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.legend()    
    plt.axis('equal')
    plt.show()
















