# Robust Pose Optimization for SLAM Algorithms using Multi-Sensor Fusion (RoPoSLAMS)

## Objective

Improve estimation accuracy for LiDAR-based SLAM algorithms.

## Getting Started

- Clone GTSAM
- Eigen C++
- Matplotlib CPP
- CMAKE

### Clone repository recursively:
```
  git clone --recurse-submodules git@github.com:hanpar/eecs568-group17-project.git
```

### Building and running the code:
```
  cd pose_optimization
  mkdir build && cd build
  cmake ..
  make
  ./pose_optimization
```

### References 

- GTSAM: https://gtsam.org/
- FLOAM: https://github.com/wh200720041/floam
- ALOAM: https://github.com/HKUST-Aerial-Robotics/A-LOAM
- KITTI Dataset: http://www.cvlibs.net/datasets/kitti/

### Created at University of Michigan - Ann Arbor as part of ROB 530.
The Team: [Boxi Jiang](mailto:boxij@umich.edu), [Nikhil Punshi](mailto:npunshi@umich.edu), [Vishrut Kaushik](mailto:vishrutk@umich.edu), [Yueh-Lin Tsai](mailto:yuehlint@umich.edu), and [Hannah Parrish](mailto:hjpa@umich.edu).