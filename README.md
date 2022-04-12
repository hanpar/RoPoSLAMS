# EECS 568 Group 17 Project

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
  #For Floam
  ./pose_optimization_floam
  #For Aloam
  ./pose_optimization_aloam
```

### Created at University of Michigan - Ann Arbor as part of ROB 530.
[Boxi Jiang](mailto:boxij@umich.edu), [Nikhil Punshi](mailto:npunshi@umich.edu), [Vishrut Kaushik](mailto:vishrutk@umich.edu), [Yueh-Lin Tsai](mailto:yuehlint@umich.edu), and [Hannah Parrish](mailto:hjpa@umich.edu).

### References 

- GTSAM: https://gtsam.org/
- FLOAM: https://github.com/wh200720041/floam
- ALOAM: https://github.com/HKUST-Aerial-Robotics/A-LOAM
