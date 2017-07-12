
# CORB-SLAM

**Authors:** Fu Li(lifu11@nudt.edu.cn),Shaowu Yang(shaowu.yang@nudt.edu.cn)

**Current version:** 1.0.0

## introduction

**CORB-SLAM(collaborative ORB-SLAM)** is a centralized multi-robot visual SLAM system based on  **[ORB_SLAM2[1-2]](https://github.com/raulmur/ORB_SLAM2)**.
The ORB-SLAM2 is a versatile and accurate visual SLAM method that has been popularly applied in single robot applications. However, this method cannot provide support to multi-robot cooperation in environmental mapping.
The CORB-SLAM system consists of multiple ORB_SLAM2 clients for local mapping and a central server for global map fusion. Specifically, we extend each of the ORB_SLAM2 clients with a memory managing module that organizes the local map and communicates with the central server. In the central server, we detect the overlaps of multiple local maps by the DBoW method, and fuse these maps by utilizing the PnP method and global optimization through bundle adjustment.

<!-- <div align=center> <img src="https://github.com/lifunudt/M2SLAM/blob/master/images/framework.png" alt="M2SLAM" height="180" align=center /> </div> -->

## 0. Related Publications

[1] F. Li, S. Yang, X. Yi, X. Yang. CORB-SLAM: a Collaborative Visual SLAM System for Multiple Robots. submitted.

## 1. Prerequisites

### 1.0 requirements
  * ubuntu 14.04
  * ROS indigo
  * ORBSLAM2 1.0.0
  * boost

### 1.1 ROS install

Inatall ROS indigo according to the instructions in [ROS wiki](http://wiki.ros.org/indigo/Installation).

### 1.2 ORBSLAM2 and its dependencies

Our CORB-SLAM system is build on the foundation of [ORB_SLAM2(https://github.com/raulmur/ORB_SLAM2). You should follow the instructions provided by ORB_SLAM2 build its dependencies. We do not list here.

### 1.3 Boost library install
We use boost library to serialize and deserialize the data.
We can install boost library using the following instruction in terminal.
```bash
sudo apt-get instal libboost-dev
```


### 2.1 build CORB-SLAM

The M2SLAM runs as the ROS package. and the M2SLAM *src* directory should be the ROS package directory, *catkin_src/*.
We use catkin tool to organize the M2SLAM packages, orbslam_client and orbslam_servcer.

Terminal in the *catkin_src/* directory.
```
catkin_make
catkin_install
```

### 2.2 run CORB-SLAM

#### 2.2.1 start ros core
```
ros core
```
#### 2.2.2 start orbslam_servcer
```
rosrun orbslam_servcer orbslam_servcer
```
### 2.2.3 run orbslam_client in different datasets

1. run KITTI datasets

Download the dataset (grayscale images) from http://www.cvlibs.net/datasets/kitti/eval_odometry.php

Execute the following command. Change `KITTIX.yaml`to KITTI00-02.yaml, KITTI03.yaml or KITTI04-12.yaml for sequence 0 to 2, 3, and 4 to 12 respectively. Change `PATH_TO_DATASET_FOLDER` to the uncompressed dataset folder. Change `SEQUENCE_NUMBER` to 00, 01, 02,.., 11.

```
rosrun ORB_SLAM2_client_KITTI_stereo ORB_SLAM2_client_KITTI_stereo Vocabulary/ORBvoc.txt Examples/Stereo/KITTIX.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER
```

## Reference
[1] Mur-Artal R, Montiel J M M, Tardos J D. ORB-SLAM: a versatile and accurate monocular SLAM system[J]. IEEE Transactions on Robotics, 2015, 31(5): 1147-1163.

[2] Mur-Artal R, Tardos J D. ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras[J]. arXiv preprint arXiv:1610.06475, 2016.

## License
M2SLAM is released under a [GPLv3 license](https://github.com/lifunudt/M2SLAM/blob/master/License-gpl.txt).

For a closed-source version of M2SLAM for commercial purposes, please contact the authors.
