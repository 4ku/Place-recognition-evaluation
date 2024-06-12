# Table of Contents
1. [Introduction](#introduction)
2. [Getting Started](#getting-started)
   - [Dependencies](#dependencies)
   - [Installation](#installation)
   - [LoGG3D-Net Setup](#logg3d-net-setup)
   - [SuperPoint + SuperGlue Setup](#superpoint--superglue-setup)  
   - [MixVPR Setup](#mixvpr-setup)
3. [Rosbag Files](#rosbag-files)
   - [Office Data](#office-data)
   - [Garage Data](#garage-data)
4. [Usage](#usage)
   - [Option 1: Launch Rosbag file from launch file](#option-1-launch-rosbag-file-from-launch-file)
   - [Option 2: Launch Rosbag file from terminal](#option-2-launch-rosbag-file-from-terminal)
   - [Launch Evaluation](#launch-evaluation)
   - [Launch Custom Evaluation](#launch-custom-evaluation)
   - [Evaluate combination of methods](#evaluate-combination-of-methods)
5. [Create own vocabulary for DBoW2](#create-own-vocabulary-for-dbow2)
6. [Results](#results)
   - [Distance-based evaluation](#distance-based-evaluation)
   - [Distance and angle-based evaluation](#distance-and-angle-based-evaluation)
   - [Combined models evaluation (distance-based)](#combined-models-evaluation-distance-based)
   - [Combined models evaluation (distance and angle-based)](#combined-models-evaluation-distance-and-angle-based)
   - [Execution time performance](#execution-time-performance)
7. [Citation](#citation)


# Introduction 
This repository was made to perform comparison and evaluation between different approaches for place recognition. The goal is to provide a comprehensive benchmarking environment for researchers and practitioners to analyze the performance of various place recognition methods. The repository contains the following methods:

1. [SuperPoint + SuperGlue](https://github.com/magicleap/SuperGluePretrainedNetwork)
2. [LoGG3D-Net](https://github.com/csiro-robotics/LoGG3D-Net)
3. [Scan Context](https://github.com/irapkaist/scancontext)
4. [DBoW2](https://github.com/dorian3d/DBoW2)
5. [MixVPR](https://github.com/amaralibey/MixVPR)
6. [STD](https://github.com/hku-mars/STD)

# Getting Started
## Dependencies

*    ROS (tested with ROS Noetic)
*    Boost
*    PCL 1.8
*    OpenCV
*    Eigen3
*    DBoW2
*    Ceres
*    [Livox ROS driver](https://github.com/Livox-SDK/livox_ros_driver)


## Installation

1. Clone the repository into your catkin workspace:

```
cd ~/catkin_ws/src
git clone https://github.com/yourusername/place_recog_eval.git
```
2. Install the required dependencies:

```
sudo apt-get install ros-$(rosversion -d)-pcl-conversions ros-$(rosversion -d)-pcl-ros ros-$(rosversion -d)-message-filters ros-$(rosversion -d)-image-transport ros-$(rosversion -d)-cv-bridge
```

3. Build the package:

```
cd ~/catkin_ws
catkin_make
```

4. Source your workspace:

```
source ~/catkin_ws/devel/setup.bash
```

5. Setup Python environment:

```
pip install -e .
```

### LoGG3D-Net Setup

At first clone LoGG3D-Net repository to **`scripts/methods/` directory!**:
```
cd ~/catkin_ws/src/Place-recognition-evaluation/scripts/methods/
git clone https://github.com/csiro-robotics/LoGG3D-Net.git LoGG3D_Net
```

You can create separate environment to install necessary dependencies for LoGG3D-Net. 

* Install requirements:

```
pip install -r requirements.txt
```

* Install PyTorch with suitable cudatoolkit version. For example:
```
pip install torch==1.13.1+cu116 torchvision==0.14.1+cu116 torchaudio==0.13.1 --extra-index-url https://download.pytorch.org/whl/cu116
```

* Install `torchsparse-1.4.0`

```
sudo apt-get install libsparsehash-dev
pip install --upgrade git+https://github.com/mit-han-lab/torchsparse.git@v1.4.0
```

* Download LoGG3D-Net pre-trained models 

```
cd scripts/methods/LoGG3D_Net/
wget -O checkpoints.zip https://cloudstor.aarnet.edu.au/plus/s/G9z6VzR72TRm09S/download
unzip checkpoints.zip
```


### SuperPoint + SuperGlue Setup

1. Install the required dependencies:
```
pip3 install numpy opencv-python torch matplotlib
```

2. Download the pretrained models from the SuperGlue repository:
```
cd scripts/methods/superglue/weights/
wget https://github.com/magicleap/SuperGluePretrainedNetwork/raw/master/models/weights/superglue_indoor.pth
wget https://github.com/magicleap/SuperGluePretrainedNetwork/raw/master/models/weights/superpoint_v1.pth
```

### MixVPR Setup

At first clone MixVPR repository to **`scripts/methods/` directory!**:
```
cd ~/catkin_ws/src/Place-recognition-evaluation/scripts/methods/
git clone https://github.com/amaralibey/MixVPR.git
```

You can create separate environment to install necessary dependencies for MixVPR. 

* Install requirements:

```
pip install -r requirements.txt
```

Download MixVPR pre-trained models [here](https://drive.google.com/file/d/1vuz3PvnR7vxnDDLQrdHJaOA04SQrtk5L/view?usp=share_link) and put it to `scripts/methods/MixVPR/` directory.


# Rosbag Files

These rosbag files contain data collected at Innopolis University for the purpose of evaluating various place recognition methods. The data is stored in two distinct rosbag files, each representing a different environment: an office room and a laboratory. Both environments feature unique geometries and visual appearances, making them suitable for use as training and testing data.

Each rosbag file contains three topics:

* `/Odometry` (type: `nav_msgs/Odometry`): Odometry information estimated by the SLAM algorithm
* `/camera/color/image_raw_sync` (type: `sensor_msgs/Image`): RGB images from the Intel Depth Camera D435i
* `/livox/lidar_pc` (type: `sensor_msgs/PointCloud2`): Point cloud data from the Livox LIDAR MID-70

## Office Data

**File**: [office_double_loop.bag](https://innopolis-my.sharepoint.com/:u:/g/personal/r_khusainov_innopolis_ru/EbxIWeJWL25AslMtR9_B6l0BIbix3rvijRL5LP9DNZz4tA?e=ny7b9c)

The office data contains two loops in and around an office room. See the figure below for a visual representation of the trajectory.

![image](./assets/0_two_loops.png)

## Garage Data

**File**: [eight_double_loop.bag](https://innopolis-my.sharepoint.com/:u:/g/personal/r_khusainov_innopolis_ru/ESR00oc6JsFMrPt2d5en0EkBGW5R1DjdN3vbClpj7kf_3g?e=XecIQv)  

The laboratory data captures a figure-eight-shaped trajectory with two loops within a laboratory environment. See the figure below for a visual representation of the trajectory.

![image](./assets/8_two_loops.png)


# Usage

It's recommended to read rosbag internally to avoid any delays. To do so, set `use_rosbag` to `true` in the `base.launch` file. If you do evaluation on rosbag file, this is the best option to make results reproducible.

## Option 1: Launch Rosbag file from launch file

1. In a `base.launch` file set `use_rosbag` to `true`.

2. In `config.yaml` file set the desired rosbag file pathes.  

`merge_rosbag_input_path` - source rosbag file with point clouds, odometry and images.
`merge_rosbag_output_path` - path to rosbag file with merged point clouds after preprocessing.

3. Launch merge.launch file to merge point clouds from source rosbag file:

```
roslaunch place_recog_eval merge.launch
```

## Option 2: Launch Rosbag file from terminal

1. In a `base.launch` file set `use_rosbag` to `false`.

2. Run rosbag file in separate terminal:

```
rosbag play <rosbag_file>
```

## Launch Evaluation

Launch the evaluation node for the desired method:

* For DBoW:

```
roslaunch place_recog_eval dbow.launch
```

* For Scan Context:

```
roslaunch place_recog_eval context.launch
```

* For LoGG3D-Net:

```
roslaunch place_recog_eval logg3d.launch
``` 

* For SuperPoint + SuperGlue:

```
roslaunch place_recog_eval superglue.launch
```

* For MixVPR:

```
roslaunch place_recog_eval mix_vpr.launch
```

* For STD:
   
```
roslaunch place_recog_eval std.launch
```

The evaluation results will be printed on the terminal and trajectory path will be saved as an image.

You have the option to modify the threshold or other parameters in the respective launch files. By default, the best parameters for each method have been fine-tuned on `office_double_loop.bag`. To further customize the settings, you can edit the `config.yaml` file.


## Launch Custom Evaluation

You can modify `evaluate.cpp` or `evaluate.py` files to run custom evaluation. It can be useful if you want to find optimal parameters for your method. All you need is to modify `getMethods()` function and add your method to the list of methods. For example:

To find an optimal threshold for Scan Context just add:  
```cpp  
for (double threshold = 0.01; threshold < 0.1; threshold += 0.01)
{
    BaseMethod *method = new ScanContext(threshold, 0.2);
    methods.push_back(method);
}
```

Then you can run evaluation by launching`evaluate_cpp.launch` or `evaluate_py.launch` files:

```
roslaunch place_recog_eval evaluate_cpp.launch
```

or (for Python):

```
roslaunch place_recog_eval evaluate_py.launch
```

In the `results` folder precision-recall curve will be saved as an image. 

## Evaluate combination of methods

1. Set `save_candidates` to `true` in `config.yaml` file. This will save predicted and real candidates for each method in the `results` folder.

2. Launch evaluation for each method separately. Depends on `angle_consideration`  parameter, results will be stored in `no_angle` or `with_angle` folder in `results` folder.

3. Run `get_results.py` script in `results` folder to get results for combination of methods for each folder (`no_angle` and `with_angle`)):

```
python get_results.py
```
It's used element-wise multiplication of predicted candidates for each method to get final candidates matrix. Then precision, recall, f1 score and accuracy are calculated for final candidates matrix. 

**Note**: after evaluation for each method separately, ground truth file will be created in `results` folder (this file looks like this: `real_*.txt`). You have to be make sure, that all `real_*.txt` files are the same. If they are different, corresponding error will be printed in the terminal.


# Create own vocabulary for DBoW2

1. Run roscore:  

```
roscore
```

2. Run the vocabulary creator node:

```
rosrun place_recog_eval dbow_create_vocab
```

3. Run rosbag file in separate terminal:

```
rosbag play <rosbag_file>
```

The resulting vocabulary will be saved in the `include/methods/dbow/` directory.

# Results

## Distance-based evaluation

| Method                | Precision | Recall | F1 Score |
|-----------------------|-----------|--------|----------|
| DBoW2                 | 0.946     | 0.137  | 0.240    |
| SuperPoint + SuperGlue| 0.970     | 0.320  | 0.481    |
| Scan Context          | 0.951     | 0.178  | 0.300    |
| LoGG3D-Net            | 0.792     | 0.122  | 0.211    |
| MixVPR                | 0.786     | 0.008  | 0.016    |
| STD                   | 0.750     | 0.002  | 0.004    |

*Table 1: Models evaluation on test data (laboratory environment, eight.bag). True positive here is when two points are close to each other (within 3 meters).*

## Distance and angle-based evaluation

| Method                | Precision | Recall | F1 Score |
|-----------------------|-----------|--------|----------|
| DBoW2                 | 0.941     | 0.324  | 0.482    |
| SuperPoint + SuperGlue| 0.970     | 0.758  | 0.851    |
| Scan Context          | 0.711     | 0.316  | 0.437    |
| LoGG3D-Net            | 0.782     | 0.285  | 0.418    |
| MixVPR                | 0.786     | 0.019  | 0.036    |
| STD                   | 0.750     | 0.005  | 0.010    |

*Table 2: Models evaluation on test data (laboratory environment). True positive here is when two points are close to each other (within 3 meters) and oriented in the same direction (within 45 degrees).*  

## Combined models evaluation (distance-based)

| Method                  | Precision | Recall | F1 Score |
|-------------------------|-----------|--------|----------|
| DBoW2 + LoGG3D          | 0.977     | 0.060  | 0.113    |
| DBoW2 + Scan Context    | 0.990     | 0.068  | 0.127    |
| DBoW2 + STD             | 1.000     | 0.001  | 0.001    |
| (S.P. + S.G.) + Scan C. | 1.000     | 0.124  | 0.220    |
| (S.P. + S.G.) + LoGG3D  | 0.970     | 0.116  | 0.207    |
| (S.P. + S.G.) + STD     | 1.000     | 0.002  | 0.004    |
| MixVPR + Scan C.        | 1.000     | 0.006  | 0.013    |
| MixVPR + LoGG3D         | 0.714     | 0.004  | 0.007    |
| MixVPR + STD            | 0.000     | 0.000  | 0.000    |

*Table 3: Combined models evaluation on test data (laboratory environment). True positive here is when two points are close to each other (within 3 meters). Note: S.P. = SuperPoint, S.G. = SuperGlue.*  

## Combined models evaluation (distance and angle-based)

| Method                  | Precision | Recall | F1 Score |
|-------------------------|-----------|--------|----------|
| DBoW2 + LoGG3D          | 0.977     | 0.142  | 0.248    |
| DBoW2 + Scan Context    | 0.990     | 0.160  | 0.276    |
| DBoW2 + STD             | 1.000     | 0.002  | 0.003    |
| (S.P. + S.G.) + Scan C. | 1.000     | 0.294  | 0.454    |
| (S.P. + S.G.) + LoGG3D  | 0.970     | 0.275  | 0.429    |
| (S.P. + S.G.) + STD     | 1.000     | 0.005  | 0.010    |
| MixVPR + Scan C.        | 1.000     | 0.015  | 0.030    |
| MixVPR + LoGG3D         | 0.714     | 0.008  | 0.017    |
| MixVPR + STD            | 0.000     | 0.000  | 0.000    |

*Table 4: Combined models evaluation on test data (laboratory environment). True positive here is when two points are close to each other (within 3 meters) and oriented in the same direction (within 45 degrees). Note: S.P. = SuperPoint, S.G. = SuperGlue.*

## Execution time performance

| Method                | Processing unit(s) | Total Duration (s) |
|-----------------------|--------------------|--------------------|
| DBoW2                 | CPU                | 2.82               |
| SuperPoint + SuperGlue| CPU + GPU          | 359.36             |
| Scan Context          | CPU                | 9.46               |
| LoGG3D-Net            | CPU + GPU          | 9.76               |
| MixVPR                | CPU + GPU          | 1.79               |
| STD                   | CPU                | 30.30              |

*Table 5: Execution time performance of each method on 100 frames (i.e., 100 seconds of data).*

# Citation

If you use this code for your research, please cite our paper:

```bibtex
@inproceedings{efremov2023comparative,
  title={A Comparative Analysis of Visual and Point Cloud-Based Place Recognition Methods in Indoor Environment},
  author={Efremov, Ivan and Khafizov, Ramil and Khusainov, Ramil},
  booktitle={2023 21st International Conference on Advanced Robotics (ICAR)},
  pages={128--133},
  year={2023},
  organization={IEEE}
}
```
