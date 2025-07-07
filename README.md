# <p align="center">Real-Time Dense 3D Reconstruction in Dynamic Scenes via NeRF-SLAM Fusion</p>

 <p align="center">Jia Liu, Die Mao, Jiaxu Ning, and Dapeng Chen</p>
  <p align="center">Nanjing University of Information Science & Technology</p>

## <p align="center">ABSTRACT</p>
This paper presents a novel approach for real-time dense 3D reconstruction in dynamic scenes by integrating Neural Radiance Fields (NeRF) with Simultaneous Localization and Mapping (SLAM). Traditional SLAM systems often struggle with poor reconstruction effects, low real-time performance, and robustness issues in dynamic environments. Our method addresses these challenges by first employing YOLO-based semantic segmentation to remove dynamic objects from the SLAM system, thereby improving localization accuracy. We then utilize Instant NGP, an extended version of NeRF, for reconstruction tasks, significantly enhancing training and inference speeds while ensuring real-time performance. To mitigate the effects of dynamic occlusion, we introduce motion consistency and depth loss functions. Experimental results on the TUM, Replica, and ScanNet datasets demonstrate that our method outperforms existing SLAM and NeRF-based mapping techniques in terms of both accuracy and real-time performance, providing high-precision dense maps for robot navigation and other tasks.


### Requirements
## C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.

## install git
sudo apt-get install git

## install cmake
sudo apt-get install cmake

## install gcc
sudo apt install gcc
sudo apt install build-essential

## install OpenGL
sudo apt-get install build-essential libgl1-mesa-dev
sudo apt-get install freeglut3-dev
sudo apt-get install libglew-dev libsdl2-dev libsdl2-image-dev libglm-dev libfreetype6-dev
sudo apt-get install libglew-dev

## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

cd Pangolin
mkdir build
cd build
cmake ..
make
sudo make install

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

cd Eigen
mkdir build
cd build
cmake ..
make
sudo make install
sudo cp -r /usr/local/include/eigen3/Eigen /usr/local/include 

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 3.0. Tested with OpenCV 3.2.0 and 4.4.0**.

## DBoW2 and g2o (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

- C++11
- cuDNN-8.1.0
- CUDA 11.1


run opencv
cd ThirdParty/opencv-4.5.5
cmake . -B build
cmake --build build --parallel $(nproc --all)
If an error occurs, use:
cd ThirdParty/opencv-4.5.5
cmake -D CMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=/usr/local -DWITH_TBB=ON -DWITH_V4L=ON -DWITH_QT=ON -DWITH_OPENGL=ON -DBUILD_TIFF=ON . -B build
cmake --build build --parallel $(nproc --all)

run Sophus、yolov5_tensorrtx in thirdparty, General run steps:
mkdir build
cd build
cmake ..
make -j


Running ORB-SLAM3 with your camera

Directory `Examples` contains several demo programs and calibration files to run ORB-SLAM3 in all sensor configurations with Intel Realsense cameras T265 and D435i. The steps needed to use your own camera are: 

1. Calibrate your camera following `Calibration_Tutorial.pdf` and write your calibration file `your_camera.yaml`

2. Modify one of the provided demos to suit your specific camera model, and build it

3. Connect the camera to your computer using USB3 or the appropriate interface

4. Run ORB-SLAM3. For example, for our D435i camera, we would execute:

```
./Examples/Stereo-Inertial/stereo_inertial_realsense_D435i Vocabulary/ORBvoc.txt ./Examples/Stereo-Inertial/RealSense_D435i.yaml
```

ROS Examples

### Building the nodes for mono, mono-inertial, stereo, stereo-inertial and RGB-D
Tested with ROS Melodic and ubuntu 18.04.

Add the path including *Examples/ROS/ORB_SLAM3* to the ROS_PACKAGE_PATH environment variable. Open .bashrc file:
  ```
  gedit ~/.bashrc
  ```
and add at the end the following line. Replace PATH by the folder where you cloned ORB_SLAM3:

  ```
  export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB_SLAM3/Examples/ROS
  ```
  
  
### Running Monocular Node
For a monocular input from topic `/camera/image_raw` run node ORB_SLAM3/Mono. You will need to provide the vocabulary file and a settings file. See the monocular examples above.

  ```
  rosrun ORB_SLAM3 Mono PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```

### Running Monocular-Inertial Node
For a monocular input from topic `/camera/image_raw` and an inertial input from topic `/imu`, run node ORB_SLAM3/Mono_Inertial. Setting the optional third argument to true will apply CLAHE equalization to images (Mainly for TUM-VI dataset).

  ```
  rosrun ORB_SLAM3 Mono PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE [EQUALIZATION] 
  ```

  
### Running RGB_D Node
For an RGB-D input from topics `/camera/rgb/image_raw` and `/camera/depth_registered/image_raw`, run node ORB_SLAM3/RGBD. You will need to provide the vocabulary file and a settings file. See the RGB-D example above.

  ```
  rosrun ORB_SLAM3 RGBD PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```
