<h1 align="center">Depth-Enhanced Direct<br/> Visual-Inertial Odometry </h1>


This work is based on DM-VIO.

### 1. Installation

	git clone https://github.com/thisparticle/RGBD-DM-VIO.git

The following instructions have been tested with Ubuntu 20.04.

#### 1.1 Required Dependencies 

##### Suitesparse, Eigen3, Boost, yaml-cpp (required).
Required, install with

    sudo apt-get install cmake libsuitesparse-dev libeigen3-dev libboost-all-dev libyaml-cpp-dev

On MacOS we recommend Homebrew to install the dependencies. It might be necessary
to install boost@1.60 instead of the newest boost, in order for the used GTSAM version to work.


##### GTSAM (required).
Build from source with

    sudo apt install libtbb-dev
    git clone https://github.com/borglab/gtsam.git
    cd gtsam
    git checkout 4.2a6          # not strictly necessary but this is the version tested with.
    mkdir build && cd build
    cmake -DGTSAM_POSE3_EXPMAP=ON -DGTSAM_ROT3_EXPMAP=ON -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF ..
    make -j
    sudo make install

(Note: It seems like the keyframe operations are 2-3% slower with the new GTSAM version. To reproduce the realtime paper 
results you should use commit `a738529af9754c7a085903f90ae8559bbaa82e75` of GTSAM).

##### OpenCV.
Used to read, write and display images.
Install with

	sudo apt-get install libopencv-dev


##### Pangolin.
Like for DSO, this is used for the GUI. You should install v0.6.
Install from [https://github.com/stevenlovegrove/Pangolin](https://github.com/stevenlovegrove/Pangolin)


	sudo apt install libgl1-mesa-dev libglew-dev pkg-config libegl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols
	git clone https://github.com/stevenlovegrove/Pangolin.git
	cd Pangolin
	git checkout v0.6
	mkdir build
	cd build
	cmake ..
	cmake --build .
	sudo make install

#### 1.2 Recommended Dependencies

##### Librealsense
This is necessary for the live demo for Realsense cameras. See 
[doc/RealsenseLiveVersion.md](doc/RealsenseLiveVersion.md) for details

##### GTest (optional).
For running tests, install with `git submodule update --init`.

##### ziplib (optional).
Used to read datasets with images as .zip.
See [src/dso/README.md](src/dso/README.md) for instructions.

##### sse2neon (required for ARM builds).
After cloning, run `git submodule update --init` to include this. 

#### 1.3 Build

    catkin_make d2vio -j10

This compiles `dmvio_dataset` to run DM-VIO on datasets (needs both OpenCV and Pangolin installed).
It also compiles the library `libdmvio.a`, which other projects can link to.

#### Trouble-Shooting
The project is based on DM-VIO. In case of problems with compilation we recommend trying to compile https://github.com/lukasvst/dm-vio
first and seeing if it works. 

### 2 Running
    roslaunch d2vio d455.launch                                                



![[video-to-gif output image]](README.assets/ezgif-4-943c7b1d06.gif)
