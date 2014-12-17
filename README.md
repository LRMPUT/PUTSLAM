PUTSLAM
=======

RGBD-based Simultaneous Localization and Mapping

Global requirements:
  - CMake > 2.8
  - GCC > 4.6 (4.7)
  - OpenCV > 2.42 (3.0)
  - PCL > 1.7 (1.7)

Xtion Grabber:

- Current version compiled on Ubuntu 14.04 x64

- Used libraries and compilers:

  - cmake version 2.8.12.2
  - gcc (Ubuntu 4.8.2-19ubuntu1) 4.8.2
  - opencv 2.4.9
  - openNI-Linux-x64-2.2
- After compiling the grabber copy all the files and the directory from "3rdParty/OpenNi2/Redist" to the location where your grabber executable is.

- Known Issues in Ubuntu 12.04.

  - cmake does not link opencv libraries if not strictly shown in add_library;
  - opencv issue -- when using cvWatiKey the value is not returned properly add to the condition & 0xff;
  - openni 2.1 should not be used while it is not supporting c++11 (problems with naming of the standard constants)


======= Installation guide on ubuntu 14.04.1

== OpenCV installation (just paste and execute each line)

sudo apt-get install build-essential
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..
make -j4
sudo make install

== PCL instllation (just paste and execute each line)

sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install libpcl-all

== PUTSLAM

Install g2o dependencies:

sudo apt-get install libeigen3-dev libsuitesparse-dev libqt4-dev qt4-qmake libqglviewer-dev

Go to directory you want to put the PUTSLAM and then:

sudo apt-get install git
git clone https://github.com/LRMPUT/PUTSLAM.git
cd PUTSLAM && mkdir build && cd build
cmake ..
make -j4

As a results in path (...)/PUTSLAM/build/bin you should have multiple files starting with "Demo_..." presenting the use of PUTSLAM.



