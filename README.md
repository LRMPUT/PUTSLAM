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
- After compiling the grabber coppy all the files and directoried from "3rdParty/OpenNi2/Redist" to the location where your grabber executable is.

- Known Issues in Ubuntu 12.04.

  - cmake does not link opencv libraries if not strictly shown in add_library;
  - opencv issue -- when using cvWatiKey the value is not returned properly add to the condition & 0xff;
  - openni 2.1 should not be used while it is not supporting c++11 (problems with naming of the standard constants)
