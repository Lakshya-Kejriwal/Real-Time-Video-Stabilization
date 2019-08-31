# Real-Time-Video-Stabilization

This is a basic implementation of a real time video stabilization algorithm based on my paper: <br>
http://www.sciencedirect.com/science/article/pii/S1877050916314624

### Prerequisites

The code requires the following 3rd Party Libraries

- opencv-2.4.9
- g++-5.4.1


### How to Run

- Clone the repository and compile using `g++` and `opencv`.
- Change the parameters of Kalman Filter in `videostab.cpp`. (optional)
- Give the path of your input file or webcam in `main.cpp`.
- Run the program using these dependencies: -lopencv_core -lopencv_calib3d -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc -lopencv_stitching -lopencv_videoio -lopencv_video -lopencv_xfeatures2d

#### Alternatively
- This repository includes a Makefile, run `make` to create the executable (Works under `Linux`)
