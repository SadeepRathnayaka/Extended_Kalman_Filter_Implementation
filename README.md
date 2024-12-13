# Extended_Kalman_Filter_Implementation
 EKF is implemented to estimate the state of a Differential drive robot using wheel encoders. This project has been done in Robot Operating System (ROS2) framework.

 This repository contains the ROS2 packages of the Turtlebot description(URDF), the Differential Drive controller for the Turtlebot robot, and the Extended Kalman Filter Implementation package.

 Visualization of the result : 
Note that 
   - Blue color trajectory   : Ground Truth
   - Red color trajectory    : Only Predicted state using state estimation model
   - Green color trajectory  : Corrected state with only encoders
   - Yellow color trajectory : Corrected with both encoders and GPS sensor (Fused output)
     
[Screencast from 12-13-2024 11:54:18 PM.webm](https://github.com/user-attachments/assets/440d5b2e-1fe4-4c8f-96bf-ee5f72a5f764)
