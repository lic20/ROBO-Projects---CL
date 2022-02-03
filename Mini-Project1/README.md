MiniProject 1
==============


Subfolders
-----------
- simple_drive: It includes the pose estimation function and robotdrive for my direct simple solution
- kinematics_solutions: It includes the geometric solutions for pose estimation and robot f(q) propagation function
- Utilties: It includes the needed utility functions for this project
- bruteforce: It includes the pose estimation function and robotdrive for my direct bruteforce solution
- lidar: It includes the scripts for using the lidar


Scripts
--------
1. Non-Linear Least Square:
  - pose_est_NLS
  - autodrive_NLS
2. Kalman Filter:
  - pose_est_filter
  - autodrive_kalman
3. SLAM:
  - pose_est_slam
  - autodrive_slam
4. Mapping:
  - autodrive_mapping
  - landmark_est
5. Room coverage:
  - room_coverage
6. Comparsion:
  - comparsion_plot
  - path.mat
  - comparsion_data.mat


Note:
-----
- All the scripts related to room and robot drawing are directly from the sample code
- I didn't use lidar in this project. It is in my code because I didn't bother to change from the sample code. 
