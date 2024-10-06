# (Deprecated) Energy Optimal Trajectory Generation using C++ and ROS
## How to use 
Navigate to reference folder and start docker container
```
cd trajectory_generation/reference
docker compose up -d 
```
Get inside docker container 
```
docker exec -it trajectory_generator bash 
```
Run generate_trajectory node 
```
source devel/setup.bash
cd /root/catkin_ws/src
rosrun trajectory_generation generate_trajectory
```

# Note for generate trajectory for Hakurou-kun with bigger sampling time

In utilities.h :
- change PATH_VEL_LIM, PATH_VEL_MIN : (Line 102, 103) Increase PATH_VEL_LIM so that it can run faster
- Changed:
    - PATH_VEL_LIM : 0.1 -> 0.3
    - PATH_VEL_MIN : 0.05 -> 0.1

In global_trajectory.cpp :
- change default_way_points : (Line 26, 27)
- Starting point : (Line 48) ```way_points.row(0) << 0.0, 0.0;```

- Changed:
    - choosing L shape waypoint : (0,0) , (5,0) , (5,5)

In global_trajectory.h :
- change SAMPLING_TIME : (Line 63) To 1 second
- change PATH_ACC_LIM : (Line 77) Increase PATH_ACC_LIM so that the velocity can be bigger

- Changed :
    - SAMPLING_TIME : 0.05 -> 1
    - Investigate PATH_ACC_LIM : Erase "0.5" factor
    - GEAR RATIO : 40.0 -> 50.0 -> 100.0 (THIS FIXED THE PROBLEM)

