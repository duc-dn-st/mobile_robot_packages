# (Deprecated) Energy Optimal Trajectory Generation using C++ and ROS
## How to use 
Navigate to reference folder and start docker container
```
cd trajectory_generation/reference
docker compose up -d 
```
Get inside docker container 
```
docker exec -it trajectory_generation bash 
```
Run generate_trajectory node 
```
source devel/setup.bash
cd /root/catkin_ws/src
rosrun trajectory_generator generate_trajectory
```