FROM ros:noetic-ros-core


# copy directory specific to scart
COPY ./ /root/catkin_ws/src/

RUN apt upgrade -y && apt update -y
RUN apt install python3-catkin-tools -y

# set environment variable for ros
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "export ROS_MASTER_URI=http://localhost:11311" >> /root/.bashrc

# install ros dependencies
WORKDIR /root/catkin_ws 
RUN catkin config --extend /opt/ros/noetic
# RUN rosdep update && \
#     rosdep install --from-paths src --ignore-src -r -y
