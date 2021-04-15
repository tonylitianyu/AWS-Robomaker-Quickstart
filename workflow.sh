#!/bin/bash
# This bash file provides a automatic workflow for "build", "bundle", "upload" for AWS Robomaker simulation


echo -e "Please enter the S3 bucket name: "
read name

cd ~/environment/HelloWorld/robot_ws/
source /opt/ros/melodic/setup.bash && rosdep install --from-paths src --ignore-src -r -y && colcon build
source /opt/ros/melodic/setup.bash && colcon bundle
aws s3 cp bundle/output.tar s3://$name/robot

cd ~/environment/HelloWorld/simulation_ws/
source /opt/ros/melodic/setup.bash && rosdep install --from-paths src --ignore-src -r -y && colcon build
source /opt/ros/melodic/setup.bash && colcon bundle
aws s3 cp bundle/output.tar s3://$name/sim
