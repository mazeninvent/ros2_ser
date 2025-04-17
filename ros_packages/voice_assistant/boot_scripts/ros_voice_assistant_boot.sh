#!/bin/bash
source /home/pib/ros_working_dir/ros_config.sh
source /opt/ros/humble/setup.bash
source /home/pib/ros_working_dir/install/setup.bash
ros2 launch voice_assistant launch.py 2>&1 | tee -a ~/ros_working_dir/src/voice_assistant/assistant.log