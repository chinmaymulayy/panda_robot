# Author
Chinmay Mulay 

Date: 21 Jan 2021

# Description
* GUI executing 'pick and place' of a box based on user inputs
 * GUI allowing the user to set/reset panda joints
 * A publisher which publishes the live pose of the box
 
# Files to View

-- robot_gui.py

    - Location - panda_robot/vention_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts 
    - Description - Launches the robot GUI
-- test.py

    - Location - panda_robot/vention_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts
    - Description - Launches the Publisher for the box cordinates
-- demo.launch

    - Location - panda_robot/vention_ws/src/panda_moveit_config/launch/
    - Description - Launches the panda robot files and spawns the robot in Rviz

# Dependencies
- ROS - Kinetic Kame
- Ubuntu 16.04
- Python 2.7
- Tkinter
