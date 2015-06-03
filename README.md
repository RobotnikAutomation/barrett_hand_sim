# barrett_hand_sim
Barrett hand simulation package for ROS

Launch the Gazebo simulation:

roslaunch barrett_hand_gazebo barrett_hand.launch

Publish topics to control the hand:

rostopic pub /bh_j11_position_controller/command std_msgs/Float64 'desired_angle'

Controller list:
bh_j11_position_controller -> spread DoF
bh_j12_position_controller -> finger 1 grasp
bh_j22_position_controller -> finger 2 grasp
bh_j32_position_controller -> finger 3 grasp
