search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=abb_irb140.srdf
robot_name_in_srdf=abb_irb140
moveit_config_pkg=abb_irb140_moveit_config
robot_name=abb_irb140
planning_group_name=my_abb_groups
ikfast_plugin_pkg=abb_irb140_my_abb_groups_ikfast_plugin
base_link_name=base_link
eef_link_name=tool0
ikfast_output_path=/home/ubuntu/catkin_ws/abb_irb140_my_abb_groups_ikfast_plugin/src/abb_irb140_my_abb_groups_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
