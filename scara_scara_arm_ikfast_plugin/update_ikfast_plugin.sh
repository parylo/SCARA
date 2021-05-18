search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=scara.srdf
robot_name_in_srdf=scara
moveit_config_pkg=scara_moveit_config
robot_name=scara
planning_group_name=scara_arm
ikfast_plugin_pkg=scara_scara_arm_ikfast_plugin
base_link_name=base_link
eef_link_name=gripper
ikfast_output_path=/home/patryk/scara_ws/src/scara_description_pkg/urdf/scara_scara_arm_ikfast_plugin/src/scara_scara_arm_ikfast_solver.cpp

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
