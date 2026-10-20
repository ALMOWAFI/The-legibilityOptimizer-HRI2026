cmake_minimum_required(VERSION 3.0.2)
project(hri_legibility_preview)
find_package(catkin REQUIRED COMPONENTS rospy moveit_msgs geometry_msgs std_msgs message_generation)
catkin_python_setup()
add_message_files(FILES Block.msg BlockLayout.msg TrialRequest.msg BaselinePlan.msg OptimizedCandidate.msg
  TrajectoryProposal.msg MotionFeedback.msg)
add_service_files(FILES OptimizeLegibility.srv PlanPreview.srv ValidateTrajectory.srv RequestMotion.srv)
generate_messages(DEPENDENCIES moveit_msgs geometry_msgs std_msgs)
catkin_package(CATKIN_DEPENDS rospy moveit_msgs geometry_msgs std_msgs message_runtime)
catkin_install_python(PROGRAMS scripts/planner_node.py scripts/optimizer_node.py scripts/layout_publisher.py
  scripts/send_trial.py scripts/check_pipeline.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
