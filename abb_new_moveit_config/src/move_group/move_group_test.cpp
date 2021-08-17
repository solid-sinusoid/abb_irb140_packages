#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

//#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_test");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  static const std::string PLANNING_GROUP = "my_abb_groups";

  // The :move_group_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadMarkerPub(true);
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Let's start", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 0.707;
  target_pose1.orientation.x = 0.0;
  target_pose1.orientation.y = 0.707;
  target_pose1.orientation.z = 0.0;
  target_pose1.position.x = 0.273;
  target_pose1.position.y = -0.471;
  target_pose1.position.z = 0.261;
  move_group.setPoseTarget(target_pose1);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  //visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("link_6"), joint_model_group, rvt::LIME_GREEN);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  move_group.move();

  // Planning with Path Constraints
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "link_6";
  ocm.header.frame_id = "base_link";
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;

  // Now, set it as the path constraint for the group.
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(test_constraints);


  move_group.setStartStateToCurrentState();
  geometry_msgs::Pose start_pose2;
  start_pose2.orientation.w = 0.010;
  start_pose2.orientation.x = 0.0;
  start_pose2.orientation.y = 1.000;
  start_pose2.orientation.z = 0.0;
  start_pose2.position.x = 0.45;
  start_pose2.position.y = -0.05;
  start_pose2.position.z = 0.5;
  move_group.setPoseTarget(target_pose1);

  // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
  // Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
  move_group.setPlanningTime(5.0);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(start_pose2, "start");
  visual_tools.publishAxisLabeled(target_pose1, "goal");
  visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
  //visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("link_6"), joint_model_group, rvt::LIME_GREEN);
  visual_tools.trigger();
  //move_group.move();
  visual_tools.prompt("next step");
  move_group.move();

  // When done with the path constraint be sure to clear it.
  move_group.clearPathConstraints();

  // Cartesian Paths
  // ^^^^^^^^^^^^^^^
  start_pose2.orientation.w = 0.010;
  start_pose2.orientation.x = 0.0;
  start_pose2.orientation.y = 1.000;
  start_pose2.orientation.z = 0.0;
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(start_pose2);

  geometry_msgs::Pose target_pose3 = start_pose2;
  
  
  target_pose3.position.x -= 0.2;
  waypoints.push_back(target_pose3);  // down

  target_pose3.position.y -= 0.2;
  waypoints.push_back(target_pose3);  // right
  target_pose3.position.x += 0.2;
  waypoints.push_back(target_pose3);
  target_pose3.position.z += 0.2;
  target_pose3.position.y += 0.2;
  target_pose3.position.x -= 0.2;
  waypoints.push_back(target_pose3);  // up and left*/

  move_group.setMaxVelocityScalingFactor(0.3);

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 5.0;
  const double eef_step = 0.02;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Catresian path", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  my_plan.trajectory_ = trajectory;
  move_group.execute(my_plan);

  // Adding/Removing Objects and Attaching/Detaching Objects
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "box1";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.1;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.7;


  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.4;
  box_pose.position.y = -0.2;
  box_pose.position.z = 0.3;


  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  moveit_msgs::CollisionObject collision_object1;
  collision_object1.header.frame_id = move_group.getPlanningFrame();
  collision_object1.id = "box2";
  shape_msgs::SolidPrimitive primitive1;
  primitive1.type = primitive1.BOX;
  primitive1.dimensions.resize(3);
  primitive1.dimensions[0] = 0.1;
  primitive1.dimensions[1] = 0.1;
  primitive1.dimensions[2] = 0.7;
  geometry_msgs::Pose box_pose1;
  box_pose1.orientation.w = 1.0;
  box_pose1.position.x = 0.0;
  box_pose1.position.y = 0.4;
  box_pose1.position.z = 0.3;
  collision_object1.primitives.push_back(primitive1);
  collision_object1.primitive_poses.push_back(box_pose1);
  collision_object1.operation = collision_object1.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  collision_objects.push_back(collision_object1);

  // Now, let's add the collision object into the world
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Add 2 objects", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Wait for MoveGroup to recieve and process the collision object message
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

  // Now when we plan a trajectory it will avoid the obstacle
  move_group.setStartStateToCurrentState();
  //move_group.setStartState(*move_group.getCurrentState());
  geometry_msgs::Pose another_pose;
  another_pose.orientation.x = 0.000;
  another_pose.orientation.y = 0.707;
  another_pose.orientation.z = -0.000;
  another_pose.orientation.w = 0.707;
  another_pose.position.x = 0.434;
  another_pose.position.y = -0.458;
  another_pose.position.z = 0.176;
  move_group.setPoseTarget(another_pose);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (pose goal move around cuboid) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Obhod prepyatstviya", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishAxisLabeled(another_pose, "p3");
  //visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("link_6"), joint_model_group, rvt::LIME_GREEN);
  visual_tools.trigger();
  visual_tools.prompt("Click next for execute");
  move_group.move();
  visual_tools.prompt("next step");

  move_group.setStartStateToCurrentState();
  geometry_msgs::Pose another_pose1;
  another_pose1.orientation.x = -0.000;
  another_pose1.orientation.y = 0.707;
  another_pose1.orientation.z = -0.000;
  another_pose1.orientation.w = 0.707;
  another_pose1.position.x = 0.627;
  another_pose1.position.y = 0.0;
  another_pose1.position.z = 0.421;
  move_group.setPoseTarget(another_pose1);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 6 (pose goal move around cuboid) %s", success ? "" : "FAILED");
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Obratno", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishAxisLabeled(another_pose1, "p4");
  //visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("link_6"), joint_model_group, rvt::LIME_GREEN);
  visual_tools.trigger();
  visual_tools.prompt("Click next for execute");
  move_group.move();

  move_group.setStartStateToCurrentState();
  geometry_msgs::Pose another_pose2;
  another_pose2.orientation.w = -0.022;
  another_pose2.orientation.x = -0.000;
  another_pose2.orientation.y = 1.000;
  another_pose2.orientation.z = 0.000;
  another_pose2.position.x = -0.426;
  another_pose2.position.y = 0.464;
  another_pose2.position.z = 0.368;
  move_group.setPoseTarget(another_pose2);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 7 (pose goal move around cuboid) %s", success ? "" : "FAILED");
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Obratno", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishAxisLabeled(another_pose2, "p5");
  //visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("link_6"), joint_model_group, rvt::LIME_GREEN);
  visual_tools.trigger();
  visual_tools.prompt("Click next for execute");
  move_group.move();
  visual_tools.prompt("next step");



  // Now, let's remove the collision object from the world.
  ROS_INFO_NAMED("tutorial", "Remove the object from the world");
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  object_ids.push_back(collision_object1.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  // Show text in RViz of status
  //visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Object removed", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  /* Wait for MoveGroup to recieve and process the attached collision object message */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disapears");
  //visual_tools.deleteAllMarkers();

  // END_TUTORIAL

  ros::shutdown();
  return 0;
}
