#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>

int main(int argc, char **argv)
{
  ros::init (argc, argv, "naive_try");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);


  robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();


  std::vector <std::string> variable_names(14);
  variable_names[0] = "left_s0"; variable_names[1] = "left_s1"; variable_names[2] = "left_e0";
  variable_names[3] = "left_e1"; variable_names[4] = "left_w0"; variable_names[5] = "left_w1";
  variable_names[6] = "left_w2";
  variable_names[7] = "right_s0"; variable_names[8] = "right_s1"; variable_names[9] = "right_e0";
  variable_names[10] = "right_e1"; variable_names[11] = "right_w0"; variable_names[12] = "right_w1";
  variable_names[13] = "right_w2";
  std::vector<double> q;
  //-0.010195506410508592, 0.49344998983144883, -0.27231046358270206, 1.0470000137134257, -0.30998406876426277, 0.030523535243855093, 0.09576347154497267
  q.push_back(0.2); q.push_back(1.0); q.push_back(-0.01);
  q.push_back(0.5); q.push_back(-0.3); q.push_back(0.03);
  q.push_back(-0.05);
  q.push_back(-0.3); q.push_back(1.0); q.push_back(0.01);
  q.push_back(0.5); q.push_back(-0.3); q.push_back(0.03);
  q.push_back(0.1);
  current_state.setVariablePositions(variable_names,q);

  current_state = planning_scene.getCurrentStateNonConst();

  std::cout << "first joint value is: " << *current_state.getJointPositions("left_s0") << std::endl;
  std::cout << "second joint value is: " << *current_state.getJointPositions("left_s1") << std::endl;
  std::cout << "third joint value is: " << *current_state.getJointPositions("left_e0") << std::endl;
  std::cout << "fourth joint value is: " << *current_state.getJointPositions("left_e1") << std::endl;
  std::cout << "fifth joint value is: " << *current_state.getJointPositions("left_w0") << std::endl;
  std::cout << "sixth joint value is: " << *current_state.getJointPositions("left_w1") << std::endl;
  std::cout << "seventh joint value is: " << *current_state.getJointPositions("left_w2") << std::endl;

  std::cout << "************* right arm ******************" << std::endl;

  std::cout << "first joint value is: " << *current_state.getJointPositions("right_s0") << std::endl;
  std::cout << "second joint value is: " << *current_state.getJointPositions("right_s1") << std::endl;
  std::cout << "third joint value is: " << *current_state.getJointPositions("right_e0") << std::endl;
  std::cout << "fourth joint value is: " << *current_state.getJointPositions("right_e1") << std::endl;
  std::cout << "fifth joint value is: " << *current_state.getJointPositions("right_w0") << std::endl;
  std::cout << "sixth joint value is: " << *current_state.getJointPositions("right_w1") << std::endl;
  std::cout << "seventh joint value is: " << *current_state.getJointPositions("right_w2") << std::endl;
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("Test 1: Current state is "
                  << (collision_result.collision ? "in" : "not in")
                  << " self collision");

  std::cout << "contact is: " << collision_result.contacts.size() << std::endl;
  collision_detection::CollisionResult::ContactMap::const_iterator it;
  for(it = collision_result.contacts.begin();
      it != collision_result.contacts.end();
      ++it)
  {
    ROS_INFO("Contact between: %s and %s",
             it->first.first.c_str(),
             it->first.second.c_str());
  }

  current_state.setToRandomPositions();
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("Test 2: Current state is "
                  << (collision_result.collision ? "in" : "not in")
                  << " self collision");


  collision_request.group_name = "right_arm";
  current_state.setToRandomPositions();
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("Test 3: Current state is "
                  << (collision_result.collision ? "in" : "not in")
                  << " self collision");


  std::vector<double> joint_values;
  const robot_model::JointModelGroup* joint_model_group =
    current_state.getJointModelGroup("right_arm");
  current_state.copyJointGroupPositions(joint_model_group, joint_values);
  joint_values[0] = 1.57; //hard-coded since we know collisions will happen here
  current_state.setJointGroupPositions(joint_model_group, joint_values);
  ROS_INFO_STREAM("Current state is "
                  << (current_state.satisfiesBounds(joint_model_group) ? "valid" : "not valid"));


  collision_request.contacts = true;
  collision_request.max_contacts = 1000;

//

  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("Test 4: Current state is "
                  << (collision_result.collision ? "in" : "not in")
                  << " self collision");
  for(it = collision_result.contacts.begin();
      it != collision_result.contacts.end();
      ++it)
  {
    ROS_INFO("Contact between: %s and %s",
             it->first.first.c_str(),
             it->first.second.c_str());
  }

  collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
  robot_state::RobotState copied_state = planning_scene.getCurrentState();

  collision_detection::CollisionResult::ContactMap::const_iterator it2;
  for(it2 = collision_result.contacts.begin();
      it2 != collision_result.contacts.end();
      ++it2)
  {
    acm.setEntry(it2->first.first, it2->first.second, true);
  }
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result, copied_state, acm);
  ROS_INFO_STREAM("Test 5: Current state is "
                  << (collision_result.collision ? "in" : "not in")
                  << " self collision");


  collision_result.clear();
  planning_scene.checkCollision(collision_request, collision_result, copied_state, acm);
  ROS_INFO_STREAM("Test 6: Current state is "
                  << (collision_result.collision ? "in" : "not in")
                  << " self collision");


  std::string end_effector_name = joint_model_group->getLinkModelNames().back();

  geometry_msgs::PoseStamped desired_pose;
  desired_pose.pose.orientation.w = 1.0;
  desired_pose.pose.position.x = 0.75;
  desired_pose.pose.position.y = -0.185;
  desired_pose.pose.position.z = 1.3;
  desired_pose.header.frame_id = "base";
  moveit_msgs::Constraints goal_constraint =
    kinematic_constraints::constructGoalConstraints(end_effector_name, desired_pose);

// Now, we can check a state against this constraint using the
// isStateConstrained functions in the PlanningScene class.

  copied_state.setToRandomPositions();
  copied_state.update();
  bool constrained = planning_scene.isStateConstrained(copied_state, goal_constraint);
  ROS_INFO_STREAM("Test 7: Random state is "
                  << (constrained ? "constrained" : "not constrained"));

// There's a more efficient way of checking constraints (when you want
// to check the same constraint over and over again, e.g. inside a
// planner). We first construct a KinematicConstraintSet which
// pre-processes the ROS Constraints messages and sets it up for quick
// processing.

  kinematic_constraints::KinematicConstraintSet kinematic_constraint_set(kinematic_model);
  kinematic_constraint_set.add(goal_constraint, planning_scene.getTransforms());
  bool constrained_2 =
    planning_scene.isStateConstrained(copied_state, kinematic_constraint_set);
  ROS_INFO_STREAM("Test 8: Random state is "
                  << (constrained_2 ? "constrained" : "not constrained"));

// There's a direct way to do this using the KinematicConstraintSet
// class.

  kinematic_constraints::ConstraintEvaluationResult constraint_eval_result =
    kinematic_constraint_set.decide(copied_state);
  ROS_INFO_STREAM("Test 9: Random state is "
                  << (constraint_eval_result.satisfied ? "constrained" : "not constrained"));

  ros::shutdown();
  return 0;
}
