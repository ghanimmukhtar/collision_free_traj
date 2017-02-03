#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include <boost/timer.hpp>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/bodies.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_request.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/MoveGroupActionGoal.h>
#include <tf/tf.h>

std::vector<std::string> baxter_joint_names;
std::vector<double> joints_values;
moveit_msgs::PlanningScene monitored_scene;
//visualization_msgs::InteractiveMarkerUpdate marker_container;


void joints_feedback_cb(sensor_msgs::JointState jo_state)
{
    baxter_joint_names = jo_state.name;
    joints_values = jo_state.position;
}

void monitored_scene_cb(moveit_msgs::PlanningScene my_scene){
    monitored_scene = my_scene;
}

void add_collision_objects(moveit_msgs::PlanningScene& my_scene){
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "/base";
    collision_object.id = "box1";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.4;
    primitive.dimensions[1] = 0.4;
    primitive.dimensions[2] = 0.1;
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x =  0.5;
    box_pose.position.y =  0.5;
    box_pose.position.z =  0.2;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    ROS_INFO("Add an object into the world");
    my_scene.world.collision_objects.push_back(collision_object);
}

void detect_collision(planning_scene::PlanningScene& _scene){
    collision_detection::CollisionRequest collision_request;
    collision_request.contacts = true;
    collision_detection::CollisionResult collision_result;
    collision_detection::AllowedCollisionMatrix acm = _scene.getAllowedCollisionMatrix();
    robot_state::RobotState copied_state = _scene.getCurrentState();
    collision_result.clear();
    _scene.checkCollision(collision_request, collision_result, copied_state, acm);
    collision_detection::CollisionWorldConstPtr my_world = _scene.getCollisionWorld();
    std::vector<std::string> my_world_objects = my_world->getWorld()->getObjectIds();
    std::cout << "object number is: " << my_world_objects.size() << std::endl;
    for(int i = 0; i < my_world_objects.size(); i++)
        std::cout << "name of object number: " << i << " is: " << my_world_objects[i] << std::endl;
    ROS_INFO_STREAM("Test 6: Current state is "
                    << (collision_result.collision ? "in" : "not in")
                    << " self collision");
    std::cout << "contact is: " << collision_result.contacts.size() << std::endl;
    std::cout << "contact result is: " << collision_result.collision << std::endl;
    std::cout << "contact number is: " << collision_result.contact_count << std::endl;
    collision_detection::CollisionResult::ContactMap::const_iterator it;
    for(it = collision_result.contacts.begin();
        it != collision_result.contacts.end();
        ++it)
    {
      ROS_INFO("Contact between: %s and %s",
               it->first.first.c_str(),
               it->first.second.c_str());
    }
}

void fill_goal(moveit_msgs::MoveGroupActionGoal& action_goal,
               moveit_msgs::PlanningScene& my_scene,
               geometry_msgs::PoseStamped& goal_point){
    ros::Time my_stamp = ros::Time::now();
    action_goal.header.stamp = my_stamp;
    action_goal.goal_id.stamp = my_stamp;
    action_goal.goal.request.workspace_parameters.header.stamp = my_stamp;
    action_goal.goal.request.workspace_parameters.header.frame_id = "/world";
    action_goal.goal.request.workspace_parameters.min_corner.x = -1.0;
    action_goal.goal.request.workspace_parameters.min_corner.y = -1.0;
    action_goal.goal.request.workspace_parameters.min_corner.z = -1.0;
    action_goal.goal.request.workspace_parameters.max_corner.x =  1.0;
    action_goal.goal.request.workspace_parameters.max_corner.y =  1.0;
    action_goal.goal.request.workspace_parameters.max_corner.z =  1.0;
    action_goal.goal.request.start_state = my_scene.robot_state;
    action_goal.goal.request.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints("left_gripper", goal_point));
    action_goal.goal.request.planner_id = "RRTConnectkConfigDefault";
    action_goal.goal.request.group_name = "left_arm";
    action_goal.goal.request.num_planning_attempts = 10;
    action_goal.goal.request.allowed_planning_time = 5.0;
    action_goal.goal.request.max_velocity_scaling_factor = 1.0;
    action_goal.goal.request.max_acceleration_scaling_factor = 1.0;
    action_goal.goal.planning_options.planning_scene_diff.is_diff = true;
    action_goal.goal.planning_options.replan_delay = 2.0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "baxter_collision_free");
    ros::NodeHandle node;
    ros::Subscriber sub_jointmsg = node.subscribe<sensor_msgs::JointState>("/robot/joint_states", 1, joints_feedback_cb);
    ros::Subscriber sub_robot_state = node.subscribe<moveit_msgs::PlanningScene>("/move_group/monitored_planning_scene", 1, monitored_scene_cb);
    ros::Publisher planning_scene_publish = node.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::Publisher goal_pub = node.advertise<moveit_msgs::MoveGroupActionGoal>("/move_group/goal", 1);
    ros::AsyncSpinner my_spinner(1);
    my_spinner.start();
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(robot_model);
    moveit_msgs::PlanningScene my_scene;
    my_scene.robot_model_name = "baxter";

    robot_state::RobotState current_state = planning_scene.getCurrentState();
    current_state.setVariablePositions(baxter_joint_names, joints_values);
    /*add_collision_objects(my_scene);

    my_scene.is_diff = true;
    planning_scene_publish.publish(my_scene);
    planning_scene.setPlanningSceneDiffMsg(my_scene);
    planning_scene.setCurrentState(current_state);
    detect_collision(planning_scene);*/
    //ask for a goal and go to it
    //visualization_msgs::InteractiveMarkerUpdate marker_updater;
    //visualization_msgs::InteractiveMarkerPose new_pose;
    tf::Quaternion my_orientation;
    my_orientation.setRPY(atof(argv[4]), atof(argv[5]), atof(argv[6]));
    geometry_msgs::PoseStamped goal_point;
    goal_point.header.frame_id = "/world";
    goal_point.pose.position.x = atof(argv[1]);
    goal_point.pose.position.y = atof(argv[2]);
    goal_point.pose.position.z = atof(argv[3]);
    goal_point.pose.orientation.w = my_orientation.getW();
    goal_point.pose.orientation.x = my_orientation.getX();
    goal_point.pose.orientation.y = my_orientation.getY();
    goal_point.pose.orientation.z = my_orientation.getZ();
    moveit_msgs::MoveGroupActionGoal action_goal;
    fill_goal(action_goal, monitored_scene, goal_point);
    goal_pub.publish(action_goal);

    /*new_pose.header.frame_id = "/world";
    new_pose.pose = goal_point;
    new_pose.name = "EE:goal_left_gripper";
    marker_updater.server_id = marker_container.server_id;
    marker_updater.seq_num = marker_container.seq_num;
    marker_updater.poses.push_back(new_pose);
    while(ros::ok())
        my_marker_pub.publish(marker_updater);*/
    return 0;
}
