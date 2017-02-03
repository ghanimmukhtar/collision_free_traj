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
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group/move_group_context.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

std::vector<std::string> baxter_joint_names;
std::vector<double> joints_values;

void jocommCallback(sensor_msgs::JointState jo_state)
{
    baxter_joint_names = jo_state.name;
    joints_values = jo_state.position;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "baxter_collision_free");
    ros::NodeHandle node;
    //ros::Publisher pub_msg;
    ros::Subscriber sub_jointmsg = node.subscribe<sensor_msgs::JointState>("/robot/joint_states",1,jocommCallback);
    //ros::Publisher collision_object_publisher = node.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
    ros::Publisher planning_scene_publish = node.advertise<moveit_msgs::PlanningScene>("planning_scene",1);
    ros::AsyncSpinner my_spinner(1);
    my_spinner.start();
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(robot_model);
    /*robot_state::RobotState baxter_state(robot_model);
    //const robot_model::JointBoundsVector
    std::vector<moveit::core::VariableBounds> joint_limits;
    std::vector<std::string> joints_names = {"left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"};
    for(int i = 0; i < joints_names.size(); ++i)
        joint_limits.push_back(robot_model->getVariableBounds(joints_names[i]));

    //double start = 0., end = 0.;
    //ROS_INFO("distance function test output is: %f", robot_model->distance(&start, &end));
    std::vector<std::string> joint_group = robot_model->getJointModelGroupNames();
    for (int i = 0; i < joint_group.size(); ++i){
            ROS_INFO_STREAM("joint group "  << i << " name is: " << joint_group[i]);
            ROS_INFO("and the joints in this group are: ");
            std::vector<std::string> joints = robot_model->getJointModelGroup(joint_group[i])->getActiveJointModelNames();
            for(int j = 0; j < joints.size(); ++j)
                ROS_INFO_STREAM("joint: " << j << " is: " << joints[j]);
    }
    baxter_state.setVariablePositions(baxter_joint_names, joints_values);
    const moveit::core::JointModelGroup* my_joint_group = baxter_state.getJointModelGroup("left_arm");
    //my_joint_group->
    std::vector<std::string> links_names =  my_joint_group->getLinkModelNames();
    for (int i = 0; i < links_names.size(); ++i){
                ROS_INFO_STREAM("link: "  << i << " name is: " << links_names[i]);
                const moveit::core::LinkModel* my_link =  my_joint_group->getLinkModel(links_names[i]);
                ROS_INFO_STREAM("and link transform is: ");
                ROS_INFO_STREAM(my_link->getJointOriginTransform().matrix());
    }

    move_group::MoveGroupContextPtr my_context;
    my_context->planning_pipeline_->generatePlan()
    for (int i = 0; i < joint_limits.size(); ++i){
        ROS_INFO("max limit of joint %d is: %f", i, joint_limits[i].max_position_);
        ROS_INFO("min limit of joint %d is: %f", i, joint_limits[i].min_position_);
    }
    moveit::core::JointBoundsVector joint_limits = robot_model->getActiveJointModelsBounds();
    ROS_INFO("size of joint limits vector is: %d", joint_limits.size());
    const moveit::core::JointModel::Bounds* my_first_bound = joint_limits[0];
    moveit::core::VariableBounds my_first_limit_vector = my_first_bound->at(0);
    ROS_INFO("max limit of first joint is: %f", my_first_limit_vector.max_position_);
    ROS_INFO("max limit of first joint is: %f", my_first_limit_vector.min_position_);*/
    moveit_msgs::PlanningScene my_scene;
    my_scene.robot_model_name = "baxter";

    collision_detection::CollisionRequest collision_request;
    collision_request.contacts = true;
    collision_detection::CollisionResult collision_result;
    robot_state::RobotState current_state = planning_scene.getCurrentState();

    std::vector <std::string> variable_names(14);
    variable_names[0] = "left_s0"; variable_names[1] = "left_s1"; variable_names[2] = "left_e0";
    variable_names[3] = "left_e1"; variable_names[4] = "left_w0"; variable_names[5] = "left_w1";
    variable_names[6] = "left_w2";
    variable_names[7] = "right_s0"; variable_names[8] = "right_s1"; variable_names[9] = "right_e0";
    variable_names[10] = "right_e1"; variable_names[11] = "right_w0"; variable_names[12] = "right_w1";
    variable_names[13] = "right_w2";
    std::vector<double> q;
    q.push_back(-0.87); q.push_back(-0.06); q.push_back(-0.60);
    q.push_back(1.172); q.push_back(-2.27); q.push_back(-0.86);
    q.push_back(2.354);
    q.push_back(-0.23); q.push_back(-0.64); q.push_back(1.005);
    q.push_back(1.187); q.push_back(-0.77); q.push_back(1.341);
    q.push_back(-0.38);
    current_state.setVariablePositions(variable_names,q);

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


    shape_msgs::Mesh primitive_1;
    //primitive_1.vertices = what_leni_gives;
    ROS_INFO("Add an object into the world");
    my_scene.world.collision_objects.push_back(collision_object);
    my_scene.is_diff = true;
    planning_scene_publish.publish(my_scene);
    planning_scene.setPlanningSceneDiffMsg(my_scene);
    planning_scene.setCurrentState(current_state);
    collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
    robot_state::RobotState copied_state = planning_scene.getCurrentState();
    collision_result.clear();
    planning_scene.checkCollision(collision_request, collision_result, copied_state, acm);
    collision_detection::CollisionWorldConstPtr my_world = planning_scene.getCollisionWorld();
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
    return 0;
}
