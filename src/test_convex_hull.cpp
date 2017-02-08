#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/PlanningScene.h>

#include <moveit/ompl_interface/model_based_planning_context.h>
#include "../include/collision_free_traj/config.h"

#include <boost/filesystem.hpp>
#include <iostream>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

#include <pcl/surface/convex_hull.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_processing/tools.hpp>


int number_of_trials, size_initial_db;
bool check_collision, controlled;
std::vector<double> all_joint_values, left_arm_joint_values(7);
std::vector<std::string> all_joint_names, left_joint_names(7);
sensor_msgs::JointState joint_state_holder;
moveit_msgs::PlanningScene my_scene;
pcl::PointCloud<pcl::PointXYZ>::Ptr my_cloud (new pcl::PointCloud<pcl::PointXYZ>);

void jo_callback(sensor_msgs::JointState jo_state)
{

    if(jo_state.position.size() > 7){
        joint_state_holder = jo_state;
        all_joint_names = jo_state.name;
        all_joint_values = jo_state.position;

        left_arm_joint_values[0] = jo_state.position[5];
        left_arm_joint_values[1] = jo_state.position[6];
        left_arm_joint_values[2] = jo_state.position[3];
        left_arm_joint_values[3] = jo_state.position[4];
        left_arm_joint_values[4] = jo_state.position[7];
        left_arm_joint_values[5] = jo_state.position[8];
        left_arm_joint_values[6] = jo_state.position[9];
    }
}

void cloud_filler_callback(sensor_msgs::PointCloud2 the_cloud){
    //ROS_ERROR_STREAM("I am saving the cloud: " << the_cloud.header.frame_id);
    pcl::fromROSMsg(the_cloud, *my_cloud);
}

bool extract_convex_hull(std::vector<geometry_msgs::Point>& vertex_list){
    pcl::ConvexHull<pcl::PointXYZ> hull_extractor;
    pcl::PointCloud<pcl::PointXYZ> hull_cloud;
    //my_cloud = my_cloud_class.Ptr;
    hull_extractor.setInputCloud(my_cloud);
    hull_extractor.reconstruct(hull_cloud);

    if(hull_cloud.empty()){
        ROS_ERROR("unable to compute the convex hull");
        return false;
    }

    for(auto it = hull_cloud.points.begin(); it != hull_cloud.points.end(); ++it){
        geometry_msgs::Point new_point;
        new_point.x = it->x;
        new_point.y = it->y;
        new_point.z = it->z;
        vertex_list.push_back(new_point);
    }

    return true;
}

void add_collision_objects(moveit_msgs::PlanningScene& my_scene){
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "/base";
    collision_object.id = "box1";
    shape_msgs::Mesh primitive_1;

    std::vector<geometry_msgs::Point> vertices;
    extract_convex_hull(vertices);
    primitive_1.vertices = vertices;
    collision_object.meshes.push_back(primitive_1);
    collision_object.operation = collision_object.ADD;
    ROS_INFO("Add an object into the world");
    my_scene.world.collision_objects.push_back(collision_object);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_baxter_database");
    ros::NodeHandle nh;
    ros::Subscriber joint_state_sub = nh.subscribe<sensor_msgs::JointState>("/robot/joint_states", 1, jo_callback);
    ros::Subscriber cloud_filler_sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 10, cloud_filler_callback);
    ros::Publisher planning_scene_publish = nh.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);

    ros::AsyncSpinner spinner(4);
    spinner.start();

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

    robot_state::RobotState start_state(robot_model);
    std::vector<std::string> joint_names = {"left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"};
    start_state.setVariablePositions(joint_names, left_arm_joint_values);


    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
    my_scene.robot_model_name = "baxter";

    add_collision_objects(my_scene);
    my_scene.robot_state.joint_state = joint_state_holder;
    my_scene.is_diff = true;
    planning_scene->setCurrentState(start_state);
    planning_scene_publish.publish(my_scene);
    planning_scene->setPlanningSceneDiffMsg(my_scene);

    return 0;
}
