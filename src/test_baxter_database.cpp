#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>

#include <eigen_conversions/eigen_msg.h>

#include <moveit/robot_model_loader/robot_model_loader.h>

#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/PlanningScene.h>

#include <boost/filesystem.hpp>
#include <iostream>

#include <pcl/surface/convex_hull.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <image_processing/tools.hpp>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <tf/transform_listener.h>

std::vector<double> all_joint_values, left_arm_joint_values(7);
std::vector<std::string> all_joint_names, left_joint_names(7);
sensor_msgs::JointState joint_state_holder;
moveit_msgs::PlanningScene my_scene;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr my_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

void jo_callback(sensor_msgs::JointState jo_state)
{

    if(jo_state.position.size() > 7){
        joint_state_holder = jo_state;
        all_joint_names = jo_state.name;
        all_joint_values = jo_state.position;

        left_arm_joint_values[0] = jo_state.position[4];
        left_arm_joint_values[1] = jo_state.position[5];
        left_arm_joint_values[2] = jo_state.position[2];
        left_arm_joint_values[3] = jo_state.position[3];
        left_arm_joint_values[4] = jo_state.position[6];
        left_arm_joint_values[5] = jo_state.position[7];
        left_arm_joint_values[6] = jo_state.position[8];

    }
}


void cloud_filler_callback(sensor_msgs::PointCloud2 the_cloud){
    ROS_WARN("I am here filling the cloud !!!!!!!!!!!!!");
    pcl::fromROSMsg(the_cloud, *my_cloud);

    //ROS_ERROR_STREAM("I am saving the cloud: " << the_cloud.header.frame_id);
    pcl::PassThrough<pcl::PointXYZRGBA> pass;
    pass.setInputCloud (my_cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 1.1);
    pass.filter (*my_cloud);

}

//Convert object position from camera frame to robot frame
void convert_mesh_vertices_to_robot_base(std::vector<Eigen::Vector3d>& vertex_pose_in_camera_frame, std::vector<Eigen::Vector3d>& vertex_pose_in_robot_frame){
    tf::TransformListener listener;
    tf::StampedTransform stamped_transform;

    try{
        listener.lookupTransform("/camera_depth_optical_frame", "/base",
                                 ros::Time::now(), stamped_transform);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    std::vector<Eigen::Vector3d>::iterator itr;
    for(itr = vertex_pose_in_camera_frame.begin(); itr != vertex_pose_in_camera_frame.end(); ++itr){
        geometry_msgs::PointStamped camera_point;
        geometry_msgs::PointStamped base_point;
        camera_point.header.frame_id = "/camera_depth_optical_frame";

        //we'll just use the most recent transform available for our simple example
        camera_point.header.stamp = ros::Time();

        //just an arbitrary point in space
        camera_point.point.x = (*itr)(0);
        camera_point.point.y = (*itr)(1);
        camera_point.point.z = (*itr)(2);

        try{

            listener.transformPoint("/base", camera_point, base_point);

            ROS_INFO("camera_link: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
                     camera_point.point.x, camera_point.point.y, camera_point.point.z,
                     base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
        }
        catch(tf::TransformException& ex){
            ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
        }
        Eigen::Vector3d point_in_robot_frame;
        point_in_robot_frame << base_point.point.x,
                base_point.point.y,
                base_point.point.z;
        vertex_pose_in_robot_frame.push_back(point_in_robot_frame);
    }
}

void add_collision_objects(moveit_msgs::PlanningScene& my_scene){
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "/base";
    collision_object.id = "box1";
    shape_msgs::Mesh primitive_1;
    shapes::ShapeMsg co_mesh_msg;

    std::vector<Eigen::Vector3d> vertex_list, vertex_list_in_camera_frame;

    image_processing::tools::extract_convex_hull(my_cloud, vertex_list_in_camera_frame);

    convert_mesh_vertices_to_robot_base(vertex_list_in_camera_frame, vertex_list);

    EigenSTL::vector_Vector3d vertices_source;
    std::vector<Eigen::Vector3d>::iterator itr;
    for(itr = vertex_list.begin(); itr != vertex_list.end(); ++itr)
        vertices_source.push_back(*itr);

    shapes::Mesh* m = shapes::createMeshFromVertices(vertices_source);
    shapes::constructMsgFromShape(m, co_mesh_msg);
    primitive_1  = boost::get<shape_msgs::Mesh>(co_mesh_msg);

    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x =  0.7;
    box_pose.position.y =  0.0;
    box_pose.position.z =  -0.15;
    collision_object.meshes.push_back(primitive_1);
    collision_object.mesh_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    ROS_INFO("Add an object into the world");
    my_scene.world.collision_objects.push_back(collision_object);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_mesh_extraction_from_pointcloud");
    ros::NodeHandle nh;
    ros::Subscriber joint_state_sub = nh.subscribe<sensor_msgs::JointState>("/robot/joint_states", 1, jo_callback);
    ros::Subscriber cloud_filler_sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 10, cloud_filler_callback);
    ros::Publisher planning_scene_publish = nh.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);

    ros::AsyncSpinner spinner(4);
    spinner.start();

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

    robot_state::RobotState start_state(robot_model);
    left_joint_names = {"left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"};
    start_state.setVariablePositions(left_joint_names, left_arm_joint_values);


    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
    my_scene.robot_model_name = "baxter";

    add_collision_objects(my_scene);
    my_scene.robot_state.joint_state = joint_state_holder;
    my_scene.is_diff = true;
    planning_scene->setCurrentState(start_state);
    planning_scene_publish.publish(my_scene);
    planning_scene->setPlanningSceneDiffMsg(my_scene);

    ROS_INFO_STREAM("finished!!!!!");
    //ros::waitForShutdown();
    return 0;
}
