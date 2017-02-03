#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <Eigen/Core>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <baxter_core_msgs/SolvePositionIK.h>
#include <geometry_msgs/PoseStamped.h>

//global variables to all methods and the main
std::vector<double> joints_values(7);

//joints feedback
void jocommCallback(sensor_msgs::JointState jo_state)
{
    joints_values[0] = jo_state.position[5]; joints_values[1] = jo_state.position[6]; joints_values[2] = jo_state.position[3];
    joints_values[3] = jo_state.position[4]; joints_values[4] = jo_state.position[7]; joints_values[5] = jo_state.position[8];
    joints_values[6] = jo_state.position[9];
}

int main(int argc, char **argv)
{
    //initialization
    ros::init(argc, argv, "test_joint_action_left");
    ros::NodeHandle node;
    ros::Subscriber sub_jointmsg;
    ros::ServiceClient ik_solver;
    sub_jointmsg = node.subscribe<sensor_msgs::JointState>("/robot/joint_states",1,jocommCallback);
    ik_solver = node.serviceClient<baxter_core_msgs::SolvePositionIK>("/ExternalTools/left/PositionKinematicsNode/IKService");
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/robot/limb/left/follow_joint_trajectory", true);
    ros::AsyncSpinner my_spinner(1);
    my_spinner.start();
    usleep(1e6);
    //create the desired position as geometric message of stamped pose
    Eigen::VectorXd target_pose(6);
    target_pose << atof(argv[1]),atof(argv[2]),atof(argv[3]),atof(argv[4]),atof(argv[5]),atof(argv[6]);
    double angle, ang_x, ang_y,  ang_z;
    //if orientation is not defined make the end effector point towards the point
    if(target_pose(3) == 0.0 && target_pose(4) == 0.0 && target_pose(5) == 0.0) {
        //angle = atan2(target_pose[1] - 0.26, target_pose[0] - 0.064);
        //ang_x = 0; ang_y = 2.; ang_z = angle;
        ang_x = 0; ang_y = M_PI; ang_z = 0;
    }
    tf::Quaternion my_angles;
    my_angles.setRPY(ang_x, ang_y, ang_z);
    geometry_msgs::PoseStamped my_desired_pose;
    my_desired_pose.header.frame_id = "/base";
    my_desired_pose.pose.position.x = target_pose(0);    my_desired_pose.pose.position.y = target_pose(1);    my_desired_pose.pose.position.z = target_pose(2);
    my_desired_pose.pose.orientation.w = my_angles.getW();   my_desired_pose.pose.orientation.x = my_angles.getX();    my_desired_pose.pose.orientation.y = my_angles.getY();
    my_desired_pose.pose.orientation.z = my_angles.getZ();
    //solve for desired position
    baxter_core_msgs::SolvePositionIK::Request req;
    baxter_core_msgs::SolvePositionIK::Response res;
    req.pose_stamp.push_back(my_desired_pose);
    ik_solver.call(req, res);
    //create the sequence of joint trajectory points
    trajectory_msgs::JointTrajectory trajectory;
    trajectory.joint_names = res.joints[0].name;
    trajectory_msgs::JointTrajectoryPoint first_pt;
    first_pt.positions = joints_values;
    first_pt.velocities.resize(7, 0.0);    first_pt.accelerations.resize(7, 0.0);    first_pt.effort.resize(7, 0.0);
    first_pt.time_from_start = ros::Duration(0.5);
    trajectory.points.push_back(first_pt);
    trajectory_msgs::JointTrajectoryPoint second_pt;
    second_pt.positions = res.joints[0].position;
    second_pt.velocities.resize(7, 0.0);    second_pt.accelerations.resize(7, 0.0);    second_pt.effort.resize(7, 0.0);
    second_pt.time_from_start = ros::Duration(2);
    trajectory.points.push_back(second_pt);
    //move the arm using joint action server
    trajectory.header.stamp = ros::Time::now();
    if (!ac.waitForServer(ros::Duration(2.0)))
    {
      ROS_ERROR("Could not connect to action server");
    }
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = trajectory;
    goal.goal_time_tolerance = ros::Duration(1);
    ac.sendGoal(goal);
    while(!ac.getState().isDone());
    return 0;
}
