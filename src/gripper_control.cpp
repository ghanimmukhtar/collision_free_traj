#include <ros/ros.h>
#include <baxter_core_msgs/EndEffectorCommand.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gripper_control");
    ros::NodeHandle nh;
    ros::Publisher gripper_pub = nh.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/left_gripper/command", true);

    //for calibrating the gripper u need the gripper id and the command "calibrate"
    baxter_core_msgs::EndEffectorCommand calibrate_command;
    calibrate_command.id = 65538;
    calibrate_command.command = "calibrate";
    gripper_pub.publish(calibrate_command);

    //close gripper command "grip"
    calibrate_command.command = "grip";
    gripper_pub.publish(calibrate_command);

    //open gripper command "release"
    calibrate_command.command = "release";
    gripper_pub.publish(calibrate_command);
}
