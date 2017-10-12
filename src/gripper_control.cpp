#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <baxter_core_msgs/EndEffectorCommand.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gripper_control");
    ros::NodeHandle nh;
    ros::Publisher gripper_pub = nh.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/left_gripper/command", true);

    ros::AsyncSpinner my_spinner(1);
    my_spinner.start();
    usleep(1e6);

    int gripper_id;
    double position;
    nh.getParam("gripper_id", gripper_id);
    nh.getParam("position", position);
    ROS_WARN_STREAM("my position is: " << position);
    //for calibrating the gripper u need the gripper id and the command "calibrate"
    baxter_core_msgs::EndEffectorCommand calibrate_command;
    ros::Rate rate(10);
    //while(ros::ok()){
        calibrate_command.id = gripper_id;
        //calibrate_command.command = "calibrate";
        //gripper_pub.publish(calibrate_command);

        //close gripper command "grip"
        //calibrate_command.command = "grip";
        //gripper_pub.publish(calibrate_command);

        //open gripper command "release"
        //std::vector<std::string> position = {std::to_string(100)};
        //const std::string position = std::to_string(100);
        if(position != 0.0)
            calibrate_command.args = "{position: 100.0}";
        else
            calibrate_command.args = "{position: 0.0}";
        calibrate_command.command = "go";

        gripper_pub.publish(calibrate_command);

        //rate.sleep();
    //}
}
