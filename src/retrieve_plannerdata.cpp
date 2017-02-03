#include <iostream>
#include <string>
#include <fstream>
#include <ros/ros.h>
#include <ompl/base/PlannerDataGraph.h>
#include <boost/graph/graphml.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/ompl_interface/ompl_interface.h>
#include <ompl/base/PlannerDataStorage.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "retrieve_planner_data");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    ompl_interface::OMPLInterface oi(robot_model, nh);
    planning_interface::PlannerConfigurationMap my_map = oi.getPlannerConfigurations();
    oi.setPlannerConfigurations(my_map);
    std::string arm_choice = "left_arm";
    ompl::base::PlannerData my_data(oi.getPlanningContext(arm_choice, "JointModel")->getOMPLSimpleSetup()->getSpaceInformation());
    std::string fn3 = "test6.xml";
    std::ofstream is3(fn3.c_str());
    my_data.printGraphML(is3);
    ompl::base::PlannerDataStorage my_storage;


    std::string fn = "test4.xml";
    std::ifstream is(fn.c_str());
    std::string fn2 = "test5.xml";
    std::ofstream is2(fn2.c_str());
    if (!is.is_open())
    {
        std::cout << "loading file '" << fn << "'failed." << std::endl;
        throw "Could not load file.";
    }
    my_storage.load(is, my_data);
    my_data.printGraphML(is2);
    ROS_INFO_STREAM("finished!!!!!");
    return 0;
}
