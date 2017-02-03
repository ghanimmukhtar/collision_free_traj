#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <ompl/base/PlannerDataStorage.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/tools/thunder/Thunder.h>
#include <ompl/tools/lightning/Lightning.h>
#include <ompl/util/PPM.h>

#include <ompl/config.h>

#include <boost/filesystem.hpp>
#include <boost/bind.h>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ot = ompl::tools;

int number_of_vertices;

bool isStateValid(const ob::State *state, robot_model::RobotModelPtr& robot_model, ompl_interface::ModelBasedPlanningContextPtr& context_ptr)
{
    planning_scene::PlanningScene my_scene(robot_model);
    collision_detection::CollisionRequest req;
    req.contacts = true;
    collision_detection::CollisionResult res;
    collision_detection::AllowedCollisionMatrix acm = my_scene.getAllowedCollisionMatrix();
    robot_state::RobotState robot_state_holder(robot_model);
    context_ptr->getOMPLStateSpace()->copyToRobotState(robot_state_holder, state);
    my_scene.checkSelfCollision(req, res, robot_state_holder, acm);
    /*if(res.collision)
        ROS_ERROR("I am in self collision");
    else
        ROS_WARN("nooooo Collision hurray");*/
    return res.collision;
}

bool terminate_condition(const std::shared_ptr<og::PRM>& my_prm){
    ob::PlannerData my_data(my_prm->getSpaceInformation());
    my_prm->getPlannerData(my_data);
    //if(my_data.numVertices()%100 == 0)
      //  ROS_INFO_STREAM("Number of vertices is: " << my_data.numVertices() );
    return my_data.numVertices() > number_of_vertices;
    //return false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "thunderlighting_example");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    nh.getParam("number_of_vertices", number_of_vertices);

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    ompl_interface::OMPLInterface oi(robot_model, nh);
    planning_interface::PlannerConfigurationMap my_map = oi.getPlannerConfigurations();
    oi.setPlannerConfigurations(my_map);
    std::string arm_choice = "left_arm";
    ob::StateSpacePtr space = oi.getPlanningContext(arm_choice, "JointModel")->getOMPLStateSpace();

    ot::ThunderPtr thunderplanner(new ot::Thunder(space));
    thunderplanner->setFilePath("test5");
    thunderplanner->setStateValidityChecker(boost::bind(&isStateValid, std::placeholders::_1,
                                                      robot_model,
                                                      oi.getPlanningContextManager().getPlanningContext(arm_choice, "JointModel")));
    space->setup();
    thunderplanner->getSpaceInformation()->setStateValidityCheckingResolution(1.0 / space->getMaximumExtent());

    ob::ProblemDefinitionPtr pdf(new ob::ProblemDefinition(thunderplanner->getSpaceInformation()));

    boost::shared_ptr<og::PRM> my_prm(new og::PRM(thunderplanner->getSpaceInformation()));

    my_prm->setProblemDefinition(pdf);
    my_prm->setup();
    ob::PlannerTerminationCondition ptc(boost::bind(&terminate_condition, my_prm));

    thunderplanner->setPlanner(my_prm);
    thunderplanner->setup();
    my_prm->growRoadmap(ptc);

    ob::PlannerData my_data(thunderplanner->getSpaceInformation());
    ROS_ERROR_STREAM("number of vertices before loading planner data: " << my_data.numVertices());
    thunderplanner->getPlannerData(my_data);
    ROS_ERROR_STREAM("number of vertices in my storage: " << my_data.numVertices());
    thunderplanner->doPostProcessing();
    thunderplanner->getExperienceDB()->getSPARSdb()->setPlannerData(my_data);
    thunderplanner->save();
}
