#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <moveit/ompl_interface/planning_context_manager.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group/move_group_capability.h>
#include <moveit/move_group/move_group_context.h>

#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/PlanningScene.h>

#include <ompl/base/PlannerDataStorage.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/tools/thunder/SPARSdb.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/tools/thunder/Thunder.h>
#include <ompl/tools/lightning/Lightning.h>
#include <ompl/util/PPM.h>

#include <moveit/ompl_interface/detail/state_validity_checker.h>

#include <ompl/config.h>
#include "../include/collision_free_traj/config.h"

#include <boost/filesystem.hpp>
#include <iostream>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

//trial with ompl stuff directly, with the check collision enabled always get segmentation fault
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ot = ompl::tools;


int number_of_trials, number_of_trials_goal;
bool check_collision;
std::vector<double> all_joint_values, left_arm_joint_values(7);
std::vector<std::string> all_joint_names, left_joint_names(7);
sensor_msgs::JointState joint_state_holder;
moveit_msgs::PlanningScene my_scene;


bool isStateValid(const ob::State *state, robot_model::RobotModelPtr& robot_model, ompl_interface::ModelBasedPlanningContextPtr& context_ptr)
//bool isStateValid(const ob::State *state, ompl_interface::ModelBasedPlanningContextPtr& context_ptr)
{
    if(state == NULL)
        return false;
    //ompl_interface::StateValidityChecker my_state_checker(context_ptr.get());
    planning_scene::PlanningScene planning_scene(robot_model);

    collision_detection::CollisionRequest req;
    req.contacts = true;
    collision_detection::CollisionResult res;
    collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
    robot_state::RobotState robot_state_holder(robot_model);
    context_ptr->getOMPLStateSpace()->copyToRobotState(robot_state_holder, state);
    planning_scene.checkSelfCollision(req, res, robot_state_holder, acm);

    if(res.collision)
        ROS_ERROR("I am in self collision");
    else
        ROS_WARN("nooooo Collision hurray");

    //return my_state_checker.isValid(state, true);
    return !res.collision;
}



template <typename T>
static ompl::base::PlannerPtr allocatePlanner(const ob::SpaceInformationPtr& si, const std::string& new_name,
                                              const ompl_interface::ModelBasedPlanningContextSpecification& spec)
{
    ROS_WARN("trying to allocate planner ....");
  ompl::base::PlannerPtr planner(new T(si));
  if (!new_name.empty())
    planner->setName(new_name);
  planner->params().setParams(spec.config_, true);
  planner->setup();
  return planner;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "generate_baxter_database");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();
    nh.getParam("number_of_trials", number_of_trials);
    nh.getParam("number_of_trials_goal", number_of_trials_goal);
    nh.getParam("check_collision", check_collision);

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

    planning_scene::PlanningSceneConstPtr planning_scene(new planning_scene::PlanningScene(robot_model));
    ompl_interface::OMPLInterface oi(robot_model, nh);
    planning_interface::PlannerConfigurationMap my_map = oi.getPlannerConfigurations();
    oi.setPlannerConfigurations(my_map);

    std::string arm_choice = "left_arm";


    //ob::StateSpacePtr space = oi.getPlanningContext(arm_choice, "JointModel")->getOMPLStateSpace();
    ompl_interface::PlanningContextManager context_manager = oi.getPlanningContextManager();
    //context_manager.registerPlannerAllocator("geometric::SPARSDB", boost::bind(&allocatePlanner<og::SPARSdb>, _1, _2, _3));
    ompl_interface::ModelBasedPlanningContextPtr context = context_manager.getPlanningContext(arm_choice, "JointModel");
    //ompl_interface::ModelBasedPlanningContextPtr context = oi.getPlanningContext(arm_choice, "JointModel");
    context->setPlanningScene(planning_scene);
    //ompl_interface::ModelBasedStateSpacePtr space = oi.getPlanningContext(arm_choice, "JointModel")->getOMPLStateSpace();
    ompl_interface::ModelBasedStateSpacePtr space = context->getOMPLStateSpace();

    ot::ThunderPtr my_thunder(new ot::Thunder(space));
    ob::ValidStateSamplerPtr vss = my_thunder->getSpaceInformation()->allocValidStateSampler();

    if(check_collision){
        my_thunder->setFilePath("test_collision_check");
        /*my_thunder->setStateValidityChecker(boost::bind(&isStateValid, _1,
                                                        robot_model,
                                                        context));*/
        my_thunder->setStateValidityChecker(ob::StateValidityCheckerPtr(new ompl_interface::StateValidityChecker(context.get())));
        //my_thunder->setStateValidityChecker(boost::bind(&ompl_interface::StateValidityChecker::isValid, _1));
    }
    else
        my_thunder->setFilePath("test_without_collision_check");

    my_thunder->setPlanner(allocatePlanner<og::RRTstar>(my_thunder->getSpaceInformation(),
                                             "geometric::RRTstar",
                                             context->getSpecification()));

    my_thunder->setup();

    for(int i = 0; i < number_of_trials; ++i){
        std::cout << std::endl;
        std::cout << "-------------------------------------------------------" << std::endl;
        std::cout << "-------------------------------------------------------" << std::endl;

        ob::ScopedState<> start(my_thunder->getStateSpace());
        vss->sample(start.get());
        ob::ScopedState<> goal(my_thunder->getStateSpace());
        vss->sample(goal.get());
        my_thunder->setStartAndGoalStates(start, goal);

        bool solved = my_thunder->solve(2.0);

        if (solved){
            OMPL_INFORM("Found solution in %g seconds",
                my_thunder->getLastPlanComputationTime());
            //my_thunder->simplifySolution(2.0);
            my_thunder->doPostProcessing();
            my_thunder->save();
        }
        else
            OMPL_INFORM("No solution found");       

        ROS_WARN_STREAM("this is iteration no: " << i);

        //my_thunder->clear();
    }
    my_thunder->save();
    ROS_INFO_STREAM("finished!!!!!");
    ros::waitForShutdown();
    return 0;
}
