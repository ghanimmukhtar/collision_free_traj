#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/PlanningScene.h>

#include <ompl/base/PlannerDataStorage.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/tools/thunder/SPARSdb.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/tools/thunder/Thunder.h>
#include <ompl/tools/lightning/Lightning.h>
#include <ompl/util/PPM.h>

//#include <ompl_ros_interface/helpers/ompl_ros_conversions.h>

#include <ompl/config.h>
#include "../include/collision_free_traj/config.h"

#include <boost/filesystem.hpp>
#include <iostream>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ot = ompl::tools;


int number_of_trials;
std::vector<double> all_joint_values, left_arm_joint_values(7);
std::vector<std::string> all_joint_names, left_joint_names(7);
sensor_msgs::JointState joint_state_holder;
moveit_msgs::PlanningScene my_scene;



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
        /*
        right_arm_joint_values[0] = jo_state.position[14];
        right_arm_joint_values[1] = jo_state.position[15];
        right_arm_joint_values[2] = jo_state.position[12];
        right_arm_joint_values[3] = jo_state.position[13];
        right_arm_joint_values[4] = jo_state.position[16];
        right_arm_joint_values[5] = jo_state.position[17];
        right_arm_joint_values[6] = jo_state.position[18];
        */
    }
}

bool isStateValid(const ob::State *state, robot_model::RobotModelPtr& robot_model, ompl_interface::ModelBasedPlanningContextPtr& context_ptr)
{
    planning_scene::PlanningScene planning_scene(robot_model);
    //planning_scene.setPlanningSceneDiffMsg(my_scene);

    collision_detection::CollisionRequest req;
    req.contacts = true;
    collision_detection::CollisionResult res;
    collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
    robot_state::RobotState robot_state_holder(robot_model);
    context_ptr->getOMPLStateSpace()->copyToRobotState(robot_state_holder, state);
    planning_scene.checkSelfCollision(req, res, robot_state_holder, acm);

    /*if(res.collision)
        ROS_ERROR("I am in self collision");
    else
        ROS_WARN("nooooo Collision hurray");*/

    return res.collision;
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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "usage_plannerdata");
    ros::NodeHandle nh;

    //actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/robot/limb/left/follow_joint_trajectory", true);
    //ros::Subscriber joint_state_sub = nh.subscribe<sensor_msgs::JointState>("/robot/joint_states", 1, jo_callback);
    //ros::Publisher planning_scene_publish = nh.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    nh.getParam("number_of_trials", number_of_trials);

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

    //robot_state::RobotState start_state(robot_model);
    //start_state.setVariablePositions(all_joint_names, all_joint_values);

    /*planning_scene::PlanningScene planning_scene(robot_model);

    my_scene.robot_model_name = "baxter";
    add_collision_objects(my_scene);
    my_scene.robot_state.joint_state = joint_state_holder;
    my_scene.is_diff = true;
    planning_scene.setCurrentState(start_state);
    planning_scene_publish.publish(my_scene);
    planning_scene.setPlanningSceneDiffMsg(my_scene);*/

    ompl_interface::OMPLInterface oi(robot_model, nh);
    planning_interface::PlannerConfigurationMap my_map = oi.getPlannerConfigurations();
    oi.setPlannerConfigurations(my_map);

    std::string arm_choice = "both_arms";

    ob::StateSpacePtr space = oi.getPlanningContext(arm_choice, "JointModel")->getOMPLStateSpace();
    ot::ThunderPtr my_thunder(new ot::Thunder(space));
    ob::ValidStateSamplerPtr vss = my_thunder->getSpaceInformation()->allocValidStateSampler();

    //moveit::planning_interface::MoveGroup group(arm_choice);

    my_thunder->setFilePath("test");
    //my_thunder->setStateValidityChecker(std::bind(&isStateValid, std::placeholders::_1,
             //                                     robot_model,
                //                                  oi.getPlanningContextManager().getPlanningContext(arm_choice, "JointModel")));
    my_thunder->setup();
    //ompl_interface::ModelBasedStateSpacePtr model_based_state_space = oi.getPlanningContext(arm_choice, "JointModel")->getOMPLStateSpace();
    for(int i = 0; i < number_of_trials; ++i){
        std::cout << std::endl;
        std::cout << "-------------------------------------------------------" << std::endl;
        std::cout << "-------------------------------------------------------" << std::endl;

        if (!my_thunder)
        {
            OMPL_ERROR("Simple setup not loaded");
            return false;
        }

        //start_state.setVariablePositions(all_joint_names, all_joint_values);
        //ROS_WARN_STREAM("perceived starting pose is: " << start_state.getGlobalLinkTransform("left_gripper").translation());
        //ROS_WARN_STREAM("thunder sparsdb planner vertices: " << my_thunder->getExperienceDB()->getSPARSdb()->getNumVertices());


        ob::ScopedState<> start(my_thunder->getStateSpace());
        vss->sample(start.get());
        //model_based_state_space->copyToOMPLState(start.get(), start_state);
        ob::ScopedState<> goal(my_thunder->getStateSpace());
        vss->sample(goal.get());
        my_thunder->setStartAndGoalStates(start, goal);

        bool solved = my_thunder->solve(10.);
        if (solved){
            OMPL_INFORM("Found solution in %g seconds",
                my_thunder->getLastPlanComputationTime());
            my_thunder->simplifySolution(2.0);
            my_thunder->doPostProcessing();
            my_thunder->saveIfChanged();
        }
        else
            OMPL_INFORM("No solution found");       

        /*
        og::PathGeometric my_path = my_thunder->getSolutionPath();


        robot_trajectory::RobotTrajectory my_robot_trajectory(robot_model, arm_choice);
        my_robot_trajectory.setGroupName(arm_choice);
        robot_state::RobotState r_state_holder(robot_model);
        for(size_t i = 0; i < my_path.getStateCount(); ++i){
            model_based_state_space->copyToRobotState(r_state_holder, my_path.getState(i));
            my_robot_trajectory.insertWayPoint(i, r_state_holder, 0.1);
        }
        //my_path.print(std::cout);
        trajectory_processing::IterativeParabolicTimeParameterization time_param;
        if(!time_param.computeTimeStamps(my_robot_trajectory))
            ROS_WARN("Time parametrization for the solution path failed.");
        moveit_msgs::RobotTrajectory robot_trajectory;
        my_robot_trajectory.getRobotTrajectoryMsg(robot_trajectory);
        if (!ac.waitForServer(ros::Duration(2.0)))
          {
            ROS_ERROR("Could not connect to action server");
            return false;
          }
        control_msgs::FollowJointTrajectoryGoal goal_action;
        goal_action.trajectory = robot_trajectory.joint_trajectory;
        goal_action.goal_time_tolerance = ros::Duration(1.0);
        ac.sendGoal(goal_action);
        while(!ac.getState().isDone());
        my_scene.robot_state.joint_state = joint_state_holder;
        my_scene.is_diff = true;
        planning_scene_publish.publish(my_scene);
        planning_scene.setPlanningSceneDiffMsg(my_scene);*/
    }

    ROS_ERROR_STREAM("trying to see number of components ... ");
    ROS_ERROR_STREAM("number of components in my experience is: " << my_thunder->getExperienceDB()->getSPARSdb()->getNumConnectedComponents());

    //ob::PlannerData my_data(my_thunder->getSpaceInformation());
    //my_thunder->getExperienceDB()->getSPARSdb()->getPlannerData(my_data);
    //ROS_ERROR_STREAM("number of vertices in my storage: " << my_data.numVertices());*/
    ROS_INFO_STREAM("finished!!!!!");
    ros::waitForShutdown();
    return 0;
}
