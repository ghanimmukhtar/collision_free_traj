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
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group/move_group_capability.h>
#include <moveit/move_group/move_group_context.h>

#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/PlanningScene.h>

#include <iostream>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

int number_of_trials;
bool check_collision;
moveit_msgs::PlanningScene my_scene;
void planning_scene_callback(moveit_msgs::PlanningScene ps_msgs){
    my_scene = ps_msgs;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "generate_baxter_database");
    ros::NodeHandle nh;


    ros::ServiceClient get_motion_plan = nh.serviceClient<moveit_msgs::GetMotionPlan>("plan_kinematic_path", 1);
    ros::Subscriber get_plan_scene = nh.subscribe<moveit_msgs::PlanningScene>("planning_scene", 1, planning_scene_callback);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    /*
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    planning_scene::PlanningScene my_planning_scene(robot_model);

    my_scene.is_diff = true;
    my_planning_scene.setPlanningSceneDiffMsg(my_scene);

    */
    nh.getParam("number_of_trials", number_of_trials);
    nh.getParam("check_collision", check_collision);


    moveit::planning_interface::MoveGroup group("left_arm");
    //group.impl_
    moveit::planning_interface::MoveGroup::Plan my_plan;
    group.setPlannerId("Thunder");

    //group.MoveGroupImpl //to do
    //group.impl_
    //move_group::MoveGroupContextPtr my_context = my_getContext();
    //ompl_interface::PlanningContextManager planning_manager = ompl_interface::OMPLInterface::getPlanningContextManager();


    //moveit_msgs::GetPlanningSceneRequest plan_scene_request;
    //moveit_msgs::GetPlanningSceneResponse plan_scene_response;

    //plan_scene_request.components.components = plan_scene_request.components.WORLD_OBJECT_GEOMETRY;
    //get_plan_scene.call(plan_scene_request, plan_scene_response);

    //std::map<std::string, ompl_interface::ConfiguredPlannerAllocator> my_planners_map = plan_scene_response.scene;
    //ompl_interface::OMPLInterface::getPlanningContext()

    for(int i = 0; i < number_of_trials; ++i){
        moveit_msgs::MotionPlanRequest req2;
        moveit_msgs::GetMotionPlanRequest req3;
        moveit_msgs::GetMotionPlanResponse res3;

        req2.group_name = "left_arm";
        req2.planner_id = "Thunder";

        req2.num_planning_attempts = 20;
        req2.allowed_planning_time = 10.0;

        req2.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints("left_gripper", group.getRandomPose()));

        req3.motion_plan_request = req2;

        get_motion_plan.call(req3, res3);
        group.setRandomTarget();
        group.plan(my_plan);

    }
    return 0;
}
