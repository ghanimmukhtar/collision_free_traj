#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>

#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <ompl/geometric/SimpleSetup.h>
#include <octomap/octomap.h>
#include <octomap/math/Pose6D.h>
#include <octomap/math/Utils.h>
#include <octomap/math/Vector3.h>
#include <octomap/OcTreeNode.h>

std::vector<double> joints_values(19);
std::vector <std::string> variable_names(19);
void jocommCallback(sensor_msgs::JointState jo_state)
{
    //simulation
    for(int i = 0; i < jo_state.name.size(); i++){
        variable_names[i] = jo_state.name[i];
        joints_values[i] = jo_state.position[i];
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "refined_baxter_collision_free");
    ros::AsyncSpinner my_spinner(4);
    my_spinner.start();
    ros::NodeHandle node_handle;
    ros::ServiceClient motion_plan_client;
    ros::Subscriber sub_jointmsg;
    //sub_jointmsg = node_handle.subscribe<sensor_msgs::JointState>("/robot/joint_states",1,jocommCallback);
    sub_jointmsg = node_handle.subscribe<sensor_msgs::JointState>("/joint_states",1,jocommCallback);
    ros::Publisher planning_scene_publish = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene",1);
    motion_plan_client = node_handle.serviceClient<moveit_msgs::GetMotionPlan>("plan_kinematic_path");

    moveit::planning_interface::MoveGroup::Plan my_plan;
    moveit::planning_interface::MoveGroup group("left_arm");
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setPlanningTime(60.0f);
    group.setEndEffectorLink(group.getEndEffectorLink());
    group.setPoseReferenceFrame("world");

    //the goal pose
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 0.0254697;
    target_pose1.orientation.x = 0.14077;
    target_pose1.orientation.y = 0.989647;
    target_pose1.orientation.z = 0.0116586;
    target_pose1.position.x = atof(argv[1]);
    target_pose1.position.y = atof(argv[2]);
    target_pose1.position.z = atof(argv[3]);


    std::vector<double> position_tolerances(3,0.01f);
    std::vector<double> orientation_tolerances(3,0.01f);
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "world";
    p.pose.orientation = target_pose1.orientation; p.pose.position = target_pose1.position;
    moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(group.getEndEffectorLink(),p,position_tolerances,
            orientation_tolerances);


    //collision objects
    moveit_msgs::AttachedCollisionObject attached_object, attached_object2;
    attached_object.link_name = "base"; attached_object2.link_name = "base";
    /* The header must contain a valid TF frame*/
    attached_object.object.header.frame_id = "base"; attached_object2.object.header.frame_id = "base";
    /* The id of the object */
    attached_object.object.id = "box"; attached_object2.object.id = "second_box";
    /* A default pose */
    geometry_msgs::Pose pose; geometry_msgs::Pose pose_2;
    pose.orientation.w = 1.0;   pose.position.x =  0.5;    pose.position.y =  0.35;    pose.position.z =  0.1;
    pose_2.orientation.w = 1.0;   pose_2.position.x =  0.5;    pose_2.position.y =  -0.2;    pose_2.position.z =  0.2;
    /* Define a box to be attached */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.4;    primitive.dimensions[1] = 0.1;    primitive.dimensions[2] = 0.4;
    attached_object.object.primitives.push_back(primitive); //attached_object2.object.primitives.push_back(primitive);
    attached_object.object.primitive_poses.push_back(pose); //attached_object2.object.primitive_poses.push_back(pose_2);
    attached_object.object.operation = attached_object.object.ADD; //attached_object2.object.operation = attached_object2.object.ADD;
    ROS_INFO("Adding two objects into the world at the locations specified by poses.");
    moveit_msgs::PlanningScene planning_scene_2;
    planning_scene_2.is_diff = true;
    planning_scene_2.world.collision_objects.push_back(attached_object.object); //planning_scene_2.world.collision_objects.push_back(attached_object2.object);

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    planning_scene::PlanningScene my_plan_scene(robot_model);
    collision_detection::CollisionRequest collision_request;
    collision_request.contacts = true; collision_request.max_contacts = 100;
    collision_detection::CollisionResult collision_result;
    my_plan_scene.setPlanningSceneDiffMsg(planning_scene_2);
    ROS_ERROR_STREAM("***************** i am here ********************.");

    // creating motion plan request
    /*moveit_msgs::GetMotionPlan motion_plan;
    moveit_msgs::MotionPlanRequest &req = motion_plan.request.motion_plan_request;
    moveit_msgs::MotionPlanResponse &res = motion_plan.response.motion_plan_response;
    moveit_msgs::RobotState start_robot_state;

    robot_state::RobotState current_state = my_plan_scene.getCurrentState();
    usleep(1e-6);
    current_state.setVariablePositions(variable_names,joints_values);

    planning_scene_publish.publish(planning_scene_2);
    my_plan_scene.setCurrentState(current_state);
    robot_state::robotStateToRobotStateMsg(current_state,start_robot_state);


    req.start_state = start_robot_state;
    req.start_state.is_diff = true;

    req.group_name = "left_arm";
    req.goal_constraints.push_back(pose_goal);
    req.allowed_planning_time = 60.0f;
    req.num_planning_attempts = 1;

    // request motion plan
    bool success = false;
    if(motion_plan_client.call(motion_plan) && res.error_code.val == res.error_code.SUCCESS)
    {
        // saving motion plan results

        my_plan.start_state_ = res.trajectory_start;
        my_plan.trajectory_ = res.trajectory;
        success = true;
    }
    if (success)
        group.execute(my_plan);*/


    collision_detection::AllowedCollisionMatrix acm = my_plan_scene.getAllowedCollisionMatrix();
    robot_state::RobotState copied_state = my_plan_scene.getCurrentState();
    Eigen::VectorXd left_joints_values, right_joints_values;
    copied_state.copyJointGroupPositions(copied_state.getRobotModel()->getJointModelGroup("left_arm"),left_joints_values);
    std::cout << "left arm joints values are: \n" << left_joints_values << "\n*****************************************************" << std::endl;
    copied_state.copyJointGroupPositions(copied_state.getRobotModel()->getJointModelGroup("right_arm"),right_joints_values);
    std::cout << "right arm joints values are: \n" << right_joints_values << std::endl;
    std::vector<std::string> my_names;
    acm.getAllEntryNames(my_names);
    for(int i = 0; i < my_names.size(); i++)
        ROS_INFO("Name of pairs no: %d is %s", i, my_names[i].c_str());

    collision_result.clear();
    my_plan_scene.checkCollision(collision_request,collision_result,copied_state,acm);
    collision_detection::CollisionWorldConstPtr my_world = my_plan_scene.getCollisionWorld();
    std::vector<std::string> my_world_objects = my_world->getWorld()->getObjectIds();
    std::cout << "object number is: " << my_world_objects.size() << std::endl;
    for(int i = 0; i < my_world_objects.size(); i++)
        std::cout << "name of object number: " << i << " is: " << my_world_objects[i] << std::endl;
    ROS_INFO_STREAM("Test 6: Current state is "
                    << (collision_result.collision ? "in" : "not in")
                    << " self collision");
    std::cout << "contact is: " << collision_result.contacts.size() << std::endl;
    std::cout << "contact result is: " << collision_result.collision << std::endl;
    std::cout << "contact number is: " << collision_result.contact_count << std::endl;
    collision_detection::CollisionResult::ContactMap::const_iterator it;
    for(it = collision_result.contacts.begin();
        it != collision_result.contacts.end();
        ++it)
    {
      ROS_INFO("Contact between: %s and %s",
               it->first.first.c_str(),
               it->first.second.c_str());
    }

    return 0;
}
