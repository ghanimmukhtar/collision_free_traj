#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include <boost/timer.hpp>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/bodies.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_request.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group/move_group_context.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <eigen_conversions/eigen_msg.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <yaml-cpp/yaml.h>
#include <baxter_core_msgs/SolvePositionIK.h>

std::vector<std::string> baxter_joint_names;
std::vector<double> joints_values;

void jocommCallback(sensor_msgs::JointState jo_state)
{
    baxter_joint_names = jo_state.name;
    joints_values = jo_state.position;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "baxter_collision_free");
    ros::NodeHandle node;
    //ros::Publisher pub_msg;
    ros::Subscriber sub_jointmsg = node.subscribe<sensor_msgs::JointState>("/robot/joint_states",1,jocommCallback);
    //ros::Publisher collision_object_publisher = node.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
    ros::Publisher planning_scene_publish = node.advertise<moveit_msgs::PlanningScene>("/planning_scene",1);
    ros::ServiceClient baxter_left_ik_solver =
            node.serviceClient<baxter_core_msgs::SolvePositionIK>("/ExternalTools/left/PositionKinematicsNode/IKService");
    ros::ServiceClient get_motion_plan = node.serviceClient<moveit_msgs::GetMotionPlan>("plan_kinematic_path",1);
    //actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/robot/limb/left/follow_joint_trajectory", true);

    ros::AsyncSpinner my_spinner(1);
    my_spinner.start();
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(robot_model);
    moveit_msgs::PlanningScene my_scene;
    my_scene.robot_model_name = "baxter";

    collision_detection::CollisionRequest collision_request;
    collision_request.contacts = true;
    collision_detection::CollisionResult collision_result;
    robot_state::RobotState current_state = planning_scene.getCurrentState();

    current_state.setVariablePositions(baxter_joint_names, joints_values);
    geometry_msgs::Pose my_pose;
    tf::poseEigenToMsg(current_state.getGlobalLinkTransform("left_gripper"), my_pose);

    ROS_INFO_STREAM("position is: " << my_pose.position.x << ", " << my_pose.position.y << ", " << my_pose.position.z );
    ROS_INFO_STREAM("orientation is: " << my_pose.orientation.w << ", " << my_pose.orientation.x << ", " << my_pose.orientation.y << ", " << my_pose.orientation.z );


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


    //shape_msgs::Mesh primitive_1;
    //primitive_1.vertices = what_leni_gives;
    ROS_INFO("Add an object into the world");
    my_scene.world.collision_objects.push_back(collision_object);
    my_scene.is_diff = true;
    planning_scene_publish.publish(my_scene);
    planning_scene.setPlanningSceneDiffMsg(my_scene);
    planning_scene.setCurrentState(current_state);

    /*XmlRpc::XmlRpcValue planners_xml;
    std::vector<std::string> planners_list;
    node.getParam("/move_group/left_arm/planner_configs", planners_xml);
    //ROS_ERROR_STREAM("planners list size is: " << planners_xml.size());
    for(size_t i = 0; i < planners_xml.size(); ++i){

        planners_list.push_back(planners_xml[i]);
        //ROS_ERROR_STREAM("planner number: " << i << " is: " << planners_list[i]);
    }*/
    moveit::planning_interface::MoveGroup group("left_arm");


    group.setPlannerId("RRTConnectkConfigDefault");
    //group.setPlanningTime(10);
    //group.allowReplanning(true);
    //ROS_INFO_STREAM("Planning time is: " << group.getPlanningTime());
    geometry_msgs::Pose pose_holder = group.getCurrentPose().pose;
    ROS_ERROR_STREAM("current position is: " << pose_holder.position.x << ", " << pose_holder.position.y << ", " << pose_holder.position.z);
    ROS_ERROR_STREAM("current orientation is: " << pose_holder.orientation.w << ", " << pose_holder.orientation.x << ", " << pose_holder.orientation.y << ", " << pose_holder.orientation.z);

    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "/base";
    pose.pose.position.x = atof(argv[1]);
    pose.pose.position.y = atof(argv[2]);
    pose.pose.position.z = atof(argv[3]);
    std::vector<double> RPY = group.getCurrentRPY();
    tf::Quaternion quat;
    quat.setRPY(RPY[0], RPY[1], RPY[2]);
    pose.pose.orientation.w = quat.getW();
    pose.pose.orientation.x = quat.getX();
    pose.pose.orientation.y = quat.getY();
    pose.pose.orientation.z = quat.getZ();

    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);

    req.group_name = "left_arm";
    moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("r_wrist_roll_link", pose, tolerance_pose, tolerance_angle);
    req.goal_constraints.push_back(pose_goal);

    //get_motion_plan.call(req, res);
    /*
    while(ros::ok()){
        std::cin.ignore();
        double x, y, z;
        std::cin >> x >> y >> z;
        //geometry_msgs::PoseStamped pose_stamped;

        pose_holder.position.x = x;
        pose_holder.position.y = y;
        pose_holder.position.z = z;

        //pose_stamped.pose = pose_holder;
        //pose_stamped.header.frame_id = "/base";


        int count = 0;
        bool successful_plan = false;
        double angle_step = 0.1;
        moveit::planning_interface::MoveGroup::Plan my_plan;

        //plan to test several orientations
        while(count < 1 && !successful_plan){
            std::vector<double> RPY = group.getCurrentRPY();
            tf::Quaternion quat;
            //quat.setRPY(0, M_PI + count*angle_step, RPY[2]);
            quat.setRPY(RPY[0], RPY[1], RPY[2]);
            pose_holder.orientation.w = quat.getW();
            pose_holder.orientation.x = quat.getX();
            pose_holder.orientation.y = quat.getY();
            pose_holder.orientation.z = quat.getZ();
            group.setPoseTarget(pose_holder);
            moveit_msgs::OrientationConstraint ocm;

            ocm.link_name = group.getEndEffectorLink();
            ocm.header.frame_id = "left_gripper";
            ocm.orientation.x = quat.getX();
            ocm.orientation.y = quat.getY();
            ocm.orientation.z = quat.getZ();
            ocm.orientation.w = quat.getW();
            ocm.absolute_x_axis_tolerance = M_PI;
            ocm.absolute_y_axis_tolerance = M_PI;
            ocm.absolute_z_axis_tolerance = M_PI;

            moveit_msgs::Constraints my_constraints;
            my_constraints.orientation_constraints.push_back(ocm);
            //group.setPathConstraints(my_constraints);
            group.setGoalOrientationTolerance(2*M_PI);
            successful_plan = group.plan(my_plan);
            /*
            if(!successful_plan){
                geometry_msgs::PoseStamped my_desired_pose;
                my_desired_pose.header.frame_id = "/base";
                my_desired_pose.pose = pose_holder;
                baxter_core_msgs::SolvePositionIK::Request req;
                baxter_core_msgs::SolvePositionIK::Response res;
                req.pose_stamp.push_back(my_desired_pose);
                baxter_left_ik_solver.call(req, res);
                if(res.joints[0].position.size() > 0){
                    ROS_INFO_STREAM("trying to print ik solution");
                    for(size_t i = 0; i < res.joints[0].position.size(); ++i)
                        ROS_INFO_STREAM("solution for joint: " << i << " is: " << res.joints[0].position[i]);
                }
                else{
                    for(int j = 0; j < 10; ++j){
                        sensor_msgs::JointState new_state;
                        current_state.setToRandomPositions(current_state.getJointModelGroup("left_arm"));
                        new_state.name = {"left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"};

                        //new_state.position = {-0.08, -1.0, -1.19, 1.94, 0.67, 1.03, -0.5};
                        current_state.copyJointGroupPositions("left_arm", new_state.position);
                        req.seed_angles.clear();
                        req.seed_angles.push_back(new_state);
                        baxter_left_ik_solver.call(req, res);
                        if(res.joints[0].position.size() > 0){
                            ROS_INFO_STREAM("trying to print ik solution");
                            for(size_t i = 0; i < res.joints[0].position.size(); ++i)
                                ROS_INFO_STREAM("solution for joint: " << i << " is: " << res.joints[0].position[i]);
                        }
                    }
                    ROS_ERROR_STREAM("even ik solver found no solution in trial number: " << count);
                }

            }
            count+=1;

        }
        if(successful_plan){
            ROS_INFO_STREAM("trying to go to pose: " << group.getPoseTarget().pose.position.x << ", "  << group.getPoseTarget().pose.position.y << ", "  << group.getPoseTarget().pose.position.z);
            group.execute(my_plan);
        }
        /*
        //loop to try all planners with different planning time
        for(int i = 1; i < 4; ++i){
            while(!successful_plan && count < planners_list.size()){
                group.setPlannerId(planners_list[count]);
                successful_plan = group.plan(my_plan);
                if(!successful_plan)
                    count+=1;
            }
            if(successful_plan){
                //ROS_ERROR("I am here ..................");
                ROS_INFO_STREAM("Planner that found solution is: " << planners_list[count] << " and its time is: " << group.getPlanningTime());
                group.execute(my_plan);
            }
            else
                ROS_WARN("no plan was found with all planners for left arm");
            group.setPlanningTime(group.getPlanningTime() + i*10);
            count = 0;
        }


        /*moveit_msgs::GetMotionPlanRequest motion_plan_request;
        moveit_msgs::GetMotionPlanResponse motion_plan_response;

        motion_plan_request.motion_plan_request.group_name = "left_arm";
        motion_plan_request.motion_plan_request.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints("left_gripper", pose_stamped));
        get_motion_plan.call(motion_plan_request, motion_plan_response);
        if (!ac.waitForServer(ros::Duration(2.0)))
          {
            ROS_ERROR("Could not connect to action server");
          }
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory = motion_plan_response.motion_plan_response.trajectory.joint_trajectory;
        goal.goal_time_tolerance = ros::Duration(1.0);
        ac.sendGoal(goal);
        while(!ac.getState().isDone());
    }


    collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
    robot_state::RobotState copied_state = planning_scene.getCurrentState();
    collision_result.clear();
    planning_scene.checkCollision(collision_request, collision_result, copied_state, acm);
    collision_detection::CollisionWorldConstPtr my_world = planning_scene.getCollisionWorld();
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
    } */

    return 0;
}
