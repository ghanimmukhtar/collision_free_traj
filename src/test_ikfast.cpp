#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <tf/tf.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>



std::vector<std::string> baxter_joint_names;
std::vector<double> joints_values;

void jocommCallback(sensor_msgs::JointState jo_state)
{
    baxter_joint_names = jo_state.name;
    joints_values = jo_state.position;
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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "baxter_collision_free");
    ros::NodeHandle node;
    //ros::Publisher pub_msg;
    ros::Subscriber sub_jointmsg = node.subscribe<sensor_msgs::JointState>("/robot/joint_states",1,jocommCallback);
    //ros::Publisher planning_scene_publish = node.advertise<moveit_msgs::PlanningScene>("/planning_scene",1);
    ros::Publisher planning_scene_publish = node.advertise<moveit_msgs::PlanningScene>("/planning_scene",1);
    ros::ServiceClient get_motion_plan = node.serviceClient<moveit_msgs::GetMotionPlan>("plan_kinematic_path",1);
    ros::ServiceClient execute_trajectory = node.serviceClient<moveit_msgs::ExecuteKnownTrajectory>("execute_kinematic_path", 1);

    ros::AsyncSpinner my_spinner(4);
    my_spinner.start();
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

    ros::WallDuration sleep_time(5.0);

    moveit_msgs::PlanningScene my_scene;
    my_scene.robot_model_name = "baxter";

    collision_detection::CollisionRequest collision_request;
    collision_request.contacts = true;
    collision_detection::CollisionResult collision_result;

    add_collision_objects(my_scene);
    my_scene.is_diff = true;
    planning_scene_publish.publish(my_scene);
    planning_scene->setPlanningSceneDiffMsg(my_scene);

    robot_state::RobotState current_state = planning_scene->getCurrentState();
    current_state.setVariablePositions(baxter_joint_names, joints_values);
    geometry_msgs::Pose my_pose;
    tf::poseEigenToMsg(current_state.getGlobalLinkTransform("left_gripper"), my_pose);
    planning_scene->setCurrentState(current_state);

    moveit::planning_interface::MoveGroup group("left_arm");
    moveit::planning_interface::MoveGroup::Plan my_plan;
    group.setPlannerId("RRTConnectkConfigDefault");
    geometry_msgs::Pose pose_holder = group.getCurrentPose().pose;
    ROS_ERROR_STREAM("current position is: " << pose_holder.position.x << ", " << pose_holder.position.y << ", " << pose_holder.position.z);
    ROS_ERROR_STREAM("current orientation is: " << pose_holder.orientation.w << ", " << pose_holder.orientation.x << ", " << pose_holder.orientation.y << ", " << pose_holder.orientation.z);

    int number_of_trials, counter = 0;
    node.getParam("number_of_trials", number_of_trials);
    while(ros::ok() && counter < number_of_trials){
        std::cin.ignore();
        //double x, y, z;
        //std::cin >> x >> y >> z;
        /*
        moveit_msgs::MotionPlanRequest req2;
        moveit_msgs::GetMotionPlanRequest req3;
        moveit_msgs::GetMotionPlanResponse res3;

        moveit_msgs::ExecuteKnownTrajectoryRequest traj_req;
        moveit_msgs::ExecuteKnownTrajectoryResponse traj_res;
        geometry_msgs::PointStamped position;

        position.header.frame_id = "/base";
        position.point.x = x;
        position.point.y = y;
        position.point.z = z;
        */
        group.setRandomTarget();
        //group.setPositionTarget(x, y, z);
        group.plan(my_plan);
        /*

        req2.group_name = "left_arm";
        req2.planner_id = "RRTStarkConfigDefault";

        req2.num_planning_attempts = 20;
        req2.allowed_planning_time = 20.0;
        moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("left_gripper", position);

        req2.goal_constraints.push_back(pose_goal);

        req3.motion_plan_request = req2;
        get_motion_plan.call(req3, res3);

        ros::Publisher display_publisher = node.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
        moveit_msgs::DisplayTrajectory display_trajectory;

        /* Visualize the trajectory
        ROS_INFO("Visualizing the trajectory");

        display_trajectory.trajectory_start = res3.motion_plan_response.trajectory_start;
        display_trajectory.trajectory.push_back(res3.motion_plan_response.trajectory);
        display_publisher.publish(display_trajectory);

        sleep_time.sleep();

        traj_req.trajectory = res3.motion_plan_response.trajectory;
        traj_req.wait_for_execution = true;

        execute_trajectory.call(traj_req, traj_res);*/

        counter+=1;
    }
    return 0;
}
