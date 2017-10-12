#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>

#include <eigen_conversions/eigen_msg.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/ompl_interface/detail/state_validity_checker.h>

#include <ompl/base/PlannerDataStorage.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>

//test this with moveit (planning context manager)
//#include <ompl/geometric/planners/experience/ThunderRetrieveRepair.h>

#include <ompl/tools/thunder/SPARSdb.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/tools/thunder/Thunder.h>
#include <ompl/tools/lightning/Lightning.h>
#include <ompl/util/PPM.h>
#include <ompl/config.h>
#include <moveit/ompl_interface/model_based_planning_context.h>
#include "../include/collision_free_traj/config.h"

#include <boost/filesystem.hpp>
#include <iostream>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

#include <pcl/surface/convex_hull.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <image_processing/tools.hpp>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <tf/transform_listener.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ot = ompl::tools;


int number_of_trials, size_initial_db;
bool check_collision, controlled;
std::vector<double> all_joint_values, left_arm_joint_values(7);
std::vector<std::string> all_joint_names, left_joint_names(7);
sensor_msgs::JointState joint_state_holder;
moveit_msgs::PlanningScene my_scene;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr my_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

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

void cloud_filler_callback(sensor_msgs::PointCloud2 the_cloud){
    pcl::fromROSMsg(the_cloud, *my_cloud);

    //ROS_ERROR_STREAM("I am saving the cloud: " << the_cloud.header.frame_id);
    pcl::PassThrough<pcl::PointXYZRGBA> pass;
    pass.setInputCloud (my_cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 1.1);
    pass.filter (*my_cloud);

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

//Convert object position from camera frame to robot frame
void convert_mesh_vertices_to_robot_base(std::vector<Eigen::Vector3d>& vertex_pose_in_camera_frame, std::vector<Eigen::Vector3d>& vertex_pose_in_robot_frame){
    tf::TransformListener listener;
    tf::StampedTransform stamped_transform;

    try{
        listener.lookupTransform("/camera_depth_optical_frame", "/base",
                                 ros::Time::now(), stamped_transform);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    std::vector<Eigen::Vector3d>::iterator itr;
    for(itr = vertex_pose_in_camera_frame.begin(); itr != vertex_pose_in_camera_frame.end(); ++itr){
        geometry_msgs::PointStamped camera_point;
        geometry_msgs::PointStamped base_point;
        camera_point.header.frame_id = "/camera_depth_optical_frame";

        //we'll just use the most recent transform available for our simple example
        camera_point.header.stamp = ros::Time();

        //just an arbitrary point in space
        camera_point.point.x = (*itr)(0);
        camera_point.point.y = (*itr)(1);
        camera_point.point.z = (*itr)(2);

        try{

            listener.transformPoint("/base", camera_point, base_point);

            ROS_INFO("camera_link: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
                     camera_point.point.x, camera_point.point.y, camera_point.point.z,
                     base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
        }
        catch(tf::TransformException& ex){
            ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
        }
        Eigen::Vector3d point_in_robot_frame;
        point_in_robot_frame << base_point.point.x,
                base_point.point.y,
                base_point.point.z;
        vertex_pose_in_robot_frame.push_back(point_in_robot_frame);
    }
}


std::vector<geometry_msgs::Point> convert_eigen_point_geometry(std::vector<Eigen::Vector3d>& vertex_list){
    ROS_ERROR_STREAM("i am convering eigen vectors into geometry msgs, the size is: " << vertex_list.size());
    std::vector<Eigen::Vector3d>::iterator itr;
    std::vector<geometry_msgs::Point> output_vertices;
    for(itr = vertex_list.begin(); itr != vertex_list.end(); ++itr){
        geometry_msgs::Point current_vertex;
        tf::pointEigenToMsg(*itr, current_vertex);
        output_vertices.push_back(current_vertex);
    }
    return output_vertices;
    /*pcl::ConvexHull<pcl::PointXYZ> hull_extractor;
    pcl::PointCloud<pcl::PointXYZ> hull_cloud;
    //my_cloud = my_cloud_class.Ptr;
    hull_extractor.setInputCloud(my_cloud);
    hull_extractor.reconstruct(hull_cloud);

    if(hull_cloud.empty()){
        ROS_ERROR("unable to compute the convex hull");
        return false;
    }

    for(auto it = hull_cloud.points.begin(); it != hull_cloud.points.end(); ++it){
        geometry_msgs::Point new_point;
        new_point.x = it->x;
        new_point.y = it->y;
        new_point.z = it->z;
        vertex_list.push_back(new_point);
    }

    return true;*/
}

void add_collision_objects(moveit_msgs::PlanningScene& my_scene){
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "/base";
    collision_object.id = "box1";
    //try to use convexhull extraction
    /*shape_msgs::Mesh primitive_1;
    shapes::ShapeMsg co_mesh_msg;

    //my_cloud = my_cloud_class.ConstPtr;
    std::vector<Eigen::Vector3d> vertex_list, vertex_list_in_camera_frame;

    image_processing::tools::extract_convex_hull(my_cloud, vertex_list_in_camera_frame);

    convert_mesh_vertices_to_robot_base(vertex_list_in_camera_frame, vertex_list);

    EigenSTL::vector_Vector3d vertices_source;
    std::vector<Eigen::Vector3d>::iterator itr;
    for(itr = vertex_list.begin(); itr != vertex_list.end(); ++itr)
        vertices_source.push_back(*itr);

    shapes::Mesh* m = shapes::createMeshFromVertices(vertices_source);
    shapes::constructMsgFromShape(m, co_mesh_msg);
    primitive_1  = boost::get<shape_msgs::Mesh>(co_mesh_msg);

    std::vector<geometry_msgs::Point> vertices = convert_eigen_point_geometry(vertex_list);
    primitive_1.vertices = vertices;*/

    //test with simple primitive
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.8;
    primitive.dimensions[1] = 1.4;
    primitive.dimensions[2] = 0.1;    

    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x =  0.7;
    box_pose.position.y =  0.0;
    box_pose.position.z =  -0.15;
    //collision_object.meshes.push_back(primitive_1);
    //collision_object.mesh_poses.push_back(box_pose);
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    ROS_INFO("Add an object into the world");
    my_scene.world.collision_objects.push_back(collision_object);
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
    ros::init(argc, argv, "test_baxter_database");
    ros::NodeHandle nh;

    //actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/robot/limb/left/follow_joint_trajectory", true);
    //ros::Subscriber joint_state_sub = nh.subscribe<sensor_msgs::JointState>("/robot/joint_states", 1, jo_callback);
    //ros::Subscriber cloud_filler_sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 10, cloud_filler_callback);
    ros::Publisher planning_scene_publish = nh.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);

    ros::AsyncSpinner spinner(14);
    spinner.start();
    nh.getParam("number_of_trials", number_of_trials);
    nh.getParam("size_initial_db", size_initial_db);
    nh.getParam("check_collision", check_collision);
    nh.getParam("controlled", controlled);

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

    robot_state::RobotState start_state(robot_model);
    //use this to give meaningful goals/starts
    //start_state.setFromIK()
    //start_state.setVariablePositions(all_joint_names, all_joint_values);
    start_state.setToRandomPositions();


    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
    my_scene.robot_model_name = "baxter";

    add_collision_objects(my_scene);
    //my_scene.robot_state.joint_state = joint_state_holder;
    moveit::core::robotStateToRobotStateMsg(start_state, my_scene.robot_state);
    my_scene.is_diff = true;
    planning_scene->setCurrentState(start_state);
    planning_scene_publish.publish(my_scene);
    planning_scene->setPlanningSceneDiffMsg(my_scene);

    ompl_interface::OMPLInterface oi(robot_model, nh);
    planning_interface::PlannerConfigurationMap my_map = oi.getPlannerConfigurations();
    oi.setPlannerConfigurations(my_map);

    std::string arm_choice = "left_arm";

    ompl_interface::PlanningContextManager context_manager = oi.getPlanningContextManager();
    ompl_interface::ModelBasedPlanningContextPtr context = context_manager.getPlanningContext(arm_choice, "JointModel");
    context->setPlanningScene(planning_scene);
    ob::StateSpacePtr space = context->getOMPLStateSpace();
    ot::ThunderPtr my_thunder(new ot::Thunder(space));
    ob::ValidStateSamplerPtr vss = my_thunder->getSpaceInformation()->allocValidStateSampler();

    //moveit::planning_interface::MoveGroup group(arm_choice);
    //moveit::planning_interface::MoveGroup::Plan my_plan;
    //group.setPlannerId("RRTConnectkConfigDefault");
    //group.setStartState(start_state);
    if(check_collision)
        my_thunder->setFilePath("test_collision_check");
        //my_thunder->setFilePath("testfari");
    else
        my_thunder->setFilePath("test_without_collision_check");

    my_thunder->setStateValidityChecker(ob::StateValidityCheckerPtr(new ompl_interface::StateValidityChecker(context.get())));
    /*my_thunder->setPlanner(allocatePlanner<og::RRTstar>(my_thunder->getSpaceInformation(),
                                             "geometric::RRTstar",
                                             context->getSpecification()));*/

    my_thunder->setup();
    int number_at_start = my_thunder->getExperienceDB()->getSPARSdb()->getNumVertices();
    ompl_interface::ModelBasedStateSpacePtr model_based_state_space = context->getOMPLStateSpace();
    ob::ScopedState<> start(my_thunder->getStateSpace());
    //generate random start configuration
    //vss->sample(start.get());
    //model_based_state_space->copyToRobotState(start_state, start.get());
    //get start configuration from joint states
    model_based_state_space->copyToOMPLState(start.get(), start_state);
    moveit::core::robotStateToRobotStateMsg(start_state, my_scene.robot_state);
    my_scene.is_diff = true;
    planning_scene_publish.publish(my_scene);
    planning_scene->setPlanningSceneDiffMsg(my_scene);

    //group.setStartState(start_state);
    context->setPlanningScene(planning_scene);

    int solutions_from_recall = 0, solutions_from_scratch = 0, no_solutions_thunder = 0, no_solutions_rrt = 0, size_data_base_before = 0;

    /*ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition( 2.0 );

    for(int i = 0; i < size_initial_db; ++i){
        ob::ScopedState<> random_state(my_thunder->getStateSpace());
        vss->sample(random_state.get());
        my_thunder->getExperienceDB()->getSPARSdb()->addStateToRoadmap(ptc, random_state.get());
        if(i%1000 == 0)
            ROS_INFO_STREAM("advancing well ... at: " << i);
    }*/

    size_data_base_before = my_thunder->getExperienceDB()->getSPARSdb()->getNumVertices();
    for(int i = 0; i < number_of_trials; ++i){
        if(controlled)
            std::cin.ignore();
        std::cout << std::endl;
        std::cout << "-------------------------------------------------------" << std::endl;
        std::cout << "-------------------------------------------------------" << std::endl;

        /*start_state.setVariablePositions(all_joint_names, all_joint_values);
        //group.setStartState(start_state);

        ROS_WARN_STREAM("perceived starting pose is: " << start_state.getGlobalLinkTransform("left_gripper").translation());
        geometry_msgs::PoseStamped group_pose = group.getCurrentPose();
        ROS_WARN_STREAM("group perceived pose is for X: " << group_pose.pose.position.x
                        << " for Y: " << group_pose.pose.position.y
                        << " for Z: " << group_pose.pose.position.z
                        << " for Rotw: " << group_pose.pose.orientation.w
                        << " for Rotx: " << group_pose.pose.orientation.x
                        << " for Roty: " << group_pose.pose.orientation.y
                        << " for Rotz: " << group_pose.pose.orientation.z);
        ob::ScopedState<> start(my_thunder->getStateSpace());
        vss->sample(start.get());

        std::vector<double> start_ompl_state = start.reals();
        std::vector<double>::iterator itr;
        for(itr = start_ompl_state.begin(); itr != start_ompl_state.end(); ++itr)
            ROS_WARN_STREAM("omple state value is: " << (*itr));

        model_based_state_space->copyToRobotState(start_state, start.get());
        moveit::core::robotStateToRobotStateMsg(start_state, my_scene.robot_state);
        my_scene.is_diff = true;
        planning_scene_publish.publish(my_scene);
        planning_scene->setPlanningSceneDiffMsg(my_scene);

        context->setPlanningScene(planning_scene);*/

        ob::ScopedState<> goal(my_thunder->getStateSpace());
        vss->sample(goal.get());
        /*
        //test of moveit can solve for this random goal
        group.setJointValueTarget(goal.reals());
        if(group.plan(my_plan)){

            //try to get the time it took the planner to plan

            no_solutions_rrt += 1;
        }
        else
            ROS_ERROR("move group couldn't plan .... let's see thunder");
        group.setRandomTarget();
        robot_state::RobotState robot_goal = group.getJointValueTarget();
        model_based_state_space->copyToOMPLState(goal.get(), robot_goal);
        */

        my_thunder->setStartAndGoalStates(start, goal);

        bool solved = my_thunder->solve(4.);
        if (solved){
            if(my_thunder->getSolutionPlannerName() == my_thunder->getRetrieveRepairPlanner().getName())
                solutions_from_recall += 1;
            if(my_thunder->getSolutionPlannerName() == my_thunder->getPlanner()->getName())
                solutions_from_scratch += 1;
            OMPL_INFORM("Found solution in %g seconds",
                my_thunder->getLastPlanComputationTime());
            my_thunder->simplifySolution(2.0);
            my_thunder->doPostProcessing();

            //my_thunder->saveIfChanged();

            // convert the ompl path to robot trajectory
            og::PathGeometric my_path = my_thunder->getSolutionPath();
            robot_trajectory::RobotTrajectory my_robot_trajectory(robot_model, arm_choice);
            //context->convertPath(my_path, my_robot_trajectory);
            my_robot_trajectory.setGroupName(arm_choice);
            robot_state::RobotState r_state_holder(robot_model);
            for(size_t i = 0; i < my_path.getStateCount(); ++i){
                model_based_state_space->copyToRobotState(r_state_holder, my_path.getState(i));
                my_robot_trajectory.insertWayPoint(i, r_state_holder, 0.1);
            }
            trajectory_processing::IterativeParabolicTimeParameterization time_param;
            if(!time_param.computeTimeStamps(my_robot_trajectory))
                ROS_WARN("Time parametrization for the solution path failed.");
            //moveit_msgs::RobotTrajectory robot_trajectory;

            //robot_state::RobotState current_r_state = planning_scene->getCurrentState();
            //current_r_state.setVariablePositions(all_joint_names, all_joint_values);
            my_robot_trajectory.getRobotTrajectoryMsg(my_plan.trajectory_);
            //moveit::core::robotStateToRobotStateMsg(current_r_state, my_plan.start_state_);
            group.execute(my_plan);

            ROS_WARN_STREAM("trajectory size is: " << my_plan.trajectory_.joint_trajectory.points.size());
            /*std::vector<double>::iterator itr2;
            for(itr2 = my_plan.trajectory_.joint_trajectory.points[0].positions.begin();
                itr2 != my_plan.trajectory_.joint_trajectory.points[0].positions.end(); ++itr2)
                ROS_WARN_STREAM("trajectory first state value is: " << (*itr2));


            if (!ac.waitForServer(ros::Duration(2.0)))
              {
                ROS_ERROR("Could not connect to action server");
                return false;
              }
            control_msgs::FollowJointTrajectoryGoal goal_action;
            goal_action.trajectory = my_robot_trajectory.joint_trajectory;
            goal_action.goal_time_tolerance = ros::Duration(1.0);
            ac.sendGoal(goal_action);
            while(!ac.getState().isDone());*/
            no_solutions_thunder += 1;
        }
        else
            OMPL_INFORM("No solution found");

        ROS_WARN_STREAM("this is iteration no: " << i);

        //my_scene.robot_state.joint_state = joint_state_holder; //in case of executing a trajectory update the planning scene
        my_scene.is_diff = true;
        planning_scene_publish.publish(my_scene);
        planning_scene->setPlanningSceneDiffMsg(my_scene);
        my_thunder->clear();
        context->setPlanningScene(planning_scene);

        collision_detection::CollisionWorldConstPtr my_world = context->getPlanningScene()->getCollisionWorld();
        std::vector<std::string> my_world_objects = my_world->getWorld()->getObjectIds();
        ROS_WARN_STREAM("object number is: " << my_world_objects.size());
        std::vector<std::string>::iterator object_itr;
        for(object_itr = my_world_objects.begin(); object_itr != my_world_objects.end(); ++object_itr)
            ROS_WARN_STREAM("name of object is: " << (*object_itr));

        my_thunder->clear();

        /*std::vector<std::string> allowed_collision;
        context->getPlanningScene()->getAllowedCollisionMatrix().getAllEntryNames(allowed_collision);
        std::vector<std::string>::iterator name_itr;
        for(name_itr = allowed_collision.begin(); name_itr != allowed_collision.end(); ++name_itr)
            ROS_INFO_STREAM("allwed joint is: " << (*name_itr));

        std::vector<double>::iterator value_itr;
        std::vector<std::string>::iterator name_itr = all_joint_names.begin();
        for(value_itr = all_joint_values.begin(); value_itr != all_joint_values.end(); ++value_itr){
            ROS_INFO_STREAM("joint: " << (*name_itr) << " value is: " << (*value_itr));
            ++name_itr;
        }
        */
    }
    my_thunder->save();
    //ROS_ERROR_STREAM("number of components in my experience is: " << my_thunder->getExperienceDB()->getSPARSdb()->getNumConnectedComponents());
    ROS_ERROR_STREAM("total number of trials is: " << number_of_trials);
    ROS_ERROR_STREAM("number of points with solution from RRTconnect: " << no_solutions_rrt);
    ROS_ERROR_STREAM("number of points without solution from RRTconnect: " << number_of_trials - no_solutions_rrt);
    ROS_ERROR_STREAM("number of vertices at creation should be zero and it is: " << number_at_start);
    ROS_ERROR_STREAM("number of vertices originally is: " << size_data_base_before);
    ROS_ERROR_STREAM("number of vertices added after is: " << my_thunder->getExperienceDB()->getSPARSdb()->getNumVertices() - size_data_base_before);
    ROS_ERROR_STREAM("number of solutions from recall is: " << solutions_from_recall);
    ROS_ERROR_STREAM("number of solutions from scratch is: " << solutions_from_scratch);
    ROS_ERROR_STREAM("number of points without solution from thunder: " << number_of_trials - no_solutions_thunder);
    ROS_INFO_STREAM("finished!!!!!");
    //ros::waitForShutdown();
    return 0;
}
