#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <Eigen/Core>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <baxter_core_msgs/SolvePositionIK.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

//global variables to all methods and the main
std::vector<double> joints_values(7);
std::vector <std::string> variable_names(7);
//joints feedback
void jocommCallback(sensor_msgs::JointState jo_state)
{
    joints_values[0] = jo_state.position[5]; joints_values[1] = jo_state.position[6]; joints_values[2] = jo_state.position[3];
    joints_values[3] = jo_state.position[4]; joints_values[4] = jo_state.position[7]; joints_values[5] = jo_state.position[8];
    joints_values[6] = jo_state.position[9];
}

double angle_pi(double angle){
    //keep the angle in Pi
    if (angle > M_PI)  angle = angle - 2*M_PI;
    if (angle < -M_PI)  angle = angle + 2*M_PI;
    return angle;
}

//check if the point is attainable by the baxter arm without collision
bool extend(std::vector<Eigen::Vector4d> &Tree, Eigen::Vector3d xrand,
            ros::ServiceClient& baxter_ik_solver, planning_scene::PlanningScene& my_scene,
            trajectory_msgs::JointTrajectory& my_path, robot_model::RobotModelPtr& baxter_model){
    double max_ang = 1., min_ang = -1., step_dist = 0.1, current_step_dist;
    Eigen::Vector4d xnear;
    double d = std::numeric_limits<double>::infinity(), ang_x = 0.0, ang_y = M_PI, ang_z = 0.0;
    for(int i = 0; i < Tree.size(); i++){
        //look for closest vertex
        Eigen::Vector3d distance(xrand(0) - Tree[i](0), xrand(1) - Tree[i](1), xrand(2) - Tree[i](2));
        double D = distance.norm();
        if(D<d){
            xnear << Tree[i];
            d=D;
        }
    }
    double X = xnear(0), Y = xnear(1), Z = xnear(2), A = xnear(3), xnear_dist = sqrt(X*X + Y*Y + Z*Z);
    double dot = X*xrand(0) + Y*xrand(1) + Z*xrand(2);
    double direction = acos(dot/(xnear_dist*xrand.norm())) - A;
    Eigen::Vector3d connector;
    connector << xrand(0) - xnear(0), xrand(1) - xnear(1), xrand(2) - xnear(2);
    //keep the angle within pi
    direction = angle_pi(direction);
    //limit the direction to prevent too much change in direction
    if (direction > max_ang) direction = max_ang; if (direction < min_ang) direction = min_ang;
    //also limit the distance for each step to 5 cm
    current_step_dist = sqrt(pow(xrand(0) - X, 2) + pow(xrand(1) - Y, 2) + pow(xrand(2) - Z, 2));
    if (current_step_dist > step_dist) current_step_dist = step_dist;
    //now calculate the coordinates of the new point
    Eigen::Vector4d xnew;
    xnew << X + current_step_dist * (connector(0)/connector.norm()),
            Y + current_step_dist * (connector(1)/connector.norm()),
            Z + current_step_dist * (connector(2)/connector.norm()),
            angle_pi(A + direction);
    tf::Quaternion my_angles;
    my_angles.setRPY(ang_x, ang_y, ang_z);
    geometry_msgs::PoseStamped my_desired_pose;
    my_desired_pose.header.frame_id = "/base";
    my_desired_pose.pose.position.x = xnew(0);    my_desired_pose.pose.position.y = xnew(1);    my_desired_pose.pose.position.z = xnew(2);
    my_desired_pose.pose.orientation.w = my_angles.getW();   my_desired_pose.pose.orientation.x = my_angles.getX();    my_desired_pose.pose.orientation.y = my_angles.getY();
    my_desired_pose.pose.orientation.z = my_angles.getZ();
    //solve for desired position
    baxter_core_msgs::SolvePositionIK::Request req;
    baxter_core_msgs::SolvePositionIK::Response res;
    req.pose_stamp.push_back(my_desired_pose);
    baxter_ik_solver.call(req, res);
    //if the solution is unvalid just return flase
    if(!res.isValid[0]) return false;
    //if it is valid check that there is no collision
    robot_state::RobotState current_state(baxter_model);
    current_state = my_scene.getCurrentState();
    //ROS_INFO("second robot state set");
    current_state.setVariablePositions(variable_names, res.joints[0].position);
    collision_detection::CollisionRequest collision_request;
    collision_request.contacts = true;
    collision_detection::CollisionResult collision_result;
    collision_detection::AllowedCollisionMatrix acm = my_scene.getAllowedCollisionMatrix();
    /*std::vector<std::string> collision_names;
    acm.getAllEntryNames(collision_names);
    for(int i = 0; i < collision_names.size(); ++i)
        std::cout << "collision pair: " << i << " is: " << collision_names[i] << std::endl;*/
    collision_result.clear();
    my_scene.checkCollision(collision_request, collision_result, current_state, acm);
    if (collision_result.collision){
        ROS_WARN_STREAM("There is collision here, we look for another point");
        collision_detection::CollisionResult::ContactMap::const_iterator it;
        for(it = collision_result.contacts.begin();
            it != collision_result.contacts.end();
            ++it)
        {
          ROS_INFO("Contact between: %s and %s",
                   it->first.first.c_str(),
                   it->first.second.c_str());
        }
        return false;
    }
    else{
        //if there is no collision add the point to the tree and the joint solution to the path
        Tree.push_back(xnew);
        //std::cout << "new added point is: \n" << xnew << std::endl;
        //std::cout << "************************************" << std::endl;
        trajectory_msgs::JointTrajectoryPoint pt;
        pt.positions = res.joints[0].position;
        pt.velocities.resize(7, 0.0); pt.accelerations.resize(7, 0.0); pt.effort.resize(7, 0.0);
        pt.time_from_start = ros::Duration(0.5);
        my_path.points.push_back(pt);
        return true;
    }
}

trajectory_msgs::JointTrajectory RRT_path_generator(Eigen::Vector3d start, Eigen::Vector3d goal,
                                                    ros::ServiceClient& baxter_ik_solver,
                                                    planning_scene::PlanningScene& my_scene,
                                                    robot_model::RobotModelPtr& baxter_model){
   trajectory_msgs::JointTrajectory my_path, goal_path;
   robot_state::RobotState updated_state(baxter_model);
   //define joints name for the trajectory
   my_path.joint_names.push_back("left_s0");
   my_path.joint_names.push_back("left_s1");
   my_path.joint_names.push_back("left_e0");
   my_path.joint_names.push_back("left_e1");
   my_path.joint_names.push_back("left_w0");
   my_path.joint_names.push_back("left_w1");
   my_path.joint_names.push_back("left_w2");
   //define first point in the trajectory which is the current joints positions
   trajectory_msgs::JointTrajectoryPoint pt;
   pt.positions = joints_values;
   pt.velocities.resize(7, 0.0); pt.accelerations.resize(7, 0.0); pt.effort.resize(7, 0.0);
   pt.time_from_start = ros::Duration(0.5);
   my_path.points.push_back(pt);
   std::vector<Eigen::Vector4d> TreeInit, TreeGoal, swap;
   Eigen::Vector4d start_point(start(0), start(1), start(2), 0.0), goal_point(goal(0), goal(1), goal(2), M_PI);
   double Radius = 1.2, step_size = 0.1;
   //initialize start tree and goal tree
   TreeInit.push_back(start_point); TreeGoal.push_back(goal_point);
   //build the biderctional path from start point to goal point
   bool my_Exit = false;
   int count = 0;
   while(!my_Exit){
       bool AddInit = false, AddGoal = false;
       Eigen::Vector3d xgoal;
       srand((unsigned)time(NULL));
       double phi = ((double)rand()/(double)RAND_MAX) * 2 * M_PI, //random angle between 0 and 2Pi
               costheta = ((double)rand()/(double)RAND_MAX) * 2 - 1, //random double between -1 and 1 so that we get random angle (theta) between 0 and Pi
               theta = acos(costheta), //random theta angle between 0 and Pi
               my_u = ((double)rand()/(double)RAND_MAX), //random double that will be used to produce a length under the radius (length of extended Baxter arm)
               r = Radius * cbrt(my_u); //random length represent the distance of the random point from origin
       //convert the random spherical coordinate to cartesian ones
       double x_cor = r * sin( theta) * cos( phi ),
               y_cor = r * sin( theta) * sin( phi ),
               z_cor = r * cos( theta );
       Eigen::Vector3d xrand(x_cor, y_cor, z_cor);
       //extend start tree
       //ROS_INFO("first robot state set");
       updated_state.setVariablePositions(variable_names, my_path.points[my_path.points.size() - 1].positions);
       my_scene.setCurrentState(updated_state);
       if((count%2) == 0)
           AddInit = extend(TreeInit, xrand, baxter_ik_solver, my_scene, my_path, baxter_model);
       else
           AddInit = extend(TreeInit, xrand, baxter_ik_solver, my_scene, goal_path, baxter_model);
       if(AddInit){
           //extend goal tree
           //ROS_INFO("third robot state set");
           if(goal_path.points.size() > 0)
               updated_state.setVariablePositions(variable_names, goal_path.points[goal_path.points.size() - 1].positions);
           my_scene.setCurrentState(updated_state);
           xgoal << TreeInit[TreeInit.size() - 1](0), TreeInit[TreeInit.size() - 1](1), TreeInit[TreeInit.size() - 1](2);
           if((count%2) == 0)
               AddGoal = extend(TreeGoal, xgoal, baxter_ik_solver, my_scene, goal_path, baxter_model);
           else
               AddGoal = extend(TreeGoal, xgoal, baxter_ik_solver, my_scene, my_path, baxter_model);
       }
       //check if we reach the goal
       if(AddGoal && AddInit){
           Eigen::Vector3d qinit, qgoal;
           qinit << TreeInit[TreeInit.size() - 1](0), TreeInit[TreeInit.size() - 1](1), TreeInit[TreeInit.size() - 1](2);
           qgoal << TreeGoal[TreeGoal.size() - 1](0), TreeGoal[TreeGoal.size() - 1](1), TreeGoal[TreeGoal.size() - 1](2);
           double dist_goal_start = sqrt(pow(qgoal(0) - qinit(0),2) + pow(qgoal(1) - qinit(1),2) + pow(qgoal(2) - qinit(2),2));
           double PsiInit = TreeInit[TreeInit.size() - 1](3), PsiGoal = TreeGoal[TreeGoal.size() - 1](3), Aig = 0.0, Agi = 0.0;
           double my_angle = angle_pi(PsiInit - PsiGoal + M_PI);
           if (dist_goal_start < 2*step_size)
               my_Exit = true;
       }
       count = count + 1;
       if (count > 10000){
           ROS_ERROR("************stop it is enough **********************");
           my_Exit = true;
       }
       swap = TreeInit;
       TreeInit = TreeGoal;
       TreeGoal = swap;
   }
   //we add the goal path to path from the end downwards the beginning
   for (int i = goal_path.points.size() - 1; i > -1; i--){
       my_path.points.push_back(goal_path.points[i]);
   }
   //correct time from start
   if(my_path.points.size() > 1)
       for(int i = 0; i < my_path.points.size(); ++i)
           my_path.points[i].time_from_start = ros::Duration(i*1 + 1);
   std::cout << "start tree final size is:" << TreeInit.size() << std::endl;
   std::cout << "goal tree final size is: " << TreeGoal.size() << std::endl;
   std::cout << "and trajectory final size is: " << my_path.points.size() << std::endl;
   return my_path;
}


int main(int argc, char **argv)
{
    //initialization
    ros::init(argc, argv, "complete_algorithm");
    ros::NodeHandle node;
    ros::Subscriber sub_jointmsg = node.subscribe<sensor_msgs::JointState>("/robot/joint_states",1,jocommCallback);
    ros::Publisher planning_scene_publisher = node.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::ServiceClient ik_solver = node.serviceClient<baxter_core_msgs::SolvePositionIK>("/ExternalTools/left/PositionKinematicsNode/IKService");
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/robot/limb/left/follow_joint_trajectory", true);
    ros::AsyncSpinner my_spinner(1);
    my_spinner.start();
    usleep(1e6);
    //planning scene and collision objects
    variable_names[0] = "left_s0"; variable_names[1] = "left_s1"; variable_names[2] = "left_e0";
    variable_names[3] = "left_e1"; variable_names[4] = "left_w0"; variable_names[5] = "left_w1";
    variable_names[6] = "left_w2";
    robot_model_loader::RobotModelLoader baxter_model_loader("robot_description");
    robot_model::RobotModelPtr baxter_model = baxter_model_loader.getModel();
    planning_scene::PlanningScene baxter_planning_scene(baxter_model);
    robot_state::RobotState baxter_state(baxter_model);
    baxter_state.setVariablePositions(variable_names, joints_values);
    Eigen::VectorXd my_values;
    baxter_state.copyJointGroupPositions(baxter_state.getRobotModel()->getJointModelGroup("left_arm"), my_values);
    std::cout << "left arm joints values are: \n" << my_values << std::endl;
    moveit_msgs::PlanningScene my_scene;
    my_scene.robot_model_name = "baxter";
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
    box_pose.position.z =  0.1;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    ROS_INFO("Add an object into the world");
    my_scene.world.collision_objects.push_back(collision_object);
    my_scene.is_diff = true;
    //planning_scene_publish.publish(my_scene);
    baxter_planning_scene.setPlanningSceneDiffMsg(my_scene);
    baxter_planning_scene.setCurrentState(baxter_state);
    planning_scene_publisher.publish(my_scene);
    Eigen::Vector3d start, pos_goal;
    start << 0.0, 0.8, 0.2;
    pos_goal << 0.8, 0.0, 0.2;
    trajectory_msgs::JointTrajectory my_trajectory = RRT_path_generator(start, pos_goal, ik_solver, baxter_planning_scene, baxter_model);
    my_trajectory.header.stamp = ros::Time::now();
    if (!ac.waitForServer(ros::Duration(2.0)))
    {
      ROS_ERROR("Could not connect to action server");
    }
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = my_trajectory;
    goal.goal_time_tolerance = ros::Duration(1);
    ac.sendGoal(goal);
    while(!ac.getState().isDone());
    return 0;
}
