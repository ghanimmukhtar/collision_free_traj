// Core ros functionality like ros::init and spin
#include <ostream>
#include <fstream>
#include <ros/ros.h>
// ROS Trajectory Action server definition
#include <control_msgs/FollowJointTrajectoryAction.h>
// Means by which we communicate with above action-server
#include <actionlib/client/simple_action_client.h>

// Includes the descartes robot model we will be using
#include <descartes_moveit/moveit_state_adapter.h>
// Includes the descartes trajectory type we will be using
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
// Includes the planner we will be using
#include <descartes_planner/dense_planner.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/PlanningSceneWorld.h>
#include <octomap/octomap.h>

typedef std::vector<descartes_core::TrajectoryPtPtr> TrajectoryVec;
typedef TrajectoryVec::const_iterator TrajectoryIter;

std::vector<double> joints_values(19);
std::vector <std::string> variable_names(19);
double length, width, height;
Eigen::Vector3d obstacle_pose;
std::ofstream start_tree, goal_tree;

void jocommCallback(sensor_msgs::JointState jo_state)
{
    //simulation
    for(int i = 0; i < jo_state.name.size(); i++){
        variable_names[i] = jo_state.name[i];
        joints_values[i] = jo_state.position[i];
        //std::cout << jo_state.position[i] << std::endl;
    }
}

/**
  generates collision free cartesian path, it tries to follow a fifth degree polynomial, and when a collision is detected it looks for collision free point near by that makes approach the final goal
  */
TrajectoryVec cart_path_generator(Eigen::Vector3d start, Eigen::Vector3d goal, collision_detection::CollisionWorldConstPtr my_collision_world);


/**
  generates collision free cartesian path, using RRT method
  */
TrajectoryVec RRT_cart_path_generator(Eigen::Vector3d start, Eigen::Vector3d goal);

/**
 * Generates an completely defined (zero-tolerance) cartesian point from a pose
 */
descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Affine3d& pose);
/**
 * Generates a cartesian point with free rotation about the Z axis of the EFF frame
 */
//descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Affine3d& pose);
descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(double x, double y, double z, double rx, double ry, double rz);

/**
 * Translates a descartes trajectory to a ROS joint trajectory
 */
trajectory_msgs::JointTrajectory
toROSJointTrajectory(const TrajectoryVec& trajectory, const descartes_core::RobotModel& model,
                     const std::vector<std::string>& joint_names, double time_delay);

/**
 * Sends a ROS trajectory to the robot controller
 */
bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory);

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "obstacle_avoidance_traj");
  ros::NodeHandle nh;

  //ros::Subscriber sub_jointmsg;
  //sub_jointmsg = nh.subscribe<sensor_msgs::JointState>("/joint_states",1,jocommCallback);
  // Required for communication with moveit components
  ros::AsyncSpinner spinner (1);
  spinner.start();

  ros::Publisher planning_scene_publish = nh.advertise<moveit_msgs::PlanningScene>("planning_scene",1);

  //collision objects
  double padding = 0.1; //add five cm (10cm/2) as security for collision objects
  moveit_msgs::AttachedCollisionObject attached_object, attached_object2;
  attached_object.link_name = "base"; attached_object2.link_name = "base";
  // The header must contain a valid TF frame
  attached_object.object.header.frame_id = "base"; attached_object2.object.header.frame_id = "base";
  // The id of the object
  attached_object.object.id = "box"; attached_object2.object.id = "second_box";
  // A default pose
  geometry_msgs::Pose pose; geometry_msgs::Pose pose_2;
  pose.orientation.w = 1.0;   pose.position.x =  0.5;    pose.position.y =  0.5;    pose.position.z =  0.1;
  obstacle_pose << pose.position.x, pose.position.y, pose.position.z;
  pose_2.orientation.w = 1.0;   pose_2.position.x =  0.5;    pose_2.position.y =  -0.2;    pose_2.position.z =  0.2;
  // Define a box to be attached
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4;    primitive.dimensions[1] = 0.4;    primitive.dimensions[2] = 0.1;
  length = primitive.dimensions[0] + padding; width = primitive.dimensions[1] + padding; height = primitive.dimensions[2] + padding;
  attached_object.object.primitives.push_back(primitive); //attached_object2.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(pose); //attached_object2.object.primitive_poses.push_back(pose_2);
  attached_object.object.operation = attached_object.object.ADD; //attached_object2.object.operation = attached_object2.object.ADD;
  ROS_INFO("Adding two objects into the world at the locations specified by poses.");
  moveit_msgs::PlanningScene planning_scene_2;
  planning_scene_2.is_diff = true;
  planning_scene_2.world.collision_objects.push_back(attached_object.object);
  planning_scene_publish.publish(planning_scene_2);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
  planning_scene::PlanningScene my_plan_scene(robot_model);
  my_plan_scene.setPlanningSceneDiffMsg(planning_scene_2);
  /*
   * moveit_msgs::PlanningSceneWorld my_collision_world;
  std::cout << "size of data of octomap is: " << my_collision_world.octomap.octomap.data.size() << std::endl;
  moveit_msgs::PlanningScene copied_scene;
  my_plan_scene.getPlanningSceneMsg(copied_scene);
  moveit_msgs::PlanningSceneWorld::_octomap_type my_octomap = planning_scene_2.world.octomap;
  std::cout << "********** my octomap name is: " << my_octomap.octomap.id << std::endl;
  */
  collision_detection::CollisionWorldConstPtr my_collision_world = my_plan_scene.getCollisionWorld();
  std::vector<std::string> my_collision_world_objects = my_collision_world->getWorld()->getObjectIds();
  std::cout << "object number is: " << my_collision_world_objects.size() << std::endl;
  for(int i = 0; i < my_collision_world_objects.size(); i++)
      std::cout << "name of object number: " << i << " is: " << my_collision_world_objects[i] << std::endl;

  // 2. Define sequence of valid points
  Eigen::Vector3d start, goal;
  start << 0.0, 0.8, 0.2;
  goal << 0.8, 0.0, 0.2;
  int count = 0; bool valid_path = false;
  while (count < 10){
      // 1. Create a robot model and initialize it
      descartes_core::RobotModelPtr model (new descartes_moveit::MoveitStateAdapter);

      // Name of description on parameter server. Typically just "robot_description".
      const std::string robot_description = "robot_description";

      // name of the kinematic group you defined when running MoveitSetupAssistant
      const std::string group_name = "left_arm";

      // Name of frame in which you are expressing poses. Typically "world_frame" or "base_link".
      const std::string world_frame = "/base";

      // tool center point frame (name of link associated with tool)
      const std::string tcp_frame = "left_gripper";
      if (!model->initialize(robot_description, group_name, world_frame, tcp_frame))
      {
        ROS_INFO("Could not initialize robot model");
        return -1;
      }

      model->setCheckCollisions(true);
      // 3. Create a planner and initialize it with our robot model
      descartes_planner::DensePlanner planner;
      //model->check_collisions_
      planner.initialize(model);

      TrajectoryVec points = RRT_cart_path_generator(start,goal);
      while (points.size() > 100){
          start_tree.open("start_tree_file.csv"); goal_tree.open("goal_tree_file.csv");
          points = RRT_cart_path_generator(start,goal);
          start_tree.close(); goal_tree.close();
      }
      std::cin.ignore();
      // 4. Feed the trajectory to the planner
      if (!planner.planPath(points)){
        ROS_ERROR("Could not solve for a valid path");
        valid_path = false;
      }
      else{
          ROS_INFO("a valid path found");
          valid_path = true;
          TrajectoryVec result;
          if (!planner.getPath(result))
          {
            ROS_ERROR("Could not retrieve path");
            return -3;
          }

          // 5. Translate the result into a type that ROS understands
          // Get Joint Names
          std::vector<std::string> names;
          nh.getParam("controller_joint_names", names);
          // Generate a ROS joint trajectory with the result path, robot model, given joint names,
          // a certain time delta between each trajectory point
          trajectory_msgs::JointTrajectory joint_solution = toROSJointTrajectory(result, *model, names, 0.05);

          // 6. Send the ROS trajectory to the robot for execution
          if (!executeTrajectory(joint_solution))
          {
            ROS_ERROR("Could not execute trajectory!");
            return -4;
          }
          ROS_INFO("Done!");
      }
      count = count + 1;
      if (valid_path) break;
  }
  return 0;
}
double angle_pi(double angle){
    //keep the angle in Pi
    if (angle > M_PI)  angle = angle - 2*M_PI;
    if (angle < -M_PI)  angle = angle + 2*M_PI;
    return angle;
}

bool extend(std::vector<Eigen::Vector4d> &Tree, Eigen::Vector3d xrand){
    double obst_x_1 = obstacle_pose(0) - length/2.0, obst_x_2 = obstacle_pose(0) + length/2.0,
            obst_y_1 = obstacle_pose(1) - width/2.0, obst_y_2 = obstacle_pose(1) + width/2.0,
            obst_z_1 = obstacle_pose(2) - height/2.0, obst_z_2 = obstacle_pose(2) + height/2.0,
            container, max_ang = 0.25, min_ang = -0.25, step_dist = 0.05, current_step_dist;
    //arrange obstacle coordinates that that x1,y1,z1 are smaller
    if (obst_x_1 > obst_x_2) {container = obst_x_1; obst_x_1 = obst_x_2; obst_x_2 = container;}
    if (obst_y_1 > obst_y_2) {container = obst_y_1; obst_y_1 = obst_y_2; obst_y_2 = container;}
    if (obst_z_1 > obst_z_2) {container = obst_z_1; obst_z_1 = obst_z_2; obst_z_2 = container;}
    Eigen::Vector4d xnear;
    double d = std::numeric_limits<double>::infinity();
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
    //check if new point is out of obstacle
    if (xnew(0) > obst_x_1 && xnew(0) < obst_x_2 && xnew(1) > obst_y_1 && xnew(1) < obst_y_2 && xnew(2) > obst_z_1 && xnew(2) < obst_z_2){
        std::cout << "this point is inside the obstacle, gonna look for another one" << std::endl;
        return false;
    }
    else{
        Tree.push_back(xnew);
        return true;
    }
}

TrajectoryVec RRT_cart_path_generator(Eigen::Vector3d start, Eigen::Vector3d goal){
   TrajectoryVec my_path;
   std::vector<Eigen::Vector4d> TreeInit, TreeGoal, swap;
   Eigen::Vector4d start_point(start(0), start(1), start(2), 0.0), goal_point(goal(0), goal(1), goal(2), M_PI);
   double X_range = fabs(start(0) - goal(0)), Y_range = fabs(start(1) - goal(1)), Z_range = fabs(start(2) - goal(2)), max_ang = 0.25, min_ang = -0.25, step_size = 0.1,
           rot_x = 0.0, rot_y = M_PI, rot_z = 0.0, Radius = 1.5;
   //initialize start tree and goal tree
   TreeInit.push_back(start_point); TreeGoal.push_back(goal_point);
   start_tree << TreeInit[TreeInit.size() - 1](0) << "," << TreeInit[TreeInit.size() - 1](1) << "," << TreeInit[TreeInit.size() - 1](2) << "\n";
   goal_tree << TreeGoal[TreeGoal.size() - 1](0) << "," << TreeGoal[TreeGoal.size() - 1](1) << "," << TreeGoal[TreeGoal.size() - 1](2) << "\n";
   //build the biderctional path from start point to goal point
   bool my_Exit = false;
   int count = 0;
   while(!my_Exit){
       bool AddInit = false, AddGoal = false;
       Eigen::Vector3d xgoal;
       srand((unsigned)time(NULL));
       //Eigen::Vector3d xrand((((double)rand()/(double)RAND_MAX) - 0.5) * 3, (((double)rand()/(double)RAND_MAX) - 0.5) * 3, (((double)rand()/(double)RAND_MAX) - 0.5) * 3);

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


       //std::cout << "random vector is: \n" << xrand << std::endl << "****************************************" << std::endl;
       std::cout << "this is loop number: " << count << std::endl << "initial tree size is:" << TreeInit.size()
                 << std::endl << "goal tree size is:" << TreeGoal.size() << std::endl << "****************************************" << std::endl;
       //extend start tree
       AddInit = extend(TreeInit,xrand);
       if(AddInit){
           start_tree << TreeInit[TreeInit.size() - 1](0) << "," << TreeInit[TreeInit.size() - 1](1) << "," << TreeInit[TreeInit.size() - 1](2) << "\n";
           //std::cout << "new point is added to start tree, we are growing ....:  " << TreeInit.size() << std::endl;
           xgoal << TreeInit[TreeInit.size() - 1](0), TreeInit[TreeInit.size() - 1](1), TreeInit[TreeInit.size() - 1](2);
           AddGoal = extend(TreeGoal,xgoal);
       }
       //check if we reach the goal
       if(AddGoal && AddInit){
           goal_tree << TreeGoal[TreeGoal.size() - 1](0) << "," << TreeGoal[TreeGoal.size() - 1](1) << "," << TreeGoal[TreeGoal.size() - 1](2) << "\n";
           //std::cout << "new point is added to goal ****************** :) tree, we are growing ....:  " << TreeGoal.size() << std::endl;
           Eigen::Vector3d qinit, qgoal;
           qinit << TreeInit[TreeInit.size() - 1](0), TreeInit[TreeInit.size() - 1](1), TreeInit[TreeInit.size() - 1](2);
           qgoal << TreeGoal[TreeGoal.size() - 1](0), TreeGoal[TreeGoal.size() - 1](1), TreeGoal[TreeGoal.size() - 1](2);
           double dist_goal_start = sqrt(pow(qgoal(0) - qinit(0),2) + pow(qgoal(1) - qinit(1),2) + pow(qgoal(2) - qinit(2),2));
           double PsiInit = TreeInit[TreeInit.size() - 1](3), PsiGoal = TreeGoal[TreeGoal.size() - 1](3), Aig = 0.0, Agi = 0.0;
           double my_angle = angle_pi(PsiInit - PsiGoal + M_PI);
           //if (dist_goal_start < step_size && Aig < max_ang && Aig > min_ang && Agi < max_ang & Agi > min_ang && my_angle < max_ang && my_angle > min_ang)
           if (dist_goal_start < 2*step_size)
               my_Exit = true;
       }
       /*swap = TreeInit;
       TreeInit = TreeGoal;
       TreeGoal = swap;*/
       count = count + 1;
       if (count > 10000){
           ROS_ERROR("************stop it is enough **********************");
           my_Exit = true;
       }
   }
   //start_tree.close(); goal_tree.close();
   //from the two trees construct the trajectoryvec
   //do the initial tree first from start to some mid point
   for (int i = 0; i < TreeInit.size(); i++){
       double x = TreeInit[i](0), y = TreeInit[i](1), z = TreeInit[i](2), rx = rot_x, ry = rot_y, rz = rot_z;
       descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(x,y,z,rx,ry,rz);
       my_path.push_back(pt);
   }
   //do the goal tree in reverse from some mid point to the goal
   for (int i = TreeGoal.size() - 1; i > -1; i--){
       double x = TreeGoal[i](0), y = TreeGoal[i](1), z = TreeGoal[i](2), rx = rot_x, ry = rot_y, rz = rot_z;
       descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(x,y,z,rx,ry,rz);
       my_path.push_back(pt);
   }
   return my_path;
}
/*
 * TrajectoryVec cart_path_generator(Eigen::Vector3d start, Eigen::Vector3d goal, collision_detection::CollisionWorldConstPtr my_collision_world){
    //extract collision objects and their position and shape
    std::vector<std::string> my_collision_world_objects = my_collision_world->getWorld()->getObjectIds();
    collision_detection::World::ObjectConstPtr my_object = my_collision_world->getWorld()->getObject(my_collision_world_objects[0]);
    EigenSTL::vector_Affine3d my_object_pose = my_object->shape_poses_;
    std::vector<shapes::ShapeConstPtr> my_shape = my_object->shapes_;
    shapes::ShapeConstPtr my_shape_ptr = my_shape[0];

    std::ostream &out = std::cout;
    my_shape_ptr->print(out);
    out.
    std::cout << "-------- shapes size is: -----------" << my_shape.size() << std::endl;
    std::cout << "-------- look at object pose: -----------\n" << my_object_pose.data()->matrix() << std::endl;


    Eigen::Vector3d distance, current_position = start, ee_twist;
    distance << goal(0) - start(0), goal(1) - start(1), goal(2) - start(2);
    double duration = 7.5*distance.norm();

    double time_elapsed = 0.0, dt = 0.02;
    while(ros::ok() && time_elapsed < duration){
        double rtdot =(30*pow(time_elapsed,2))/pow(duration,3) - (60*pow(time_elapsed,3))/pow(duration,4) + (30*pow(time_elapsed,4))/pow(duration,5);
        ee_twist << rtdot * distance(0), rtdot * distance(1), rtdot * distance(2), rtdot * my_alpha * u(0), rtdot * my_alpha * u(1), rtdot * my_alpha * u(2);
        current_position = current_position + dt * ee_twist;
        //check that current position is out of the obstacle
    }
}
*/


descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Affine3d& pose)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;

  return TrajectoryPtPtr( new CartTrajectoryPt( TolerancedFrame(pose)) );
}

//descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Affine3d& pose)
descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(double x, double y, double z, double rx, double ry, double rz)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;
  //return TrajectoryPtPtr( new AxialSymmetricPt(pose, M_PI/2.0-0.0001, AxialSymmetricPt::Z_AXIS) );
    return TrajectoryPtPtr( new AxialSymmetricPt(x,y,z,rx,ry,rz, M_PI/2.0-0.0001, AxialSymmetricPt::Z_AXIS) );
}

trajectory_msgs::JointTrajectory
toROSJointTrajectory(const TrajectoryVec& trajectory,
                     const descartes_core::RobotModel& model,
                     const std::vector<std::string>& joint_names,
                     double time_delay)
{
  // Fill out information about our trajectory
  trajectory_msgs::JointTrajectory result;
  result.header.stamp = ros::Time::now();
  result.header.frame_id = "world_frame";
  result.joint_names = joint_names;

  // For keeping track of time-so-far in the trajectory
  double time_offset = 0.0;

  // Loop through the trajectory
  for (TrajectoryIter it = trajectory.begin(); it != trajectory.end(); ++it)
  {
    // Find nominal joint solution at this point
    std::vector<double> joints;
    it->get()->getNominalJointPose(std::vector<double>(), model, joints);

    //std::cout << "joints size is: " << joints.size() << std::endl;
    // Fill out a ROS trajectory point
    trajectory_msgs::JointTrajectoryPoint pt;
    for(int i = 0; i < joints.size(); i++)
        pt.positions.push_back(joints[i]);

    // velocity, acceleration, and effort are given dummy values
    // we'll let the controller figure them out
    pt.velocities.resize(7, 0.0);
    pt.accelerations.resize(7, 0.0);
    pt.effort.resize(7, 0.0);
    /*
    for(int i = 0; i < 10; i++)
        pt.positions.push_back(0.0);
    for(int i = 0; i < joints.size(); i++)
        pt.positions.push_back(joints[i]);
    // velocity, acceleration, and effort are given dummy values
    // we'll let the controller figure them out
    pt.velocities.resize(14, 0.0);
    pt.accelerations.resize(14, 0.0);
    pt.effort.resize(14, 0.0);*/
    // set the time into the trajectory
    pt.time_from_start = ros::Duration(time_offset);
    // increment time
    time_offset += time_delay;

    result.points.push_back(pt);
  }

  return result;
}

bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory)
{
  // Create a Follow Joint Trajectory Action Client
  //actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/robot/limb/left/follow_joint_trajectory", true);
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("joint_trajectory_action", true);
  if (!ac.waitForServer(ros::Duration(2.0)))
  {
    ROS_ERROR("Could not connect to action server");
    return false;
  }

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  goal.goal_time_tolerance = ros::Duration(1.0);

  ac.sendGoal(goal);
  //std::cout << "siz is: " << goal.trajectory.points.size()-1 << std::endl;
  if (ac.waitForResult(goal.trajectory.points[goal.trajectory.points.size()-1].time_from_start + ros::Duration(5)))
  {
    ROS_INFO("Action server reported successful execution");
    return true;
  } else {
    ROS_WARN("Action server could not execute trajectory");
    return false;
  }
}
