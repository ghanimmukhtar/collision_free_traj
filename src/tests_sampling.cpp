#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <tf/transform_listener.h>
#include <moveit/move_group/move_group_capability.h>
#include <boost/algorithm/string/join.hpp>
#include <boost/tokenizer.hpp>
#include <moveit/macros/console_colors.h>
#include <moveit/move_group/node_name.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <moveit/ompl_interface/planning_context_manager.h>
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/Goal.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSampler.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <moveit/move_group/move_group_context.h>
#include <moveit/move_group/move_group_capability.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/constraint_samplers/default_constraint_samplers.h>
#include <ompl/geometric/planners/prm/PRM.h>

//#include <moveit/ompl_interface

static const std::string ROBOT_DESCRIPTION =
    "robot_description";  // name of the robot description (a param name, so it can be changed externally)

namespace ob = ompl::base;

ob::ValidStateSamplerPtr allocOBValidStateSampler(const ob::SpaceInformation *si)
{
    // we can perform any additional setup / configuration of a sampler here
    // but there is nothing to tweak in case of the ObstacleBasedValidStateSampler.
    return ob::ValidStateSamplerPtr(new ob::ObstacleBasedValidStateSampler(si));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, move_group::NODE_NAME);
  ros::NodeHandle nh;
  ros::Publisher planning_scene_publish = nh.advertise<moveit_msgs::PlanningScene>("planning_scene",1);

  ros::AsyncSpinner spinner(1);
  spinner.start();


  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
  ompl_interface::OMPLInterface oi(robot_model, nh);

  planning_interface::PlannerConfigurationMap my_map = oi.getPlannerConfigurations();
  ompl_interface::PlanningContextManager my_planning_context_manager = oi.getPlanningContextManager();
  my_planning_context_manager.setPlannerConfigurations(my_map);
  oi.setPlannerConfigurations(my_map);

  std::map<std::string, ompl_interface::ModelBasedStateSpaceFactoryPtr> my_factories = my_planning_context_manager.getRegisteredStateSpaceFactories();
  ompl_interface::ModelBasedPlanningContextPtr context_ptr = my_planning_context_manager.getPlanningContext("both_arms", "JointModel");
  context_ptr->configure();
  ompl_interface::ModelBasedStateSpaceSpecification space_spec(robot_model, "both_arms");
  ompl::geometric::SimpleSetup my_setup((*my_factories.begin()).second->getNewStateSpace(space_spec));
  boost::shared_ptr<ompl::geometric::SimpleSetup> second_setup = context_ptr->getOMPLSimpleSetup();

  //my_setup.getSpaceInformation()->setValidStateSamplerAllocator(allocOBValidStateSampler);

  //get pointer to space information for setup deduced from the context

  //ompl::base::StateSpacePtr my_space = second_setup->getStateSpace();
  //ompl::base::SpaceInformation si(my_space);
  //ompl::base::SpaceInformation *pointer_si = &si;

  //ompl::base::ObstacleBasedValidStateSampler my_uniform_sampler(pointer_si);
  //ompl::base::StateSamplerPtr my_uniform_sampler = my_setup.getStateSpace()->allocStateSampler();
  //my_uniform_sampler->setNrAttempts(2);
  //working statespace
  const boost::shared_ptr<ompl::base::StateSpace> state_space_ptr = second_setup->getStateSpace();

  const ompl_interface::ModelBasedStateSpacePtr model_based_state_space = context_ptr->getOMPLStateSpace();


  ompl::base::State *test_state = state_space_ptr->allocState();
  ompl::base::State *test_state2 = state_space_ptr->allocState();
  //state_space_ptr->setup();

  //working sampler
  ompl::base::StateSamplerPtr my_sampler = state_space_ptr->allocStateSampler();

  boost::shared_ptr<kinematic_constraints::KinematicConstraintSet> kin_set(new kinematic_constraints::KinematicConstraintSet(robot_model));
  ompl_interface::ModelBasedPlanningContext my_context(context_ptr->getName(), context_ptr->getSpecification());
  ompl_interface::ModelBasedPlanningContext *context_pointer = &my_context;
  ompl_interface::ValidConstrainedSampler kin_con_sampler(context_pointer, kin_set);

  planning_scene::PlanningScene my_scene(robot_model);
  collision_detection::CollisionRequest req;
  req.contacts = true;
  collision_detection::CollisionResult res;
  collision_detection::AllowedCollisionMatrix acm = my_scene.getAllowedCollisionMatrix();

  /*std::filebuf fb;
  fb.open ("test.txt",std::ios::out);
  std::ostream os(&fb);
  acm.print(os);*/

  robot_state::RobotState robot_state_holder(robot_model);
  //ompl::geometric::PRM my_bit_star(second_setup->getSpaceInformation());

  std::filebuf fb;
  fb.open ("test.txt",std::ios::out);
  std::ostream os(&fb);
  os << "Test sentence\n";
  for(int i = 0; i < 100; ++i){

      res.clear();
    //  ROS_INFO_STREAM(my_sampler->getName());
      while(!res.collision){
          my_sampler->sampleUniform(test_state);
          //kin_con_sampler.sample(test_state);
          //ROS_INFO_STREAM(kin_con_sampler.getName());
          //ROS_INFO_STREAM("bounds satisfaction is: " << state_space_ptr->satisfiesBounds(test_state));
          model_based_state_space->copyToRobotState(robot_state_holder, test_state);

          //my_scene.setCurrentState(robot_state_holder);

          my_scene.checkSelfCollision(req, res, robot_state_holder, acm);
          collision_detection::CollisionResult::ContactMap::const_iterator it;
          for(it = res.contacts.begin();
              it != res.contacts.end();
              ++it)
          {
            ROS_INFO("Contact between: %s and %s",
                     it->first.first.c_str(),
                     it->first.second.c_str());
          }
      }
      ROS_ERROR("no collision !!!!!!!!! ");
      context_ptr->getOMPLStateSpace()->printState(test_state, os);
  }
  ROS_INFO("Finished!!!!!!!!!!!!!!!!");
  //ROS_INFO_STREAM(my_uniform_sampler->getName());


  //my_uniform_sampler->sampleUniform(test_state2);
  fb.close();

  ros::waitForShutdown();
  return 0;
}
