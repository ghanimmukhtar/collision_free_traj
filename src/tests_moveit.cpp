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

//#include <moveit/ompl_interface

static const std::string ROBOT_DESCRIPTION =
    "robot_description";  // name of the robot description (a param name, so it can be changed externally)

namespace ob = ompl::base;
namespace og = ompl::geometric;

move_group::MoveGroupContextPtr context_;// context_2;
boost::shared_ptr<pluginlib::ClassLoader<move_group::MoveGroupCapability> > capability_plugin_loader_;
std::vector<boost::shared_ptr<move_group::MoveGroupCapability> > capabilities_;

ob::ValidStateSamplerPtr allocOBValidStateSampler(const ob::SpaceInformation *si)
{
    // we can perform any additional setup / configuration of a sampler here
    // but there is nothing to tweak in case of the ObstacleBasedValidStateSampler.
    return ob::ValidStateSamplerPtr(new ob::ObstacleBasedValidStateSampler(si));
}
void plan(int samplerIndex)
{
    // construct the state space we are planning in
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(3));
    og::SimpleSetup ss(space);
    // set sampler (optional; the default is uniform sampling
    if (samplerIndex==1)
        // use obstacle-based sampling
        ss.getSpaceInformation()->setValidStateSamplerAllocator(allocOBValidStateSampler);
    ss.print();
}

void configureCapabilities(ros::NodeHandle &node_handle_)
{
  try
  {
    capability_plugin_loader_.reset(
        new pluginlib::ClassLoader<move_group::MoveGroupCapability>("moveit_ros_move_group", "move_group::MoveGroupCapability"));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating plugin loader for move_group capabilities: " << ex.what());
    return;
  }

  // add individual capabilities move_group supports
  std::string capability_plugins;
  if (node_handle_.getParam("capabilities", capability_plugins))
  {
    boost::char_separator<char> sep(" ");
    boost::tokenizer<boost::char_separator<char> > tok(capability_plugins, sep);
    for (boost::tokenizer<boost::char_separator<char> >::iterator beg = tok.begin(); beg != tok.end(); ++beg)
    {
      std::string plugin = *beg;
      try
      {
        printf(MOVEIT_CONSOLE_COLOR_CYAN "Loading '%s'...\n" MOVEIT_CONSOLE_COLOR_RESET, plugin.c_str());
        move_group::MoveGroupCapability* cap = capability_plugin_loader_->createUnmanagedInstance(plugin);
        cap->setContext(context_);
        cap->initialize();
        capabilities_.push_back(boost::shared_ptr<move_group::MoveGroupCapability>(cap));
      }
      catch (pluginlib::PluginlibException& ex)
      {
        ROS_ERROR_STREAM("Exception while loading move_group capability '"
                         << plugin << "': " << ex.what() << std::endl
                         << "Available capabilities: "
                         << boost::algorithm::join(capability_plugin_loader_->getDeclaredClasses(), ", "));
      }
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, move_group::NODE_NAME);
  ros::NodeHandle nh;
  ros::Publisher planning_scene_publish = nh.advertise<moveit_msgs::PlanningScene>("planning_scene",1);


  ros::AsyncSpinner spinner(1);
  spinner.start();

  //plan(1);


  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
  planning_scene::PlanningScene m_planning_scene(robot_model);
  //m_planning_scene.enable_shared_from_this;
  planning_scene::PlanningSceneConstPtr scene_ptr(new planning_scene::PlanningScene(robot_model));
  /*planning_scene_monitor::PlanningSceneMonitorPtr plan_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
  plan_scene_monitor->startSceneMonitor();
  plan_scene_monitor->startWorldGeometryMonitor();
  plan_scene_monitor->startStateMonitor();
  context_.reset(new move_group::MoveGroupContext(plan_scene_monitor, false, false));
  planning_pipeline::PlanningPipelinePtr pipeline_ptr = context_->planning_pipeline_;
  std::vector<std::string> adapters = pipeline_ptr->getAdapterPluginNames();
  //for(int i = 0; i < adapters.size(); ++i)
     // ROS_INFO_STREAM("adapter: " << i << " is: " << adapters[i]);
  ROS_INFO_STREAM("planner plugin name: " << pipeline_ptr->getPlannerPluginName());
  planning_interface::PlannerManagerPtr planner_manager = pipeline_ptr->getPlannerManager();
  planner_manager->initialize(robot_model, nh.getNamespace());
  std::vector<std::string> algs;
  planner_manager->getPlanningAlgorithms(algs);
  for(int i = 0; i < algs.size(); ++i)
        ROS_INFO_STREAM("algorithm: " << i << " is: " << algs[i]);*/

  //scene_ptr.reset;
  //scene_ptr->clone()
  robot_state::RobotState current_state = m_planning_scene.getCurrentState();
  //current_state.setToRandomPositions();
  /*boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
  planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
                                  "moveit_core", "planning_interface::PlannerManager"));
  planning_interface::PlannerManagerPtr planning_instance;
  planning_instance.reset(planner_plugin_loader->createUnmanagedInstance("ompl_interface/OMPLPlanner"));

  planning_instance->initialize(robot_model, nh.getNamespace());
  ROS_INFO_STREAM("Using planning interface '" << planning_instance->getDescription() << "'");
  planning_interface::PlannerConfigurationMap second_map = planning_instance->getPlannerConfigurations();
  for(std::map<std::string, planning_interface::PlannerConfigurationSettings>::iterator itr = second_map.begin(); itr != second_map.end(); ++itr){
      for(std::map<std::string, std::string>::iterator inner_iter = (*itr).second.config.begin(); inner_iter != (*itr).second.config.end(); ++inner_iter)
          ROS_INFO_STREAM((*inner_iter).first << " is: " << (*inner_iter).second);
      ROS_INFO_STREAM((*itr).first << " is: " << (*itr).second.name);
      ROS_INFO("**********************************************************");
  }
  planning_interface::MotionPlanRequest my_request;
  my_request.group_name = "both_arms";

  planning_interface::PlanningContextPtr second_context = planning_instance->getPlanningContext(scene_ptr, my_request);*/
  //second_context->solve();
  ompl_interface::OMPLInterface oi(robot_model, nh);

  configureCapabilities(nh);

  /*oi.getConstraintsLibrary();
  oi.loadConstraintApproximations();
  oi.printStatus();*/
  ompl_interface::ConstraintsLibrary const_lib = oi.getConstraintsLibrary();
  ROS_INFO_STREAM("using constraints: " << oi.isUsingConstraintsApproximations());
  planning_interface::PlannerConfigurationMap my_map = oi.getPlannerConfigurations();
  ompl_interface::PlanningContextManager my_planning_context_manager = oi.getPlanningContextManager();
  my_planning_context_manager.setPlannerConfigurations(my_map);
  oi.setPlannerConfigurations(my_map);


  std::map<std::string, ompl_interface::ConfiguredPlannerAllocator> aloc = my_planning_context_manager.getRegisteredPlannerAllocators();
  //ROS_INFO_STREAM(aloc.size());
  //for(std::map<std::string, ompl_interface::ConfiguredPlannerAllocator>::iterator itr = aloc.begin(); itr != aloc.end(); ++itr)
    //  ROS_INFO_STREAM((*itr).first);

  for(std::map<std::string, planning_interface::PlannerConfigurationSettings>::iterator itr = my_map.begin(); itr != my_map.end(); ++itr){
      for(std::map<std::string, std::string>::iterator inner_iter = (*itr).second.config.begin(); inner_iter != (*itr).second.config.end(); ++inner_iter)
          ROS_INFO_STREAM((*inner_iter).first << " is: " << (*inner_iter).second);
      ROS_INFO_STREAM((*itr).first << " is: " << (*itr).second.name);
      ROS_INFO("**********************************************************");
  }

  std::map<std::string, ompl_interface::ModelBasedStateSpaceFactoryPtr> my_factories = my_planning_context_manager.getRegisteredStateSpaceFactories();
  for(std::map<std::string, ompl_interface::ModelBasedStateSpaceFactoryPtr>::iterator itr = my_factories.begin(); itr != my_factories.end(); ++itr)
        ROS_INFO_STREAM((*itr).first);
  ompl_interface::ModelBasedStateSpaceFactoryPtr my_state_space_factory = (*my_factories.begin()).second;
  ROS_INFO_STREAM(my_state_space_factory->getType());
  moveit::core::JointModelGroup *jmp = robot_model->getJointModelGroup("both_arms");
  std::vector<std::string> links = jmp->getJointModelNames();
  //for(int i = 0; i < links.size(); ++i)
     // ROS_INFO_STREAM(links[i]);

  ompl_interface::ModelBasedPlanningContextPtr context_ptr = my_planning_context_manager.getPlanningContext("both_arms", "JointModel");
  context_ptr->configure();
  ROS_INFO_STREAM(context_ptr->getSpecification().state_space_->getJointModelGroupName());
  ompl_interface::ModelBasedStateSpaceSpecification space_spec(robot_model, "both_arms");
  space_spec.joint_bounds_ = context_ptr->getSpecification().state_space_->getJointsBounds();
  robot_model::JointBoundsVector my_bounds = space_spec.joint_bounds_;

  ROS_INFO_STREAM(space_spec.joint_model_group_->getJointModelNames().size());
  ROS_INFO_STREAM(my_bounds.size());
  ROS_INFO_STREAM(context_ptr->getSpecification().state_space_->getJointsBounds().size());
  ompl::geometric::SimpleSetup my_setup((*my_factories.begin()).second->getNewStateSpace(space_spec));
  ompl::geometric::SimpleSetupPtr second_setup = context_ptr->getOMPLSimpleSetup();

  my_setup.getSpaceInformation()->setValidStateSamplerAllocator(allocOBValidStateSampler);
  /*ompl::base::SpaceInformationPtr space_info_ptr = my_setup.getSpaceInformation();
  ompl::base::StateSpacePtr my_space = my_setup.getStateSpace();
  ompl::base::SpaceInformation si(my_space);
  ompl::base::SpaceInformation *pointer_si = &si;*/
  //ompl::base::SpaceInformationPtr space_info_ptr = second_setup->getSpaceInformation();
    ompl::base::StateSpacePtr my_space = second_setup->getStateSpace();
    ompl::base::SpaceInformation si(my_space);
    ompl::base::SpaceInformation *pointer_si = &si;
  ompl::base::UniformValidStateSampler my_uniform_sampler(pointer_si);


  constraint_samplers::JointConstraintSampler const_sampler(scene_ptr, "left_arm");
  //const_sampler.configure()
  //my_setup.print();
  //ompl::base::PlannerPtr my_planner_ptr = my_setup.getPlanner();
  ompl::base::StateSpacePtr state_space_ptr = second_setup->getStateSpace();
  ROS_INFO_STREAM(state_space_ptr->getName());
  ompl::base::State *test_state = state_space_ptr->allocState();
  ompl::base::State *test_state2 = my_space->allocState();
  robot_state::RobotState test_robot_state(robot_model);
  state_space_ptr->setup();
  ompl::base::StateSamplerPtr my_sampler = state_space_ptr->allocStateSampler();

  std::filebuf fb;
  fb.open ("test.txt",std::ios::out);
  std::ostream os(&fb);
  os << "Test sentence\n";
  for(int i = 0; i < 5; ++i){
      ROS_INFO_STREAM(i << i << i << i << i);
      my_sampler->sampleUniform(test_state);
      context_ptr->getOMPLStateSpace()->printState(test_state, os);
  }
  ROS_INFO_STREAM(my_uniform_sampler.getName());
  ROS_INFO_STREAM(my_uniform_sampler.getNrAttempts());
  my_uniform_sampler.sample(test_state2);
  fb.close();


  //state_space_ptr->StateSampler.sampleUniform(test_state);
  //ompl::base::PlannerPtr my_planner_ptr(new ompl::base::Planner(my_setup.getSpaceInformation(), "crustcrawler_arm[RRTstarkConfigDefault]"));
  //my_setup.setPlanner(my_planner_ptr);
  //ROS_INFO_STREAM(my_planner_ptr->getName());
  /*planning_interface::MotionPlanRequest my_request;
  my_request.group_name = "crustcrawler_arm";
  ompl_interface::ModelBasedPlanningContextPtr context2(oi.getPlanningContext(scene_ptr, my_request));
  ompl_interface::ModelBasedPlanningContextPtr context(oi.getPlanningContext("crustcrawler_arm[RRTstarkConfigDefault]"));

  context->setCompleteInitialState(current_state);
  context->setPlanningScene(scene_ptr);

  context->configure();
  context->getOMPLSimpleSetup()->setup();
  /*ROS_INFO("******************************************");
  std::map<std::string, std::string> specs = context->getSpecificationConfig();
  for(std::map<std::string, std::string>::iterator itr = specs.begin(); itr != specs.end(); ++itr){
      ROS_INFO_STREAM((*itr).first << " is: " << (*itr).second);
      ROS_INFO("************************************");
  }

  ROS_INFO_STREAM(context->getOMPLSimpleSetup()->getPlanner()->getName());
  ompl_interface::ModelBasedStateSpacePtr mbss = context->getOMPLStateSpace();


  //ompl::base::StateSamplerPtr sampler = mbss->allocDefaultStateSampler();

  ompl::base::State *test_state;

  ompl_interface::og::SimpleSetupPtr og_ptr(context->getOMPLSimpleSetup());
  ompl::base::SpaceInformationPtr my_space_info = og_ptr->getSpaceInformation();
  ompl::geometric::SimpleSetup my_simple_setup(context->getOMPLSimpleSetup()->getSpaceInformation());
  my_simple_setup.setPlanner(og_ptr->getPlanner());

  my_simple_setup.setup();
  my_simple_setup.print();
  my_simple_setup.getSpaceInformation()->setValidStateSamplerAllocator(allocOBValidStateSampler);
  ROS_INFO_STREAM(my_simple_setup.getPlanner()->getName());
  //my_space_info->printSettings();
  ompl::base::ValidStateSamplerPtr sampler = my_simple_setup.getSpaceInformation()->allocValidStateSampler();

  //ompl::base::ValidStateSamplerPtr sptr(new ompl::base::ObstacleBasedValidStateSampler(my_space_info));
  //og_ptr->print();
  ROS_INFO_STREAM(sampler->getName());
  ROS_INFO("***********************************************");
  //og_ptr->getSpaceInformation()->setValidStateSamplerAllocator(allocOBValidStateSampler);
  //sampler = og_ptr->getSpaceInformation()->allocValidStateSampler();
  //og_ptr->setup();
  //og_ptr->print();
  //ROS_INFO_STREAM(sampler->getName());
  ROS_INFO_STREAM("no of trial is: " << sampler->getNrAttempts());

  std::filebuf fb;
  fb.open ("test.txt",std::ios::out);
  std::ostream os(&fb);
  os << "Test sentence\n";
  for(int i = 0; i < 5; ++i){
      ROS_INFO_STREAM(i << i << i << i << i);

      sampler->sample(test_state);
      mbss->printState(test_state, os);
  }
  fb.close();

  //sampler->sampleUniform(test_state);

  //std::vector<double> output;
  //test_state->as<double >();
  //ompl_interface::ModelBasedStateSpace::StateType* value;// = test_state->as<ompl_interface::ModelBasedStateSpace::StateType>();
  //mbss->StateType  = test_state->as<ompl_interface::ModelBasedStateSpace::StateType>();
  //std::cout << "hello: ****************> " << mbss->StateType.values << std::endl;



  og_ptr->setup();
  const ompl::base::PlannerPtr my_planner = og_ptr->getPlanner();

  ROS_INFO_STREAM(my_planner->getName());
  const ompl::base::ProblemDefinitionPtr pro_ptr = og_ptr->getProblemDefinition();
  //pro_ptr->print();
  //ompl::tools::SelfConfig config(si);*/

  ros::waitForShutdown();
  return 0;
}
