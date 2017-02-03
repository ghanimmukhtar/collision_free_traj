#include <fstream>
#include <istream>
#include <moveit/ompl_interface/ompl_interface.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/PlannerDataGraph.h>
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <moveit/ompl_interface/detail/state_validity_checker.h>
#include <ompl/tools/thunder/SPARSdb.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <ompl/base/PlannerDataStorage.h>
#include <ompl/tools/thunder/Thunder.h>
#include <ompl/tools/lightning/Lightning.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ot = ompl::tools;

int number_of_vertices;
bool isStateValid(const ob::State *state, robot_model::RobotModelPtr& robot_model, ompl_interface::ModelBasedPlanningContextPtr& context_ptr)
{
    planning_scene::PlanningScene my_scene(robot_model);
    collision_detection::CollisionRequest req;
    req.contacts = true;
    collision_detection::CollisionResult res;
    collision_detection::AllowedCollisionMatrix acm = my_scene.getAllowedCollisionMatrix();
    robot_state::RobotState robot_state_holder(robot_model);
    context_ptr->getOMPLStateSpace()->copyToRobotState(robot_state_holder, state);
    my_scene.checkSelfCollision(req, res, robot_state_holder, acm);
    /*if(res.collision)
        ROS_ERROR("I am in self collision");
    else
        ROS_WARN("nooooo Collision hurray");*/
    return res.collision;
}

bool terminate_condition(const std::shared_ptr<og::PRM>& my_prm){
    ob::PlannerData my_data(my_prm->getSpaceInformation());
    my_prm->getPlannerData(my_data);
    //if(my_data.numVertices()%100 == 0)
      //  ROS_INFO_STREAM("Number of vertices is: " << my_data.numVertices() );
    return my_data.numVertices() > number_of_vertices;
    //return false;
}

/*bool terminate_condition(const ompl::tools::SPARSdbPtr& my_spars){
    ompl::base::PlannerData my_data(my_spars->getSpaceInformation());
    my_spars->getPlannerData(my_data);
    //if(my_data.numVertices()%100 == 0)
    ROS_INFO_STREAM("Number of vertices is: " << my_data.numVertices() );
    return my_data.numVertices() > number_of_vertices;
    //return false;
}*/

int main(int argc, char** argv)
{
  ros::init(argc, argv, "generate_roadmap");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  nh.getParam("number_of_vertices", number_of_vertices);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
  ompl_interface::OMPLInterface oi(robot_model, nh);
  std::string arm_choice = "left_arm";
  oi.getPlanningContext(arm_choice, "JointModel")->getOMPLSimpleSetup()->setStateValidityChecker(std::bind(&isStateValid, std::placeholders::_1,
                                                                                                            robot_model,
                                                                                                            oi.getPlanningContextManager().getPlanningContext(arm_choice, "JointModel")));
  //oi.getPlanningContext(arm_choice, "JointModel")->getOMPLSimpleSetup()->getStateSpace()->setup();
  //oi.getPlanningContext(arm_choice, "JointModel")->getOMPLSimpleSetup()->getSpaceInformation()->setup();
  og::SimpleSetup my_setup(oi.getPlanningContext(arm_choice, "JointModel")->getOMPLStateSpace());
  my_setup.setStateValidityChecker(std::bind(&isStateValid, std::placeholders::_1,
                                             robot_model,
                                             oi.getPlanningContextManager().getPlanningContext(arm_choice, "JointModel")));
  ob::ProblemDefinitionPtr pdf(new ob::ProblemDefinition(my_setup.getSpaceInformation()));

  robot_state::RobotState robot_state_holder(robot_model);
  robot_state_holder.setToRandomPositions();
  ob::State* start_state = oi.getPlanningContext(arm_choice, "JointModel")->getOMPLStateSpace()->allocState();
  oi.getPlanningContext(arm_choice, "JointModel")->getOMPLStateSpace()->copyToOMPLState(start_state, robot_state_holder);
  pdf->addStartState(start_state);

  robot_state_holder.setToRandomPositions();
  ob::State* goal_state = oi.getPlanningContext(arm_choice, "JointModel")->getOMPLStateSpace()->allocState();
  oi.getPlanningContext(arm_choice, "JointModel")->getOMPLStateSpace()->copyToOMPLState(goal_state, robot_state_holder);
  pdf->setGoalState(goal_state);
  //std::shared_ptr<ompl::geometric::SPARSdb> my_prm(new ompl::geometric::SPARSdb(my_setup.getSpaceInformation()));
  std::shared_ptr<og::PRM> my_prm(new og::PRM(my_setup.getSpaceInformation()));
  my_prm->setProblemDefinition(pdf);
  my_prm->setup();

  //boost::shared_ptr<ompl::geometric::PRM> my_planner(new ompl::geometric::PRM(oi.getPlanningContext(arm_choice, "JointModel")->getOMPLSimpleSetup()->getSpaceInformation()));
  my_setup.setPlanner(my_prm);

  ob::PlannerTerminationCondition ptc(std::bind(&terminate_condition, my_prm));
  //ot::SPARSdbPtr my_spars;
  //my_spars.reset(new og::SPARSdb(my_setup.getSpaceInformation()));
  //ob::PlannerTerminationCondition ptc(std::bind(&terminate_condition, my_spars));
  //my_prm->growRoadmap(ptc);
  //ompl::geometric::SPARSdb* my_spars = my_prm->as<ompl::geometric::SPARSdb>();

  my_prm->growRoadmap(ptc);
  //my_spars->setProblemDefinition(pdf);
  ob::PlannerData my_data(my_setup.getSpaceInformation());
  ob::PlannerData my_data2(my_setup.getSpaceInformation());
  //my_prm->getPlannerData(my_data);
  //my_spars->getPlannerData(my_data2);

  //my_spars->checkValidity();
  //my_setup.setPlanner(my_spars);
  //my_setup.getSolutionPath();
  //my_setup.solve(10.0);
  //my_spars->solve(ptc);

  my_setup.getPlannerData(my_data2);
  //my_spars->getPlannerData();
  //my_spars->setPlannerData(my_data2);
  //my_prm->getPlannerData(my_data);
  //my_spars->getPlannerData(my_data);
  ROS_ERROR_STREAM("number of vertices in spars: " << my_data2.numVertices());
  //my_setup.getPlannerData(my_data);
  /*std::map<std::string, std::string> planner_data_properties = my_data.properties;
  std::map<std::string, std::string>::iterator itr;
  for(itr = planner_data_properties.begin(); itr != planner_data_properties.end(); ++itr)
        ROS_INFO_STREAM("plannar property: " << (*itr).first << " is: " << (*itr).second);*/

  std::filebuf fb2;
  fb2.open ("test6.db", std::ios::out);
  std::ostream os2(&fb2);
  ob::PlannerDataStorage my_storage;
  my_storage.store(my_data2, os2);
  fb2.close();
  ROS_ERROR_STREAM("number of vertices: " << my_data2.numVertices());
  return 0;
}
