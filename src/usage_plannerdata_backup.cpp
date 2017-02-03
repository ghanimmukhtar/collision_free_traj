/*#include <iostream>
#include <string>
#include <fstream>
//#include <ros/ros.h>
#include <ompl/base/PlannerDataGraph.h>
#include <boost/graph/graphml.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/ompl_interface/ompl_interface.h>
#include <ompl/base/PlannerDataStorage.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/tools/thunder/Thunder.h>
#include <ompl/tools/experience/ExperienceSetup.h>
#include <ompl/tools/thunder/SPARSdb.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "/usr/local/include/ompl/tools/experience/ExperienceSetup.h"
#include "/usr/local/include/ompl/tools/thunder/Thunder.h"
#include "/usr/local/include/ompl/base/spaces/RealVectorStateSpace.h"
*/
#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <ompl/base/PlannerDataStorage.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/tools/thunder/SPARSdb.h>
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/tools/thunder/Thunder.h>
#include <ompl/tools/lightning/Lightning.h>
#include <ompl/util/PPM.h>

#include <ompl/config.h>
#include "../include/collision_free_traj/config.h"

#include <boost/filesystem.hpp>
#include <iostream>

/*class MY_PRM : public ompl::geometric::PRM {
public :
    MY_PRM(const ompl::base::SpaceInformationPtr &si, bool starStrategy = true) :
        ompl::geometric::PRM(si, starStrategy)
    {

    }

    void set_graph(ompl::geometric::PRM& prm_planner){
        g_ = prm_planner.getRoadmap();
    }
};*/

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ot = ompl::tools;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "retrieve_planner_data");
    ros::NodeHandle nh;

    //ros::nodehandle n2;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    ompl_interface::OMPLInterface oi(robot_model, nh);
    planning_interface::PlannerConfigurationMap my_map = oi.getPlannerConfigurations();
    oi.setPlannerConfigurations(my_map);
    std::string arm_choice = "left_arm";
    //ob::StateSpacePtr space = to_std_ptr(oi.getPlanningContext(arm_choice, "JointModel")->getOMPLStateSpace());
    ob::StateSpacePtr space = oi.getPlanningContext(arm_choice, "JointModel")->getOMPLStateSpace();
    ot::ThunderPtr my_thunder(new ot::Thunder(space));
    my_thunder->setFilePath("test5.db");
    my_thunder->save();

    std::vector<ob::PlannerDataPtr> planner_datas;
    ob::PlannerData my_data(my_thunder->getSpaceInformation());
    my_thunder->getExperienceDB()->getSPARSdb()->getPlannerData(my_data);
    ROS_ERROR_STREAM("number of vertices in my storage: " << my_data.numVertices());
    /*my_thunder->getAllPlannerDatas(planner_datas);
    for(size_t i = 0; i < planner_datas.size(); ++i){
        ROS_ERROR_STREAM("number of vertices in my storage: " << planner_datas[i]->numVertices());
    }*/

    /*
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    ompl_interface::OMPLInterface oi(robot_model, nh);
    planning_interface::PlannerConfigurationMap my_map = oi.getPlannerConfigurations();
    oi.setPlannerConfigurations(my_map);
    std::string arm_choice = "left_arm";

    //ompl_interface::ModelBasedPlanningContextPtr model_base_state_space = oi.getPlanningContext(arm_choice, "JointModel")->getOMPLStateSpace();
    //ompl::base::StateSpacePtr state_space = model_base_state_space;
    ompl::geometric::SimpleSetupPtr my_setup = oi.getPlanningContext(arm_choice, "JointModel")->getOMPLSimpleSetup();

    //ompl::geometric::SimpleSetup my_setup(oi.getPlanningContext(arm_choice, "JointModel")->getOMPLStateSpace());
    const ompl::base::SpaceInformationPtr si_ = my_setup->getSpaceInformation();
    ompl::tools::ExperienceSetupPtr expPlanner_;
    ompl::base::StateSpacePtr my_space(new ompl::base::RealVectorStateSpace());
    ompl::tools::ThunderDBPtr my_thunder_db_ptr(new ompl::tools::ThunderDB(my_setup->getStateSpace()));
    ompl::tools::SPARSdbPtr my_spars;
    my_spars.reset(new ompl::geometric::SPARSdb(si_));
    my_thunder_db_ptr->setSPARSdb(my_spars);
    //my_thunder_db_ptr->load("/home/ghanim/.ros/test4.xml");
    //ompl::base::RealVectorStateSpace *space = new ompl::base::RealVectorStateSpace();
    //expPlanner_.reset(new ompl::tools::Thunder(ompl::base::StateSpacePtr(space)));

    //= my_thunder_db_ptr->getSPARSdb();
    ompl::tools::SPARSdbPtr spars;
    spars.reset(new ompl::geometric::SPARSdb(si_));
    //my_planner.
    //ROS_ERROR_STREAM("number of vertices before loading planner data: " << my_spars->getNumVertices());

    ompl::base::PlannerData my_data(si_);
    ROS_ERROR_STREAM("number of vertices before loading planner data: " << my_data.numVertices());
    ompl::base::PlannerData my_data3(si_);
    ompl::base::PlannerDataStorage my_storage;
    std::string fn = "test5.db";
    const std::string fn2 = "test5.db";
    std::ifstream is(fn.c_str(), std::ios::binary);
    if (!is.is_open())
    {
        std::cout << "loading file '" << fn << "'failed." << std::endl;
        throw "Could not load file.";
    }
    my_thunder_db_ptr->load(fn2);
    my_storage.load(is, my_data);
    spars->setPlannerData(my_data);

    //ROS_INFO_STREAM("loaded");
    ROS_ERROR_STREAM("number of vertices after loading planner data: " << my_thunder_db_ptr->getSPARSdb()->getNumVertices());
    ROS_ERROR_STREAM("number of vertices in my storage: " << my_data.numVertices());
    ROS_ERROR_STREAM("number of vertices in my spars: " << spars->getNumVertices());
    
    //my_planner.setPlannerData(my_data);

    //my_planner.getPlannerData(my_data3);
    //ROS_ERROR_STREAM("number of vertices after: " << my_planner.getNumVertices());
    */
    /*try to define a goal and then generate a plan using the saved roadmap only
    //ompl::geometric::SimpleSetup my_setup(my_data.getSpaceInformation());

    //my_setup.getPlannerData(my_data);

    //boost::shared_ptr<MY_PRM> my_prm_planner(new MY_PRM(my_data.getSpaceInformation()));


    my_planner->getSpaceInformation()->setup();
    my_planner->setup();

    my_setup.setPlanner(my_planner);
    //ompl::base::PlannerData my_data2(oi.getPlanningContext(arm_choice, "JointModel")->getOMPLSimpleSetup()->getSpaceInformation());
    */

    /*
    //ROS_ERROR_STREAM("number of vertices in first my_data: " << my_data2.numVertices());
    ompl::base::State *st = my_setup.getStateSpace()->allocState();
    ompl::base::State *abs_start_state = my_setup.getStateSpace()->allocState();
    robot_state::RobotState start_robot_state(robot_model);
    start_robot_state.setToRandomPositions();
    ompl::base::ScopedState<> start_state(my_setup.getStateSpace());
    oi.getPlanningContext(arm_choice, "JointModel")->getOMPLStateSpace()->copyToOMPLState(abs_start_state, start_robot_state);
    start_state = abs_start_state;
    my_setup.setStartState(start_state);
    //std::vector<double> robot_goal = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    std::vector<double> robot_goal = {0, 0, 0, 0, 0, 0, 0};
    //goal_robot_state.setVariablePositions(goal_robot_state.getVariableNames(), robot_goal);
    my_setup.getStateSpace()->copyFromReals(st, robot_goal);
    //const ompl::base::ScopedState<> goal_state = *st->as<ompl::base::ScopedState::StateType>();
    ompl::base::ScopedState<> goal_state(my_setup.getStateSpace());
    goal_state = st;
    my_setup.setGoalState(goal_state);
    //my_prm_planner->declareParam;
    my_setup.solve();
    ompl::geometric::PathGeometric& my_sol = my_setup.getSolutionPath();
    my_sol.print(std::cout);
    */

    ROS_INFO_STREAM("finished!!!!!");
    return 0;
}
