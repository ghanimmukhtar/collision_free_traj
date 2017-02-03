#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <ompl/geometric/planners/prm/PRM.h>
#include <boost/graph/adjacency_list.hpp>
#include <ompl/base/PlannerDataGraph.h>
#include <boost/graph/graphml.hpp>
#include <boost/property_map/function_property_map.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/ompl_interface/ompl_interface.h>
#include <ros/ros.h>
#include <boost/graph/named_graph.hpp>
#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>


using namespace boost;
typedef adjacency_list<vecS, vecS, directedS,property<vertex_name_t, std::string, property<vertex_type_t, Eigen::VectorXd> >, property<edge_weight_t,double> > BoostGraphType;
typedef boost::adjacency_list <
    boost::vecS, boost::vecS, boost::undirectedS,
    boost::property < ompl::geometric::PRM::vertex_state_t, ompl::base::State*,
    boost::property < ompl::geometric::PRM::vertex_total_connection_attempts_t, unsigned long int,
    boost::property < ompl::geometric::PRM::vertex_successful_connection_attempts_t, unsigned long int,
    boost::property < boost::vertex_predecessor_t, unsigned long int,
    boost::property < boost::vertex_rank_t, unsigned long int > > > > >,
    boost::property < boost::edge_weight_t, ompl::base::Cost >
> Graph;
typedef dynamic_properties BoostDynamicProperties;
// Property map for extracting states as arrays of doubles
std::vector<double> vertexCoords (std::string& str)
{
    std::vector<double> joints_angles;
    std::stringstream ss(str);
    double i;
    int count = 0;
    while(ss >> i){
        joints_angles.push_back(i);
        count+=1;
        if(ss.peek() == ',')
            ss.ignore();
    }
    return joints_angles;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_our_prm");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    ompl_interface::OMPLInterface oi(robot_model, nh);
    planning_interface::PlannerConfigurationMap my_map = oi.getPlannerConfigurations();
    oi.setPlannerConfigurations(my_map);
    std::string fn = "test2.xml";
    std::ifstream is(fn.c_str());
    std::string fn2 = "test5.xml";
    std::ofstream is2(fn2.c_str());
    if (!is.is_open())
    {
        std::cout << "loading file '" << fn << "'failed." << std::endl;
        throw "Could not load file.";
    }
    //void* graphRw;

    void* graphRaw2_;
    graphRaw2_ = new ompl::geometric::PRM::Graph;
    ompl::geometric::PRM::Graph* graph2_ = reinterpret_cast<ompl::geometric::PRM::Graph*>(graphRaw2_);

    void* graphRaw_;
    graphRaw_ = new ompl::base::PlannerData::Graph;
    ompl::base::PlannerData::Graph* graph_ = reinterpret_cast<ompl::base::PlannerData::Graph*>(graphRaw_);

    //PlannerDataGraph g;
    BoostGraphType g;

    Graph gg;
    //BoostDynamicProperties dp ;
    dynamic_properties dp;
    std::string vn = "coords";
    const std::string wg = "weight";

    dp.property(vn, get(vertex_name, g));
    dp.property(wg, get(edge_weight, g));
    read_graphml(is, g, dp);

    ROS_INFO_STREAM("edges number is: " << boost::num_edges(g));
    property_map<BoostGraphType, vertex_type_t>::type
      city_name = get(vertex_type, g);

    robot_state::RobotState robot_state_holder(robot_model);
    //property_map<PlannerDataGraph, vertex_index_t>::type test_type = get(vertex_index, *graph_);
    //vertices(g).first;
    for (auto vp = vertices(g); vp.first != vp.second; ++vp.first){
        //boost::add_vertex(get(vertex_property_type<ompl::base::PlannerDataVertex*>(),g,*vp.first), *graph_);
        //std::cout << "index '" << get(vertex_index,g,*vp.first) << "' ";
        //std::cout << "name '" << get(vertex_name,g,*vp.first) << "'"
        //<< std::endl;
        std::vector<double> my_coords = vertexCoords(get(vertex_name,g,*vp.first));
        //graph_traits<ompl::base::PlannerData::Graph>::vertex_descriptor my_vertex = boost::add_vertex(*graph_);
        //graph_traits<ompl::geometric::PRM::Graph>::vertex_descriptor other_vertex = *vp.first;
        //graph_traits<ompl::base::PlannerData::Graph>::vertex_descriptor my_vertex = boost::add_vertex(*graph_);
        //graph_traits<ompl::base::PlannerData::Graph>::vertex_descriptor other_vertex = *vp.first;
        //graph_[my_vertex].added_vertex(other_vertex);

        //test_type[my_vertex] = get(vertex_index, g, *vp.first);
        //*graph_->added_vertex(*vp.first);
    }
    /*ROS_INFO_STREAM("vertices number after is: " << boost::num_vertices(*graph_));
    ROS_INFO_STREAM("edges number after is: " << boost::num_edges(*graph_));
    robot_state::RobotState my_robot_state(robot_model);
    ompl::base::ScopedState<> s(oi.getPlanningContextManager().getPlanningContext("both_arms", "JointModel")->getOMPLStateSpace());
    for (auto vp = vertices(*graph_); vp.first != vp.second; ++vp.first){
        ROS_INFO_STREAM("vertices index is: " << get(vertex_index, *graph_, *vp.first));
        //ROS_INFO_STREAM("vertices name is: " << get(vertex_name, *graph_, *vp.first));
        //graph_traits<PlannerDataGraph>::vertex_descriptor other_vertex = *vp.first;
        //s = *get(vertex_type_t(), *graph_)[*vp.first]->getState(); //problem because verteices aren't of PlannerDataVertex type
        ROS_ERROR_STREAM(" i have my state ");
        //ompl::base::State* my_state = get(ompl::geometric::PRM::vertex_state_t(), *graph_, *vp.first);
        ompl::base::PlannerDataVertex* my_vertex = get(vertex_type_t(), *graph_, *vp.first);
        ROS_INFO_STREAM("my tag is: " << my_vertex->getTag());
        //const ompl::base::State* my_state = my_vertex->getState();
        //ROS_INFO_STREAM(my_state->as<ompl_interface::ModelBasedStateSpace::StateType>()->values);
        //oi.getPlanningContextManager().getPlanningContext("both_arms", "JointModel")->getOMPLStateSpace()->printState(my_state, std::cout);
        /*std::vector<double> coords(s.reals());
        std::vector<double>::iterator my_iter;
        for(my_iter = coords.begin(); my_iter != coords.end(); ++my_iter)
            ROS_ERROR_STREAM("my coordinates: " << (*my_iter));*/
    //}
    /*ompl::base::ScopedState<> s(oi.getPlanningContextManager().getPlanningContext("both_arms", "JointModel")->getOMPLStateSpace());
    for (auto vp = vertices(g); vp.first != vp.second; ++vp.first)
        {
            std::cout << "index '" << get(vertex_index,g,*vp.first) << "' ";
            std::cout << "name '" << get(vertex_name,g,*vp.first) << "'"
            << std::endl;
            //s = *get(vertex_type_t(), g)[*vp.first]->getState();
            //++vp.second;

        }
    for (auto vp = edges(g); vp.first != vp.second; ++vp.first)
        {
        ROS_INFO_STREAM("edge weight: " << *vp.first);
    }*/
    /*
    //add_vertex()
    ompl::geometric::PRM::Graph prm_graph;
    prm_graph.added_vertex(g.stored_vertex);

    std::filebuf fb2;
    fb2.open("test3.xml",std::ios::out);
    std::ostream is2(&fb2);
    boost::write_graphml(is2, g, dp);*/
    ROS_INFO_STREAM("finished!!!!!");
    return 0;
}
