#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/base/PlannerDataGraph.h>
#include <moveit/ompl_interface/detail/state_validity_checker.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <fstream>
#include <istream>
#include <tinyxml.h>
#include <boost/graph/graphml.hpp>
#include <boost/property_map/function_property_map.hpp>
//#include <boost/graph/graph_mutability_traits.hpp>


namespace ob = ompl::base;
int number_of_vertices;
size_t get_size(std::vector<double> input){
    return input.size();
}

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

// Property map for extracting states as arrays of doubles
std::string vertexCoords (ompl::base::PlannerData::Graph::Type &g,
                          ompl::base::ScopedState<>& s,
                          ompl::base::PlannerData::Graph::Vertex v)
{
    ROS_ERROR("trying to read vertices coordinates");
    s = *get(vertex_type_t(), g)[v]->getState();
    std::vector<double> coords(s.reals());
    std::ostringstream sstream;
    if (coords.size()>0)
    {
        sstream << coords[0];
        for (std::size_t i = 1; i < coords.size(); ++i)
            sstream << ',' << coords[i];
    }
    return sstream.str();
}

bool terminate_condition(const boost::shared_ptr<ompl::geometric::PRM>& my_prm){
    ompl::base::PlannerData my_data(my_prm->getSpaceInformation());
    my_prm->getPlannerData(my_data);
    ROS_INFO_STREAM("Number of vertices is: " << my_data.numVertices() );
    return my_data.numVertices() > number_of_vertices;
    //return false;
}

double edgeWeightAsDouble(ompl::base::PlannerData::Graph::Type &g,
                          ompl::base::PlannerData::Graph::Edge e)
{
    ROS_ERROR("trying to read edge weight");
    return get(boost::edge_weight_t(), g)[e].value();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_our_prm");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  nh.getParam("number_of_vertices", number_of_vertices);

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
  planning_scene::PlanningScene m_planning_scene(robot_model);

  //functions to invetigate in planning scene class
  /*std::filebuf fb, fb2;
  fb.open ("shapes.txt",std::ios::out);
  std::ostream os(&fb);
  m_planning_scene.saveGeometryToStream(os);
  fb.close();
  m_planning_scene.processCollisionObjectMsg();
  m_planning_scene.loadGeometryFromStream();
  Eigen::Isometry3d test_isometry;
  test_isometry.matrix();*/


  ompl_interface::OMPLInterface oi(robot_model, nh);
  ompl_interface::ModelBasedStateSpacePtr model_based_space = oi.getPlanningContext("left_arm", "JointModel")->getOMPLStateSpace();
  ompl_interface::ModelBasedPlanningContextPtr context_ptr = oi.getPlanningContextManager().getPlanningContext("left_arm", "JointModel");

  ompl::base::StateSpacePtr my_space_ptr = model_based_space;
  ompl::geometric::SimpleSetup my_setup(my_space_ptr);
  my_setup.setStateValidityChecker(std::bind(&isStateValid, std::placeholders::_1, robot_model, context_ptr));
  my_setup.getStateSpace()->setup();
  my_setup.getSpaceInformation()->setup();

  ompl::base::ProblemDefinitionPtr pdf(new ompl::base::ProblemDefinition(my_setup.getSpaceInformation()));

  boost::shared_ptr<ompl::geometric::PRM> my_prm(new ompl::geometric::PRM(my_setup.getSpaceInformation()));
  //boost::shared_ptr<ompl::geometric::PRM> test_prm(new ompl::geometric::PRM(my_setup.getSpaceInformation()));

  my_prm->setProblemDefinition(pdf);
  //ompl::base::PlannerDataPtr my_data_ptr(new ompl::base::PlannerData(my_setup.getSpaceInformation()));
  //ompl::base::PlannerTerminationCondition ptc = ompl::base::timedPlannerTerminationCondition(1.0);
  my_setup.setPlanner(my_prm);


  ompl::base::PlannerTerminationCondition ptc(std::bind(&terminate_condition, my_prm));
  my_prm->growRoadmap(ptc);


  ompl::base::PlannerData my_data(my_setup.getSpaceInformation());
  my_prm->getPlannerData(my_data);
  std::map<std::string, std::string> planner_data_properties = my_data.properties;
  std::map<std::string, std::string>::iterator itr;
  for(itr = planner_data_properties.begin(); itr != planner_data_properties.end(); ++itr)
        ROS_INFO_STREAM("plannar property: " << (*itr).first << " is: " << (*itr).second);

  /*void* graphRaw_;
  graphRaw_ = new ob::PlannerData::Graph;
  ompl::base::PlannerData::Graph* graph_ = reinterpret_cast<ompl::base::PlannerData::Graph*>(graphRaw_);

  boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS,
                                 boost::property<vertex_type_t, ompl::base::PlannerDataVertex*,
                                 boost::property<boost::vertex_index_t, unsigned int> >,
                                 boost::property<edge_type_t, ompl::base::PlannerDataEdge*,
                                 boost::property<boost::edge_weight_t, ompl::base::Cost> > > graph;


  void* graphRaw_2;
  graphRaw_2 = new ompl::geometric::PRM::Graph;
  ompl::geometric::PRM::Graph* test_graph = reinterpret_cast<ompl::geometric::PRM::Graph*>(graphRaw_2);
  //ompl::base::PlannerData::Graph::Type& another_graph = my_data.toBoostGraph();*/

  std::filebuf fb, fb2;
  //fb.open ("test2.xml",std::ios::out);
  //fb2.open("test3.xml",std::ios::out);
  //std::istream is(&fb);

  /*std::string my_string;
  while(!is.eof()){
      getline(is, my_string);
      ROS_WARN_STREAM("recieved stream is: " << my_string);
  }*/

  /*std::ostream is2(&fb2);
  boost::function_property_map<
      std::function<double (ompl::base::PlannerData::Graph::Edge)>,
      ompl::base::PlannerData::Graph::Edge,
      double>
      weightmap(std::bind(&edgeWeightAsDouble, graph, std::placeholders::_1));

  ompl::base::ScopedState<> s(oi.getPlanningContextManager().getPlanningContext("both_arms", "JointModel")->getOMPLStateSpace());
  boost::function_property_map<
      std::function<std::string (ompl::base::PlannerData::Graph::Vertex)>,
      ompl::base::PlannerData::Graph::Vertex,
      std::string >
      coordsmap(std::bind(&vertexCoords, graph, s, std::placeholders::_1));
  //ompl::geometric::PRM::Graph my_graph;

  boost::dynamic_properties dp;
  dp.property("coords", coordsmap);
  dp.property("weight", weightmap);
  //dp.property()*/


  /*ROS_ERROR("I am here .....");
  std::vector<double> h(1);
  //h.push_back(1.2);
  ROS_WARN_STREAM("size is: " << h.size());
  boost::read_graphml(is, graph, dp, get_size(h));
  OMPL_INFORM("Created %u states", boost::num_vertices(graph));
  boost::write_graphml(is2, graph, dp);*/

  fb2.open ("test2.xml",std::ios::out);
  std::ostream os2(&fb2);
  my_data.printGraphML(os2);
  //my_data.printGraphviz(os);
  //fb.close();
  fb2.close();
  /*
  ROS_INFO_STREAM("starting test road map");
  TiXmlDocument saved_map("/home/ghanim/.ros/test2.xml");
  saved_map.LoadFile();

  //saved_map.Print();
  TiXmlNode* NodeList = 0;
  TiXmlElement* nodeElement = saved_map.RootElement();
  ROS_INFO_STREAM(nodeElement->Attribute("id"));
  NodeList = saved_map.FirstChild();
  nodeElement = NodeList->ToElement();
  //std::string id;
  //nodeElement->QueryStringAttribute("coords", &id);
  TiXmlNode* child = 0;
  while( (child = nodeElement->IterateChildren(child)))
  {
      ROS_INFO("hello_again");
  }

  //ROS_INFO_STREAM("id is: " << id);


  ROS_INFO_STREAM("finished!!!!!");
  //ros::waitForShutdown();*/
  return 0;
}
