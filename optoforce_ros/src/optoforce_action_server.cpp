/********************************************
 * This program adds an interface through topics to:
 *   enable publishing topics
 *   enable store data to file when accquisition finish
 *
 * The program inherits optoforce_node, which:
 *   reads parameters from ROS parameter server
 *   initialize  optoforce driver
 *   get periodically wrench data
 *   if configured can:
 *     publish wrench data
 *     store   wrench data
 */
#include "optoforce_action_server.h"

optoforce_action_server::optoforce_action_server(std::string name) :
  as_(nh_, "example_action", boost::bind(&optoforce_action_server::executeCB, this, _1),false),
  action_name_(name)
{
  //ros::NodeHandle nh("~");

}

optoforce_action_server::~optoforce_action_server()
{

}
void optoforce_action_server::executeCB(const actionlib::SimpleActionServer<optoforce_ros::OptoForceAction>::GoalConstPtr& goal)
{


}

// Inherited virtual method from optoforce_node
// Start transmision trough topics, only enable the flag
void optoforce_action_server::transmitStart()
{
  puplish_enable_ = true;
}

// Inherited virtual method from optoforce_node
// Stop transmision trough topics, only enable the flag
void optoforce_action_server::transmitStop()
{
  puplish_enable_ = false;
}




int main(int argc, char* argv[])
{

  ros::init(argc, argv, "optoforce_action_server");
  ROS_INFO_STREAM("Node name is:" << ros::this_node::getName());

  std::string action_name = "optoforce_action";
  optoforce_action_server optoforce_as(action_name);

  if (optoforce_as.init() < 0)
  {
    std::cout << "optoforce_topic could not be initialized" << std::endl;
  }
  else
  {
    std::cout << "optoforce_topic Correctly initialized" << std::endl;

    // Execute main loop of optoforce_node
    optoforce_as.run();
  }
  std::cout << "exit main" << std::endl;

  return 1;

}
