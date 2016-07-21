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
  as_(nh_, name, boost::bind(&optoforce_action_server::executeCB, this, _1),false),
  action_name_(name)
{
  as_.start();
}

optoforce_action_server::~optoforce_action_server()
{

}
void optoforce_action_server::executeCB(const actionlib::SimpleActionServer<optoforce_ros::OptoForceAction>::GoalConstPtr& goal)
{
  ROS_INFO("[optoforce_action_server::executeCB] Enter executeCB");

  int it = 0;

  ros::Rate timer(goal->freq); // 1Hz timer

  while (it < goal->duration)
  {
    //ROS_INFO("[optoforce_action_server::executeCB] Running executeCB");
    if (as_.isPreemptRequested()){
       ROS_WARN("goal cancelled!");
       result_.result = 0;
       as_.setAborted(result_); // tell the client we have given up on this goal; send the result message as well
       return; // done with callback
    }
    std::vector<float> data;
    data.push_back(0.2);
    data.push_back(0.3);
    //ROS_INFO_STREAM("force: " << wrench_[0].wrench.force.z);
    /*
    for (int i = 0; i < 6; i++)
    {
        data.push_back(wrench_[0].wrench.force.x);
        data.push_back(wrench_[0].wrench.force.y);
        data.push_back(wrench_[0].wrench.force.z);
        data.push_back(wrench_[0].wrench.torque.x);
        data.push_back(wrench_[0].wrench.torque.y);
        data.push_back(wrench_[0].wrench.torque.z);
    }*/

    feedback_.wrench = data;

    as_.publishFeedback(feedback_); // send feedback to the action client that requested this goal

    it++;
    timer.sleep();
  }

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

  std::string action_name = "action_server";
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
