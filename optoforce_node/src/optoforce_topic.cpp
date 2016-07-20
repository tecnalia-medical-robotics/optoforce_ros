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
#include "optoforce_topic.h"

optoforce_topic::optoforce_topic()
{
  ros::NodeHandle nh("~");

  subs_[0] = nh.subscribe("enable_publish",
                           1,
                           &optoforce_topic::enablePublishCB,
                           this);

  subs_[1] = nh.subscribe("enable_store",
                           1,
                           &optoforce_topic::enableStoreCB,
                           this);
}

optoforce_topic::~optoforce_topic()
{

}

// Calback enable/disable publishing topic
void optoforce_topic::enablePublishCB(const std_msgs::Bool::ConstPtr& msg)
{
  ROS_INFO_STREAM("[optoforce_topic::enablePublishCB] data: " << int(msg->data));

  if (msg->data == true)
    transmitStart();
  else if (msg->data == false)
    transmitStop();
}

// Calback enable/disable storing data to file when accquisition finish
void optoforce_topic::enableStoreCB(const std_msgs::Bool::ConstPtr& msg)
{
  ROS_INFO_STREAM("[optoforce_topic::enableStoreCB] data: " << int(msg->data));

  if (msg->data == true)
    storeData_enable_ = true;
  else if (msg->data == false)
    storeData_enable_ = false;
}

// Inherited virtual method from optoforce_node
// Start transmision trough topics, only enable the flag
void optoforce_topic::transmitStart()
{
  puplish_enable_ = true;
}

// Inherited virtual method from optoforce_node
// Stop transmision trough topics, only enable the flag
void optoforce_topic::transmitStop()
{
  puplish_enable_ = false;
}


//int main(int argc, char* argv[])
//{

//  ros::init(argc, argv, "optoforce_topic");
//  ROS_INFO_STREAM("Node name is:" << ros::this_node::getName());

//  optoforce_topic of_topic;

//  if (of_topic.init() < 0)
//  {
//    std::cout << "optoforce_topic could not be initialized" << std::endl;
//  }
//  else
//  {
//    std::cout << "optoforce_topic Correctly initialized" << std::endl;

//    // Set Flag to Store Data
//    //of_topic.storeData_enable_ = true;

//    // Set Flag to enable publishing data
//    //of_topic.transmitStart();

//    // Execute main loop of optoforce_node
//    of_topic.run();
//  }
//  std::cout << "exit main" << std::endl;

//  return 1;

//}
