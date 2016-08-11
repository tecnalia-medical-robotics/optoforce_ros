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

}

optoforce_topic::~optoforce_topic()
{

}
void optoforce_topic::add_ros_interface()
{
  std::cout << "[optoforce_ros_interface]connectedDAQs_" << connectedDAQs_ << std::endl;

  // Create Publishers
  for (int i = 0; i < connectedDAQs_; i++)
  {
      std::string publisher_name = "wrench_" + device_list_[i].name;
      wrench_pub_[i] = nh_.advertise<geometry_msgs::WrenchStamped>(publisher_name, 1);
  }

  subs_[0] = nh_.subscribe("start_publishing",
                           1,
                           &optoforce_topic::startPublishingCB,
                           this);

  subs_[1] = nh_.subscribe("start_new_acquisition",
                           1,
                           &optoforce_topic::startRecordingCB,
                           this);
}

// Calback enable/disable publishing topic
void optoforce_topic::startPublishingCB(const std_msgs::Bool::ConstPtr& msg)
{
  ROS_INFO_STREAM("[optoforce_topic::startPublishingCB] data: " << int(msg->data));

  if (msg->data == true)
  {
    if (!force_acquisition_->isRecording())
    {
      ROS_INFO("[optoforce_topic::startPublishingCB] start acq");
      force_acquisition_->startRecording();
    }

    puplish_enable_ = true;
    //transmitStart();
  }
  else if (msg->data == false)
    puplish_enable_ = false;
    //transmitStop();
}

// Calback enable/disable storing data to file when accquisition finish
void optoforce_topic::startRecordingCB(const std_msgs::Bool::ConstPtr& msg)
{
  ROS_INFO_STREAM("[optoforce_topic::enableStoreCB] data: " << int(msg->data));

  if (msg->data == true)
    force_acquisition_->startRecording();

  else if (msg->data == false)
  {
    if (force_acquisition_->isRecording())
    force_acquisition_->stopRecording();
    force_acquisition_->storeData();
    puplish_enable_ = false;
  }
}
// Inherited virtual method from optoforce_node
// Start transmision trough topics, only enable the flag
int optoforce_topic::run()
{
  ros::Rate loop_rate(loop_rate_);
  geometry_msgs::WrenchStamped wrench;
  std::vector< std::vector<float> > latest_samples;

  while(ros::ok())
  {
    // It is assumed that, callback function enables acquisition
    if (puplish_enable_)
    {
      latest_samples.clear();
      force_acquisition_->getData(latest_samples);

      if (connectedDAQs_ > 0 )
      {
        wrench.header.stamp = ros::Time::now();
        wrench.wrench.force.x  = latest_samples[0][0];
        wrench.wrench.force.y  = latest_samples[0][1];
        wrench.wrench.force.z  = latest_samples[0][2];
        wrench.wrench.torque.x = latest_samples[0][3];
        wrench.wrench.torque.y = latest_samples[0][4];
        wrench.wrench.torque.z = latest_samples[0][5];
        wrench_pub_[0].publish(wrench);
      }
      if (connectedDAQs_ == 2)
      {
        wrench.header.stamp = ros::Time::now();
        wrench.wrench.force.x  = latest_samples[1][0];
        wrench.wrench.force.y  = latest_samples[1][1];
        wrench.wrench.force.z  = latest_samples[1][2];
        wrench.wrench.torque.x = latest_samples[1][3];
        wrench.wrench.torque.y = latest_samples[1][4];
        wrench.wrench.torque.z = latest_samples[1][5];
        wrench_pub_[1].publish(wrench);
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
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


int main(int argc, char* argv[])
{

  ros::init(argc, argv, "optoforce_topic");
  ROS_INFO_STREAM("[optoforce_topic] Node name is:" << ros::this_node::getName());

  optoforce_topic of_topic;

  if (of_topic.init() < 0)
  {
    std::cout << "[optoforce_topic] optoforce_topic could not be initialized" << std::endl;
  }
  else
  {
    std::cout << "[optoforce_topic] optoforce_topic Correctly initialized" << std::endl;

    // Add ROS Publisher and Subscribers
    of_topic.add_ros_interface();

    // Execute main loop of optoforce_node
    of_topic.run();
  }
  std::cout << "[optoforce_topic] exit main" << std::endl;

  return 1;

}
