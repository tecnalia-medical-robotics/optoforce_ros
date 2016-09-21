/********************************************
 * This program adds an interface through topics to:
 * ROS Subscribers
 *   start_publishing:
 *      true:   if acquisition not started, start new acquisition and publish data.
 *              if acquisition started, publish data.
 *      false:  if acquisition started, stop acquisition and stop publishing.
 *              if acquisition not started, do nothing
 *   start_new_acquisition:
 *      true:   if acquisition not started, start new acquisition.
 *              if acquisition started, stop previous acquisition and start new acquisition.
 *      false:  if acquisition not started, do nothing
 *              if acquisition started, stop previous acquisition
 *   auto_store:
 *      true:   store data after an acquisition finishes
 *      false:  do not store data after an acquisition finishes
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
  start_recording_ = false;
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
  subs_[2] = nh_.subscribe("auto_store",
                           1,
                           &optoforce_topic::autoStoreCB,
                           this);
}

// Calback enable/disable publishing topic
void optoforce_topic::autoStoreCB(const std_msgs::Bool::ConstPtr& msg)
{
  /*
  ROS_INFO_STREAM("[optoforce_topic::autoStoreCB] data: " << int(msg->data));

  if (msg->data == true)
    force_acquisition_->setAutoStore(true);
  else
    force_acquisition_->setAutoStore(false);
   */
}

// Calback enable/disable publishing topic
void optoforce_topic::startPublishingCB(const std_msgs::Bool::ConstPtr& msg)
{
  ROS_INFO_STREAM("[optoforce_topic::startPublishingCB] data: " << int(msg->data));

  puplish_enable_ = msg->data;

  if (puplish_enable_)
  {
    if (!force_acquisition_->isReading())
    {
      ROS_INFO("[optoforce_topic::startPublishingCB] start acq");
      force_acquisition_->startReading();
    }
    //transmitStart();
  }
  else
  {
    puplish_enable_ = false;
    //transmitStop();
  }
}

// Calback enable/disable storing data to file when accquisition finish
void optoforce_topic::startRecordingCB(const std_msgs::Bool::ConstPtr& msg)
{
  ROS_INFO_STREAM("[optoforce_topic::startRecordingCB] data: " << int(msg->data));

  start_recording_ = msg->data;

  if (start_recording_)
  {
   if (!force_acquisition_->isRecording())
   {
     ROS_INFO("[optoforce_topic::startRecordingCB] is not Recording. START RECORDING");
     force_acquisition_->startRecording();
   }
   else
   {
     ROS_INFO("[optoforce_topic::startRecordingCB] is not Reading. proceed to start reading and recording data");
     ROS_INFO("[optoforce_topic::startRecordingCB] STOP RECORDING");
     force_acquisition_->stopRecording();
     ROS_INFO("[optoforce_topic::startRecordingCB] START RECORDING");
     force_acquisition_->startRecording();
   }

  }
  else if (msg->data == false)
  {
    if (force_acquisition_->isRecording())
    {
      ROS_INFO("[optoforce_topic::startRecordingCB] is Recording. proceed to stop recording and save data ");
      force_acquisition_->stopRecording();
    }
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

    bool publish_enable = false;
    // It is assumed that, callback function enables acquisition
    if(puplish_enable_ && force_acquisition_->isReading() )
    {
      latest_samples.clear();
      //std::cout << "**************************************size: " << latest_samples.size() << std::endl;
      force_acquisition_->getData(latest_samples);
      //std::cout << "**************************************size: " << latest_samples.size() << std::endl;;
      //std::cout << "**************************************size: " << latest_samples[0].size() << std::endl;;
      //std::cout << "**************************************size: " << latest_samples[1].size() << std::endl;;
      if ( latest_samples.size() == connectedDAQs_)
      {
        for (int i = 0; i < connectedDAQs_; i++)
        {
          if (latest_samples[i].size() == 6)
          {
            wrench.header.stamp = ros::Time::now();
            wrench.wrench.force.x  = latest_samples[i][0];
            wrench.wrench.force.y  = latest_samples[i][1];
            wrench.wrench.force.z  = latest_samples[i][2];
            wrench.wrench.torque.x = latest_samples[i][3];
            wrench.wrench.torque.y = latest_samples[i][4];
            wrench.wrench.torque.z = latest_samples[i][5];
            wrench_pub_[i].publish(wrench);    
          }
        }
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
