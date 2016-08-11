/**
 * @file   optoforce_topic.h
 * @author Asier Fernandez <asier.fernandez@tecnalia.com>
 * @date   2016
 *
 * Copyright 2016 Tecnalia Research and Innovation.
 *
 * @brief Interface trough topics to optoforce_node
 *          ROS Publishers
 *            enable_publish
 *            enable_store
 *          ROS Subscribers
 *            enable_publish
 *            enable_store
 */
#include "optoforce_node.h"
#include "std_msgs/Bool.h"

class optoforce_topic : public optoforce_node {

  public:

    //! Constructor
    //! Initialize ROS Publishers and Subscribers
    optoforce_topic();

    //! Destructor
    ~optoforce_topic();

    //! Inherited virtual method from optoforce_node
    //! Start transmision trough topics, only enable the flag
    //! Add ROS Publisher and Subscribers
    void add_ros_interface();

    //! Inherited virtual method from optoforce_node
    //! Start transmision trough topics, only enable the flag
    int run ();


    //! Inherited virtual method from optoforce_node
    //! Start transmision trough topics, only enable the flag
    void transmitStart ();

    //! Inherited virtual method from optoforce_node
    //! Stop transmision trough topics, only enable the flag
    void transmitStop ();

  private:
    //! Wrench Publisher
    ros::Publisher wrench_pub_[2];

    //! ROS Subscribers
    //! subs_[0]: start_publishing
    //! subs_[1]: start_new_acquisition
    //! subs_[2]: auto_store
    ros::Subscriber subs_[3];

    //! Calback enable/disable publishing topic
    void startPublishingCB(const std_msgs::Bool::ConstPtr& msg);

    //! Calback enable/disable new acquisition
    void startRecordingCB(const std_msgs::Bool::ConstPtr& msg);

    //! Calback enable/disable auto-storing data after an acquisition starts
    void autoStoreCB(const std_msgs::Bool::ConstPtr& msg);


};
