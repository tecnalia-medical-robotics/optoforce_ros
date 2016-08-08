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
    void transmitStart ();

    //! Inherited virtual method from optoforce_node
    //! Stop transmision trough topics, only enable the flag
    void transmitStop ();

  private:
    //! Wrench Publisher
    ros::Publisher wrench_pub_[2];

    //! ROS Subscribers
    //! subs_[0]: enable_publish
    //! subs_[1]: enable_store
    ros::Subscriber subs_[2];

    //! Calback enable/disable publishing topic
    void enablePublishCB(const std_msgs::Bool::ConstPtr& msg);

    //! Calback enable/disable storing data to file when accquisition finish
    void enableStoreCB(const std_msgs::Bool::ConstPtr& msg);

};
