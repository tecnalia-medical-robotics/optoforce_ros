#include "optoforce_node.h"
#include <optoforce_ros/OptoForceAction.h>
#include <actionlib/server/simple_action_server.h>

class optoforce_action_server : public optoforce_node {

  public:

    // Constructor
    // Initialize ROS Subscribers
    optoforce_action_server();

    // Destructor
    ~optoforce_action_server();

    // Inherited virtual method from optoforce_node
    // Start transmision trough topics, only enable the flag
    void transmitStart ();

    // Inherited virtual method from optoforce_node
    // Stop transmision trough topics, only enable the flag
    void transmitStop ();

  private:

};
