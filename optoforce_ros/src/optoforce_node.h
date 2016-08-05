#include <optoforce/optoforce_acquisition.hpp>

#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

class optoforce_node {
  public:
    //! Constructor
    optoforce_node();

    //! Destructor
    ~optoforce_node();

    //! init
    int init();

    //! configure
    int configure();

    //! run
    int run();

    // Flag that enables publishing
    bool puplish_enable_;

    // Flag that enables storing data
    bool storeData_enable_;

    // This function must be overwriten by the derived class
    // Start transmision of data independently of the interface
    virtual void transmitStart() {};

    // This function must be overwriten by the derived class
    // Stop transmision of data independently of the interface
    virtual void transmitStop() {};

  protected:
    std::vector<geometry_msgs::WrenchStamped> wrench_;

    //! ROS node handler
    ros::NodeHandle nh_;


  private:

    // finish node
    void finish();



    OptoforceAcquisition * force_acquisition_;

    // Wrench Publisher
    ros::Publisher wrench_pub_[2];

    // Frequency in which the program will read sensor data
    int acquisition_rate_;

    // Publish frequency
    int loop_rate_;

    // Number of devices connected
    int connectedDAQs_;

    // Senor's transmission frequency
    int transmission_speed_;

    // Sensor Filter
    int filter_;

    // File where data will be stored
    std::string filename_;

    // Samples to be stored
    int num_samples_;

    std::vector<std::string> ldevice_;
    std::vector<std::vector<float> > lcalib_;
    std::vector<int> lspeed_;

};
