#include "optoforce_node.h"

optoforce_node::optoforce_node(): nh_("~")
{

}

optoforce_node::~optoforce_node()
{
  finish();
}
void optoforce_node::finish()
{
  std::cout << "FINISH program" << std::endl;

  if (force_acquisition_ != NULL)
  {
    delete force_acquisition_;
    force_acquisition_ = NULL;
  }
}

int optoforce_node::init()
{

  // Read configuration file
  configure();

  std::cout << "Looking for " << connectedDAQs_ << " connected DAQ " << std::endl;
  force_acquisition_ = new OptoforceAcquisition();

  if (!force_acquisition_->initDevices(connectedDAQs_))
  {
    std::cerr << "Something went wrong during initialization. Bye!" << std::endl;
    return -1;
  }

  for (int i = 0; i < connectedDAQs_; ++i)
  {
    std::cout << "ldevice[" << i << "] " << ldevice_[i] << std::endl;

    if (!force_acquisition_->isDeviceConnected(ldevice_[i]))
    {
      std::cerr << "[optoforce_initi]Could not find device " << ldevice_[i] << std::endl;
      std::cerr << "Quitting the application." << std::endl;
      finish();
    }
  }
  // Configure Force Sensors Speed
  for (int i = 0; i < connectedDAQs_; ++i)
  {
    if (!force_acquisition_->setSensorSpeed(lspeed_[i]))
    {
      std::cerr << "Could not setSensorSpeed" << std::endl;
    }
    else
      std::cout << "Correctly setFrequency:  " << lspeed_[i] << std::endl;
  }


  // Configure Force Sensors Filter Frequency in all connected devices
  if (!force_acquisition_->setSensorFilter(filter_))
    std::cerr << "Could not setSensorFilter" << std::endl;
  else
    std::cout << "Correctly setSensorFilter:  " << filter_ << std::endl;


  // Configure FT Calibration
  for (int i = 0; i < connectedDAQs_; ++i)
  {
    std::cout << "[optoforce_node::init]" << std::endl;
    if (!force_acquisition_->setDeviceCalibration(ldevice_[i], lcalib_[i]))
    {
      std::cerr << "Could not setDeviceCalibration" << std::endl;
    }
    else
    {
      std::cout << "Correctly setDeviceCalibration: [ ";

      for (int j = 0; j < lcalib_[i].size(); j++)
      {
        std::cout << lcalib_[i][j] << " ";
      }
      std::cout << "]" << std::endl;

    }
  }

  // Set to Zero All OptoForce devices
  force_acquisition_->setZeroAll();

  // Set Acquisition Frequency
  // This frequency determines how often we get a new data. Independently from Sensor Transmission Rate
  force_acquisition_->setAcquisitionFrequency(acquisition_rate_);

  // Set Filename to Store Data
  force_acquisition_->setFilename(filename_);

  // Initialize ROS publisher
  std::vector<std::string> serial_numbers;
  force_acquisition_->getSerialNumbers(serial_numbers);

  // By default Do Store publish
  storeData_enable_ = false;

  // By default DO publish
  puplish_enable_ = false;

  // Create Publishers
  if (connectedDAQs_ > 0 )
  {
    std::string publisher_name = "wrench_" + serial_numbers[0];
    wrench_pub_[0] = nh_.advertise<geometry_msgs::WrenchStamped>(publisher_name, 1);
  }
  if (connectedDAQs_ > 0 && connectedDAQs_ == 2)
  {
    std::string publisher_name = "wrench_" + serial_numbers[1];
    wrench_pub_[1] = nh_.advertise<geometry_msgs::WrenchStamped>(publisher_name, 1);
  }


  return 0;
}

// Read
int optoforce_node::configure()
{
  nh_.param("loop_rate", loop_rate_, 100); // Loop Rate in Hz
  loop_rate_ = 100; // Loop Rate in Hz

  nh_.param("num_samples", num_samples_, 150000); // Maximun Number of Samples to be stored
  //num_samples_ = 10000; // Maximun Number of Samples to be stored

  nh_.param<std::string>("filename", filename_, "/tmp/optoforce_node"); // Loop Rate in Hz
  //filename_ = "test_optoforce_node";

  nh_.param("connectedDAQs", connectedDAQs_, 2); // Maximun Number of Samples to be stored
  //connectedDAQs_ = 2;

  acquisition_rate_ = 1000;   // Rate in Hz
  
  transmission_speed_ = 1000; // Rate in Hz
  
  filter_ = 15; // in Hz
  
  filename_ = "test_optoforce_node";

  // List devices names
  ldevice_.clear();
  //ldevice_.push_back("95 v1.0");  //IRE004
  //ldevice_.push_back("64 v0.9");  // IRE005
  ldevice_.push_back("IRE004");  //IRE004
  ldevice_.push_back("IRE005");  //IRE005
  lcalib_.clear();
  std::vector<float> calib;

  calib.clear();
  calib.push_back(97.78);      // IRE004
  calib.push_back(101.72);     // IRE004
  calib.push_back(20.53);      // IRE004
  calib.push_back(5210.6);     // IRE004
  calib.push_back(5267.2);     // IRE004
  calib.push_back(7659.7);     // IRE004
  lcalib_.push_back(calib);

  calib.clear();
  calib.push_back(92.6);       // IRE005
  calib.push_back(93.6);       // IRE005
  calib.push_back(20.12);      // IRE005
  calib.push_back(5054.3);     // IRE005
  calib.push_back(5085.4);     // IRE005
  calib.push_back(6912.5);     // IRE005
  lcalib_.push_back(calib);


  lspeed_.push_back(transmission_speed_);
  lspeed_.push_back(transmission_speed_);

  return 0;
}

int optoforce_node::run()
{

  std::cout << "[optoforce_node::run] start recording" << std::endl;
  std::vector< std::vector<float> > latest_samples;

  force_acquisition_->startRecording(num_samples_);

  ros::Rate loop_rate(loop_rate_);
  while(ros::ok() && (force_acquisition_ != NULL) && force_acquisition_->isRecording())
  {
    latest_samples.clear();

    force_acquisition_->getData(latest_samples);

    if (latest_samples.size() == connectedDAQs_)
    {
        bool isDataValid = true;

        // Check if all received data's dimension is 6
        for (int i = 0; i < latest_samples.size(); i++)
        {
          if (latest_samples[i].size() == 6)
            isDataValid = (isDataValid & true);
          else
            isDataValid = false;
        }

        // Check Force and Torque vector dimension i 6 -> 3 Force and 3 Torque
        if (isDataValid && puplish_enable_)
        {
          wrench_.clear();

          geometry_msgs::WrenchStamped wrench;

          if (connectedDAQs_ > 0 )
          {
            wrench.header.stamp = ros::Time::now();
            wrench.wrench.force.x  = latest_samples[0][0];
            wrench.wrench.force.y  = latest_samples[0][1];
            wrench.wrench.force.z  = latest_samples[0][2];
            wrench.wrench.torque.x = latest_samples[0][3];
            wrench.wrench.torque.y = latest_samples[0][4];
            wrench.wrench.torque.z = latest_samples[0][5];
            wrench_.push_back(wrench);
            wrench_pub_[0].publish(wrench_[0]);

          }
          if (connectedDAQs_ > 0 && connectedDAQs_ == 2)
          {
            wrench.wrench.force.x  = latest_samples[1][0];
            wrench.wrench.force.y  = latest_samples[1][1];
            wrench.wrench.force.z  = latest_samples[1][2];
            wrench.wrench.torque.x = latest_samples[1][3];
            wrench.wrench.torque.y = latest_samples[1][4];
            wrench.wrench.torque.z = latest_samples[1][5];
            wrench_.push_back(wrench);
            wrench_pub_[1].publish(wrench_[1]);
          }
        }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  force_acquisition_->stopRecording();

  if (storeData_enable_)
    force_acquisition_->storeData();

  // finish program
  finish();
  return 0;
}

//int main(int argc, char* argv[])
//{
//  ros::init(argc, argv, "optoforce_node");
//  ROS_INFO_STREAM("Node name is:" << ros::this_node::getName());
//  //ROS_ERROR_STREAM("Bye");

//  optoforce_node optoforce_node;

//  if (optoforce_node.init() < 0)
//  {
//    std::cout << "optoforce_node could not be initialized" << std::endl;
//  }
//  else
//  {
//    std::cout << "optoforce_node Correctly initialized" << std::endl;
//    optoforce_node.run();
//  }
//  std::cout << "exit main" << std::endl;
//  return 1;
//}
