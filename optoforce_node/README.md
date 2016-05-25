## Instructions

1. Dependencies

This ROS package depends on optoforce driver developed in Tecnalia.
[https link for package](http://gitsrvbav01/git/Repository/optoforce)

git link: http://gitsrvbav01/git/optoforce.git

2. Configuration

Within **cfg** folder **acquisition_params.yaml** file can be found. This file is loaded when *optoforce_node.launch* is executed

3. Execution

roslaunch optoforce_node optoforce_node.launch

4. Plot

Depending on the OptoForce FT sensor data to be visualized, a different topic has to be choosen.

* IRE005 OptoForce device

rqt_plot /optoforce_node/wrench_IRE005/wrench/force

rqt_plot /optoforce_node//wrench_IRE005/wrench/torque


* IRE004 OptoForce device

rqt_plot /optoforce_node/wrench_/wrench/force

rqt_plot /optoforce_node/wrench_/wrench/torque

