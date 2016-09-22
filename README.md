# Description

This ROS package adds topic and action based interface to the optoforce driver developed in Tecnalia.
[https link for package](https://github.com/tecnalia-medical-robotics/optoforce.git)

# Instructions

### Configuration

Within **cfg** folder **acquisition_params.yaml** file can be found. This file is loaded when *optoforce_node.launch* is executed.

**acquisition_params.yaml** contains all relevant configuration parameters that can be modified.
It's important to point that as many as defined devices have to be phisically plugged in order to execute properly.

Two type of acquisition frequencies can be modified:

```bash
# Publish frequency
loop_rate: 100 # Frecuency in Hz, in which topics will be published

# Acquisition frecuency
acquisition_freq: 1000    # Frecuency in Hz, in which the program will read from DAQ
                          # This frequency has to be equal of faster than OptoForce Transmission speed

# OptoForce Sensors to be opened.
device:
	- name: "device_name"
      speed: 1000           # OptoForce DAQ Transmission frequency in Hz
```

### Basic usage

```bash
roslaunch optoforce_node optoforce_node.launch
```

### Plot

* Using rqt_plot    

For example, for a IRE005 OptoForce device

```bash
rqt_plot /optoforce_node/wrench_IRE005/wrench/force
rqt_plot /optoforce_node/wrench_IRE005/wrench/torque
```

* Using gnuplot:    

The node can save data in a csv file, and a easy way to plot this csv file is using gnuplot

```bash
set datafile separator ";" 
plot "filename.csv" using 1:2 with lines title "Fx(N)", "filename.csv" using 1:3 with lines title "Fy(N)", "filename.csv" using 1:4 with lines title "Fz(N)"
```