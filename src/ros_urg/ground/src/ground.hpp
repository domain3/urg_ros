#ifndef SRC_GROUND_HPP_
#define SRC_GROUND_HPP_

#include "ros_urg/uart.h"
#include "sensor_msgs/LaserScan.h"

void posCallBack(const ros_urg::uart &msg);
void laserCallback(const sensor_msgs::LaserScan &msg);

#endif
