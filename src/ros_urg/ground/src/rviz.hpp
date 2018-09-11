#include <ros/ros.h>

#include <visualization_msgs/Marker.h>

#include <vector>

#include "module.hpp"

class cRviz {
  private:
  public:
  void set_ransac(visualization_msgs::Marker &list, ros::Time stamp);
  void set_field(visualization_msgs::Marker &list, ros::Time stamp);
  void set_urg(visualization_msgs::Marker &list, ros::Time stamp);
  void set_particle(visualization_msgs::Marker &list, ros::Time stamp);
  void print_point(std::vector<tFrame> &data,
		   visualization_msgs::Marker &points_list);
  void print_line(std::vector<tLine> &data,
		  visualization_msgs::Marker &line_list);
};
