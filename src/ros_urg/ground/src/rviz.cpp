#include <iostream>

#include "rviz.hpp"

/**
 * @brief rvizに送信するときの形式設定
 * 
 * @param list   送信データのリスト
 * @param stamp  データのタイムスタンプ
 */
void cRviz::set_ransac(visualization_msgs::Marker &list, ros::Time stamp)
{
  list.header.frame_id = "/laser";
  list.header.stamp = stamp;
  list.ns = "ransac";
  list.id = 0;
  list.type = visualization_msgs::Marker::LINE_LIST;
  list.action = visualization_msgs::Marker::ADD;
  list.scale.x = 0.3;
  list.color.r = 1.0f;
  list.color.b = 1.0f;
  list.color.g = 0;
  list.color.a = 1.0f;
}

/**
 * @brief rvizに送信するときの形式設定
 * 
 * @param list   送信データのリスト
 * @param stamp  データのタイムスタンプ
 */
void cRviz::set_field(visualization_msgs::Marker &list, ros::Time stamp)
{
  list.header.frame_id = "/laser";
  list.header.stamp = stamp;
  list.ns = "field";
  list.id = 0;
  list.type = visualization_msgs::Marker::LINE_LIST;
  list.action = visualization_msgs::Marker::ADD;
  list.scale.x = 0.1;
  list.color.r = 1.0f;
  list.color.a = 1.0f;
  list.color.b = 0;
}

/**
 * @brief rvizに送信するときの形式設定
 * 
 * @param list   送信データのリスト
 * @param stamp  データのタイムスタンプ
 */
void cRviz::set_urg(visualization_msgs::Marker &list, ros::Time stamp)
{
  list.header.frame_id = "/laser";
  list.header.stamp = stamp;
  list.ns = "points";
  list.id = 0;
  list.type = visualization_msgs::Marker::POINTS;
  list.action = visualization_msgs::Marker::ADD;
  list.scale.x = 0.1;
  list.scale.y = 0.1;
  list.color.g = 1.0f;
  list.color.a = 1.0f;
}

/**
 * @brief rvizに送信するときの形式設定
 * 
 * @param list   送信データのリスト
 * @param stamp  データのタイムスタンプ
 */
void cRviz::set_particle(visualization_msgs::Marker &list, ros::Time stamp)
{
  list.header.frame_id = "/laser";
  list.header.stamp = stamp;
  list.ns = "particle";
  list.id = 0;
  list.type = visualization_msgs::Marker::POINTS;
  list.action = visualization_msgs::Marker::ADD;
  list.scale.x = 0.05;
  list.scale.z = 0.05;
  list.scale.y = 0.05;
  list.color.r = 1.0f;
  list.color.g = 0.5f;
  list.color.a = 1.0f;
  list.color.b = 0;
}

/**
 * @brief listに点群を格納
 * 
 * @param data 　　　　点群データ
 * @param points_list rviz送信用のリスト
 */
void cRviz::print_point(std::vector<tFrame> &data,
                        visualization_msgs::Marker &points_list)
{
  for (int i = 0; i < data.size(); i++)
  {
    geometry_msgs::Point points;
    points.x = data[i].x / 1000;
    points.y = data[i].y / 1000;
    points.z = 0;
    points_list.points.push_back(points);
    points_list.lifetime = ros::Duration();
  }
}

/**
 * @brief listに検知した直線を格納
 * 
 * @param data       直線データ    
 * @param line_list  rviz送信用のリスト
 */
void cRviz::print_line(std::vector<tLine> &data,
                       visualization_msgs::Marker &line_list)
{
  geometry_msgs::Point line;

  for (int i = 0; i < data.size(); i++)
  {
    if (fabs(data[i].b / data[i].a) < 1)
    {
      line.x = -data[i].c / data[i].a / 1000;
      line.y = 10.0;
      line.z = 0;
      line_list.points.push_back(line);

      line.x = -data[i].c / data[i].a / 1000;
      line.y = -10.0;
      line.z = 0;
      line_list.points.push_back(line);
      line_list.lifetime = ros::Duration();
    }
    else
    {
      line.x = 10.0;
      line.y = -line.x * data[i].a / data[i].b - data[i].c / data[i].b / 1000;
      line.z = 0;
      line_list.points.push_back(line);

      line.x = -10.0;
      line.y = -line.x * data[i].a / data[i].b - data[i].c / data[i].b / 1000;
      line.z = 0;
      line_list.points.push_back(line);
      line_list.lifetime = ros::Duration();
    }
  }
}
