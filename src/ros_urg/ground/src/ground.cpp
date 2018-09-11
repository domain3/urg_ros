#include <iostream>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "sensor_msgs/LaserScan.h"

#include "ros_urg/uart.h"

#include "dead.hpp"
#include "ground.hpp"
#include "particleFilter.hpp"
#include "rviz.hpp"

namespace
{
namespace var
{

//データの送信用のPublisher
ros::Publisher ransac_publisher;
ros::Publisher points_publisher;
ros::Publisher field_publisher;
ros::Publisher particle_publisher;
ros::Publisher rewritePos_publisher;

//前のloop終了時間
ros::Time oldTime = (ros::Time)0;

//uart設定クラス
ros_urg::uart uart_data;

//書き込みが始まってたらtrueとなる
bool write_position = false;

} // namespace var
} // namespace

/**
 * @brief マイコンから自己位置データをもらう
 * 
 * @param msg 自己位置データ
 */
void posCallBack(const ros_urg::uart &msg)
{
  var::uart_data.x = msg.x;
  var::uart_data.y = msg.y;
  var::uart_data.angle = msg.angle;
  var::uart_data.stamp = msg.stamp;
  var::write_position = true;
}

/**
 * @brief urgのデータを入手し機体の自己位置を
 * 
 * @param msg 
 */
void laserCallback(const sensor_msgs::LaserScan &msg)
{
  //機体からのデータが来てから算出スタート
  if (!var::write_position)
    return;

  //時間の測定.
  ros::Time begin = ros::Time::now();
  ros::Duration loop = begin - var::oldTime;

  //機体の自己位置
  tPos robotPos;
  robotPos.x = var::uart_data.x;
  robotPos.y = var::uart_data.y;
  robotPos.angle = var::uart_data.angle;

  // std::cerr << robotPos.x << "," << robotPos.y << "," << robotPos.angle
  //          << std::endl;

  //rvizの用のデータリスト
  visualization_msgs::Marker ransac_list;
  visualization_msgs::Marker field_list;
  visualization_msgs::Marker urg_list;
  visualization_msgs::Marker particle_list;

  //rviz用のクラス
  cRviz rviz;

  //rvizの設定開始
  rviz.set_ransac(ransac_list, var::uart_data.stamp);
  rviz.set_field(field_list, var::uart_data.stamp);
  rviz.set_urg(urg_list, var::uart_data.stamp);
  rviz.set_particle(particle_list, var::uart_data.stamp);

  //自己位置の算出用クラス
  cDead dead;

  // urgのデータ数
  int points = (msg.angle_max - msg.angle_min) / msg.angle_increment + 1;

  //urgのデータ格納用
  std::vector<tUrg> urgData;

  //urgのデータ格納
  for (int i = 0; i < points; i++)
  {
    //角度算出
    double angle = msg.angle_min + i * msg.angle_increment;

    //条件にあうデータを格納
    if (msg.ranges[i] > 0.05 && angle > -M_PI / 2 && angle < M_PI / 2)
    {
      tUrg urg;
      urg.length = msg.ranges[i] * 1000;
      urg.angle = angle;
      urgData.push_back(urg);
      // std::cerr << angle << std::endl;
    }
  }

  //urgデータ(x-y)の格納用
  std::vector<tFrame> urg_data;

  //直線データの格納用
  std::vector<tLine> line_data;

  //自己位置候補
  std::vector<double> laserx;
  std::vector<double> lasery;

  //直線検知からの自己位置候補算出
  dead.execuseRansac(urgData, robotPos, line_data, urg_data, laserx, lasery);

  /*
  for (int i = 0; i < laserx.size(); i++) {
    std::cerr << "x, " << laserx[i] << ",";
  }
  std::cerr << std::endl;

  for (int i = 0; i < lasery.size(); i++) {
    std::cerr << "y, " << lasery[i] << ",";
  }
  std::cerr << std::endl;
*/

  //パーティクルの結果格納用
  tFrame particleFrame;

  //パーティクルのクラス作成
  cParticleFilter particlefilter = cParticleFilter(20, 20, 50, 50);

  //パーティクルに必要な情報入力
  particlefilter.init(robotPos);

  //重さの計算
  particlefilter.updateWeights(laserx, lasery);

  //重みから自己位置を算出
  particleFrame = particlefilter.getPos();

  //rvizにパーティクルの結果を送信
  particlefilter.pushRviz(particle_list);

  //データ削除
  particlefilter.resample();

  // dead.middle_num(rewritePos);

  // Push
  ros_urg::uart output; //出力データ.
  output.x = particleFrame.x - robotPos.x;
  output.y = particleFrame.y - robotPos.y;
  output.stamp = var::uart_data.stamp;
  output.flag = 1;

  if (urg_data.size() > 0)
    rviz.print_point(urg_data, urg_list);
  if (line_data.size() > 0)
    rviz.print_line(line_data, ransac_list);
  dead.printField(field_list);

  var::ransac_publisher.publish(ransac_list);
  var::points_publisher.publish(urg_list);
  var::field_publisher.publish(field_list);
  var::particle_publisher.publish(particle_list);
  var::rewritePos_publisher.publish(output);

  //計算時間の計算
  ros::Time end = ros::Time::now();
  ros::Duration diff = end - begin;
  var::oldTime = begin;

  std::cerr << output.x << ", " << output.y << "," << diff << ", " << loop
            << std::endl;
}

/**
 * @brief main関数
 */
int main(int argc, char **argv)
{
  std::cerr << "start ros ground" << std::endl;

  // rosの初期化.
  ros::init(argc, argv, "ground");
  ros::NodeHandle n;

  // 受信データのアドレス登録.
  ros::Subscriber urg = n.subscribe("urg_node/scan", 1, laserCallback);
  ros::Subscriber PosData = n.subscribe("uart_data", 1, posCallBack);

  // 送信データのアドレス登録.
  var::rewritePos_publisher = n.advertise<ros_urg::uart>("posData", 1);
  var::ransac_publisher =
      n.advertise<visualization_msgs::Marker>("groundRansac", 1);
  var::points_publisher =
      n.advertise<visualization_msgs::Marker>("groundPoints", 1);
  var::field_publisher = n.advertise<visualization_msgs::Marker>("ground", 1);
  var::particle_publisher =
      n.advertise<visualization_msgs::Marker>("particle", 1);

  //loop時間の設定
  ros::Rate loop_rate(100);

  //loop
  while (ros::ok())
  {
    ros::spinOnce();
    // loop_rate.sleep();
  }

  return 0;
};
