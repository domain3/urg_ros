#include <math.h>
#include <ros/ros.h>
#include <iostream>
#include <vector>

#include "dead.hpp"
#include "ransac_cuda.hpp"

namespace
{
namespace param
{

//fieldの検知する木枠の数
const int lineNum = 9;

//機体とurgの距離
const double robotX = 370;

//木枠の直線パラメータ
tLine fieldLine[lineNum] = {{0, 1, 0}, {1, 0, 0}, {0, 1, -5390}, {0, 1, -2160}, {1, 0, -10150}, {1, 0, -7030}, {1, 0, -12120}, {1, 0, -1500}, {1, 0, -14070}};

//木枠の角データ
tFrame fieldFrame1[lineNum] = {{0, 0}, {0, 0}, {0, 5390}, {7030, 2160}, {10150, 2160}, {7030, 2160}, {12120, 2160}, {1500, 0}, {14070, 0}};
tFrame fieldFrame2[lineNum] = {{14070, 0}, {0, 5390}, {7030, 5390}, {14070, 2160}, {10150, 5390}, {7030, 5390}, {12120, 5390}, {1500, 1000}, {14070, 5390}};
} // namespace param
} // namespace

/**
 * @brief ransacから直線を検知し誤差を検知する
 * 
 * @param urg           urgから得た点群データ
 * @param robotPos      機体の位置
 * @param pushDataLine  直線データを格納する
 * @param pushDataUrg   点群(x-y)を格納
 * @param laserx        直線から得た機体の自己位置候補(x軸)
 * @param lasery        直線から得た機体の自己位置候補(y軸)
 */
void cDead::execuseRansac(std::vector<tUrg> &urg, tPos robotPos,
                          std::vector<tLine> &pushDataLine,
                          std::vector<tFrame> &pushDataUrg,
                          std::vector<double> &laserx,
                          std::vector<double> &lasery)
{

  //木枠の直線データの格納
  std::vector<tLine> field_line;
  for (int i = 0; i < param::lineNum; i++)
    field_line.push_back(param::fieldLine[i]);

  //木枠の角データを格納
  std::vector<tFrame> field_frame;
  for (int i = 0; i < param::lineNum; i++)
  {
    field_frame.push_back(param::fieldFrame1[i]);
    field_frame.push_back(param::fieldFrame2[i]);
  }

  //直線のidを格納
  std::vector<int> line_id;

  //urgの点群をx-y座標に直しransac(直線検知)を行う
  excuse_ransac_cuda(urg, robotPos, 32, field_line, field_frame, pushDataLine,
                     line_id, pushDataUrg);

  //直線データから機体の自己位置候補を計算
  for (int i = 0; i < pushDataLine.size(); i++)
  {
    //機体と直線データとの距離
    double length = fabs(pushDataLine[i].a * robotPos.x +
                         pushDataLine[i].b * robotPos.y + pushDataLine[i].c) /
                    sqrt(pushDataLine[i].a * pushDataLine[i].a +
                         pushDataLine[i].b * pushDataLine[i].b);

    //機体と木枠との距離
    double fieldLength =
        fabs(param::fieldLine[line_id[i]].a * robotPos.x +
             param::fieldLine[line_id[i]].b * robotPos.y +
             param::fieldLine[line_id[i]].c) /
        sqrt(param::fieldLine[line_id[i]].a * param::fieldLine[line_id[i]].a +
             param::fieldLine[line_id[i]].b * param::fieldLine[line_id[i]].b);

    //直線から機体の自己位置候補を算出
    if (line_id[i] == 0)
    {
      double y = robotPos.y + (length - fieldLength);
      lasery.push_back(y);
    }
    else if (line_id[i] == 2 || line_id[i] == 3)
    {
      if (line_id[i] == 2 && robotPos.x > 6000)
        continue;
      if (line_id[i] == 3 && robotPos.x > 7000)
        continue;
      double y = robotPos.y - (length - fieldLength);
      lasery.push_back(y);
    }
    else if (line_id[i] == 1 /*|| line_id[i] == 4*/)
    {
      double x = robotPos.x + (length - fieldLength);
      laserx.push_back(x);
    }
    else if (line_id[i] == 8 || line_id[i] == 5)
    {
      double x = robotPos.x - (length - fieldLength);
      laserx.push_back(x);
    }
  }
}

/**
 * @brief 出力するデータを過去5回分の中央値にする
 * 
 * @param rewriteFrame 算出した機体の自己位置データ(中央値に変更する)
 */
void cDead::middle_num(tFrame &rewriteFrame)
{
  //過去五回分のデータ
  static std::vector<double> oldx(5, rewriteFrame.x);
  static std::vector<double> oldy(5, rewriteFrame.y);

  //データの格納
  for (int i = 4; i > 0; i--)
  {
    oldx[i] = oldx[i - 1];
    oldy[i] = oldy[i - 1];
  }
  oldx[0] = rewriteFrame.x;
  oldy[0] = rewriteFrame.y;

  //sort用vecotr
  std::vector<double> sortx = oldx;
  std::vector<double> sorty = oldy;

  //データを大きい順に
  std::sort(std::begin(sortx), std::end(sortx));
  std::sort(std::begin(sorty), std::end(sorty));

  //中央値を算出
  rewriteFrame.x = sortx[2];
  rewriteFrame.y = sorty[2];
}

/**
 * @brief rvizに木枠データを送る
 * 
 * @param line_list lineデータを格納する
 */
void cDead::printField(visualization_msgs::Marker &line_list)
{
  for (int i = 0; i < param::lineNum; i++)
  {
    geometry_msgs::Point line_point;
    line_point.x = param::fieldFrame1[i].x / 1000;
    line_point.y = param::fieldFrame1[i].y / 1000;
    line_point.z = 0;
    line_list.points.push_back(line_point);

    line_point.x = param::fieldFrame2[i].x / 1000;
    line_point.y = param::fieldFrame2[i].y / 1000;
    line_point.z = 0;
    line_list.points.push_back(line_point);
    line_list.lifetime = ros::Duration();
  }
}
