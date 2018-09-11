#ifndef SRC_MODULE_HPP_
#define SRC_MODULE_HPP_

//平面・
typedef struct
{
  double x; // x軸・
  double y; // y軸・
} tFrame;

//urgの値納入
typedef struct
{
  double length; //距離
  double angle;  //角度
} tUrg;

//直線・
typedef struct
{
  // ax + by + c= 0.
  double a;
  double b;
  double c;
} tLine;

//機体の自己位置座標.
typedef struct
{
  double x;     // x軸・
  double y;     // y軸・
  double angle; //角度.
} tPos;

#endif
