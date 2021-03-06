#ifndef SRC_MODULE_HPP_
#define SRC_MODULE_HPP_

//½ÊE
typedef struct
{
  double x; // x²E
  double y; // y²E
} tFrame;

//urgÌl[ü
typedef struct
{
  double length; //£
  double angle;  //px
} tUrg;

//¼üE
typedef struct
{
  // ax + by + c= 0.
  double a;
  double b;
  double c;
} tLine;

//@ÌÌ©ÈÊuÀW.
typedef struct
{
  double x;     // x²E
  double y;     // y²E
  double angle; //px.
} tPos;

#endif
