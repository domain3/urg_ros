#ifndef SRC_MODULE_HPP_
#define SRC_MODULE_HPP_

//���ʁE
typedef struct
{
  double x; // x���E
  double y; // y���E
} tFrame;

//urg�̒l�[��
typedef struct
{
  double length; //����
  double angle;  //�p�x
} tUrg;

//�����E
typedef struct
{
  // ax + by + c= 0.
  double a;
  double b;
  double c;
} tLine;

//�@�̂̎��Ȉʒu���W.
typedef struct
{
  double x;     // x���E
  double y;     // y���E
  double angle; //�p�x.
} tPos;

#endif
