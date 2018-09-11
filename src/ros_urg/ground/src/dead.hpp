#ifndef SRC_DEAD_HPP_
#define SRC_DEAD_HPP_

#include <visualization_msgs/Marker.h>
#include <vector>

#include <cuda.h>
#include <curand_kernel.h>

#include "module.hpp"

class cDead
{
  private:
  public:
    void execuseRansac(std::vector<tUrg> &urg, tPos robotPos,
                       std::vector<tLine> &pushDataLine,
                       std::vector<tFrame> &pushDataUrg,
                       std::vector<double> &laserx,
                       std::vector<double> &lasery);

    double gapLength(int id, tLine ransac);

    void printField(visualization_msgs::Marker &line_list);

    void middle_num(tFrame &rewriteFrame);
};

#endif
