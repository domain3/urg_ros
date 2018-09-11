#include <vector>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>

#include "module.hpp"

typedef struct tParticle
{
public:
  double x;
  double y;
  double weight;
  tParticle(
      double x,
      double y,
      double weight)
      : x(x), y(y), weight(weight){};
} tParticle;

class cParticleFilter
{
private:
  const double comDeviX;
  const double comDeviY;

  const double laserDeviX;
  const double laserDeviY;

  std::vector<tParticle> particle;

public:
  cParticleFilter(
      double comDeviX,
      double comDeviY,
      double laserDeviX,
      double laserDeviY);

  ~cParticleFilter()
  {
    particle.clear();
  }

  void init(tPos comPos);
  void updateWeights(std::vector<double> &x,
                                    std::vector<double> &y) ;
  void resample();

  tFrame getPos();

  void pushRviz(visualization_msgs::Marker &particle_list);
};

