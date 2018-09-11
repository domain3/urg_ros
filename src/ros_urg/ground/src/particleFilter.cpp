#include <iostream>
#include <random>
#include <vector>

#include "module.hpp"
#include "particleFilter.hpp"

/**
 * @brief Construct a new c Particle Filter::c Particle Filter object
 * 
 * @param comDeviX   マイコンデータの自己位置のガウス分布(x軸)
 * @param comDeviY   マイコンデータの自己位置のガウス分布(y軸)
 * @param laserDeviX urgデータのガウス分布(x軸)
 * @param laserDeviY urgデータのガウス分布(y軸)
 */
cParticleFilter::cParticleFilter(double comDeviX, double comDeviY,
                                 double laserDeviX, double laserDeviY)
    : comDeviX(comDeviX),
      comDeviY(comDeviY),
      laserDeviX(laserDeviX),
      laserDeviY(laserDeviY){};

/**
 * @brief パーティクル拡散
 * 
 * @param comPos 機体の自己位置座標
 */
void cParticleFilter::init(tPos comPos)
{
  //パーティクルの数
  const int particleNum = 100;

  //randam関数
  std::default_random_engine engine;

  //分散に基づいたrandam設定
  std::normal_distribution<double> dist_x(comPos.x, comDeviX);
  std::normal_distribution<double> dist_y(comPos.y, comDeviY);

  //パーティクル拡散
  for (int i = 0; i < particleNum; i++)
  {
    tParticle p = tParticle(dist_x(engine), dist_y(engine), 1);
    particle.push_back(p);
    // std::cerr << p.x << "," << p.y << std::endl;
  }
}

/**　
 * @brief 重さの計算
 * 
 * @param x urgによる自己位置候補(x軸)
 * @param y urgによる自己位置候補(y軸)
 */
void cParticleFilter::updateWeights(std::vector<double> &x,
                                    std::vector<double> &y)
{
  //重さの合計
  double weights_sum = 0;

  // std::cerr << x.size() << "," << y.size() << std::endl;

  //x軸の重さの再計算
  for (int i = 0; i < x.size(); i++)
  {
    for (int j = 0; j < particle.size(); j++)
    {
      double dx = particle[j].x - x[i];
      // std::cerr << particle[i].x << "," << x[i] << std::endl;
      // std::cerr << dx << std::endl;
      double num = exp(-dx * dx / (2 * laserDeviX * laserDeviX));
      double denom = 2 * M_PI * laserDeviX * laserDeviX;

      particle[j].weight *= num / denom;
    }
  }

  //y軸の重さの再計算
  for (int i = 0; i < y.size(); i++)
  {
    for (int j = 0; j < particle.size(); j++)
    {
      double dy = particle[j].y - y[i];
      double num = exp(-dy * dy / (2 * laserDeviY * laserDeviY));
      double denom = 2 * M_PI * laserDeviY * laserDeviY;

      particle[j].weight *= num / denom;
    }
  }

  //重さの合計を計算
  for (int i = 0; i < particle.size(); i++)
  {
    weights_sum += particle[i].weight;
  }

  //重さの合計をparticle.size()にする
  for (int i = 0; i < particle.size(); i++)
  {
    particle[i].weight /= (weights_sum / particle.size());
    // std::cerr << particle[i].weight << std::endl;
  }

  //重さの合計を計算
  double sum = 0;
  for (int i = 0; i < particle.size(); i++)
  {
    sum += particle[i].weight;
  }
}

/**
 * @brief 自己位置の算出
 * 
 * @return tFrame 機体の自己位置
 */
tFrame cParticleFilter::getPos()
{
  //x軸の重さの合計
  double weight_sum_x = 0;
  //y軸の重さの合計
  double weight_sum_y = 0;

  //重さと位置をかける
  for (int i = 0; i < particle.size(); i++)
  {
    weight_sum_x += particle[i].weight * particle[i].x;
    weight_sum_y += particle[i].weight * particle[i].y;
  }

  //重さの平均化
  tFrame pushFrame;
  pushFrame.x = weight_sum_x / particle.size();
  pushFrame.y = weight_sum_y / particle.size();

  return pushFrame;
}

/**
 * @brief rvizにパーティクルを送信
 * 
 * @param particle_list パーティクルデータを格納するリスト
 */
void cParticleFilter::pushRviz(visualization_msgs::Marker &particle_list)
{
  for (int i = 0; i < particle.size(); i++)
  {
    geometry_msgs::Point points;
    points.x = particle[i].x / 1000;
    points.y = particle[i].y / 1000;
    points.z = particle[i].weight;
    particle_list.points.push_back(points);
    particle_list.lifetime = ros::Duration();
  }
}

/**
 * @brief パーティクルのクリア
 * 
 */
void cParticleFilter::resample() { particle.clear(); }
