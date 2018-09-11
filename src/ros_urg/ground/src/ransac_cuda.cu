#include <stdio.h>
#include <iostream>
#include <vector>

#include <ros/ros.h>

#include <thrust/device_vector.h>
#include <thrust/host_vector.h>

#include "ransac_cuda.hpp"

/**
 * @brief cudaにて直線検知を行う
 * 
 * @param urg             urgから得た点群データ
 * @param robotPos        機体の自己位置
 * @param warp            ブロックごとのthread間隔
 * @param fieldLine       fieldの木枠の直線データ
 * @param fieldFrame      木枠の角データ
 * @param pushDataLine    検知した直線データを格納
 * @param pushDataLineId  検知した直線データの種類
 * @param pushUrgData     x-y座標に直したurg点群データ
 */
void excuse_ransac_cuda(std::vector<tUrg> &urg, tPos robotPos, double warp,
                        std::vector<tLine> &fieldLine,
                        std::vector<tFrame> &fieldFrame,
                        std::vector<tLine> &pushDataLine,
                        std::vector<int> &pushDataLineId,
                        std::vector<tFrame> &pushUrgData)
{
  // if there in not enough points call return.
  if (urg.size() < 300)
  {
    std::cerr << "Error : urgFrame.size" << std::endl;
    return;
  }

  const int diffThresh = 60; //(int)((double)urg.size() / 5);   //鬪値.
  const int ransacMax = 720; //試行回数.

  // device上のメモリ定義.
  thrust::device_vector<tUrg> device_urg_data(urg.size());

  thrust::host_vector<tFrame> host_Frame_data(urg.size());
  thrust::device_vector<tFrame> device_Frame_data(urg.size());
  thrust::device_vector<int> device_Frame_id(urg.size());

  thrust::device_vector<tLine> device_field_data(fieldLine.size());
  thrust::device_vector<tFrame> device_field_frame(fieldFrame.size());

  // copy data host to  device.
  cudaMemcpy(thrust::raw_pointer_cast(device_urg_data.data()), urg.data(),
             urg.size() * sizeof(tFrame), cudaMemcpyHostToDevice);

  // copy data host to  device.
  cudaMemcpy(thrust::raw_pointer_cast(device_field_data.data()),
             fieldLine.data(), fieldLine.size() * sizeof(tLine),
             cudaMemcpyHostToDevice);

  // copy data host to  device.
  cudaMemcpy(thrust::raw_pointer_cast(device_field_frame.data()),
             fieldFrame.data(), fieldFrame.size() * sizeof(tFrame),
             cudaMemcpyHostToDevice);

  // detect area for program.
  dim3 blocks_0((int)urg.size() / warp + 1, 1, 1);
  dim3 threads_0((int)warp, 1, 1);

  // changr urg data
  urg_kernel<<<blocks_0, threads_0>>>(
      thrust::raw_pointer_cast(device_urg_data.data()), urg.size(),
      thrust::raw_pointer_cast(device_field_data.data()),
      device_field_data.size(),
      thrust::raw_pointer_cast(device_field_frame.data()), robotPos,
      thrust::raw_pointer_cast(device_Frame_data.data()),
      thrust::raw_pointer_cast(device_Frame_id.data()));
  　
　 //コピー
      cudaMemcpy(thrust::raw_pointer_cast(host_Frame_data.data()),
                 thrust::raw_pointer_cast(device_Frame_data.data()),
                 device_Frame_data.size() * sizeof(tFrame), cudaMemcpyDeviceToHost);

  for (int i = 0; i < urg.size(); i++)
    pushUrgData.push_back(host_Frame_data[i]);

  // gpuの領域設定.
  dim3 blocks_1((int)ransacMax / warp + 1, 1, 1);
  dim3 threads_1((int)warp, 1, 1);

  // ransacのデータのメモリ定義.
  thrust::host_vector<tLine> host_line_data(ransacMax);
  thrust::device_vector<tLine> device_line_data(ransacMax);
  thrust::host_vector<int> host_nearPoints_data(ransacMax);
  thrust::device_vector<int> device_nearPoints_data(ransacMax);
  thrust::host_vector<int> host_line_id_data(ransacMax);
  thrust::device_vector<int> device_line_id_data(ransacMax);

  // ransac.
  ransac_kernel<<<blocks_1, threads_1>>>(
      thrust::raw_pointer_cast(device_Frame_data.data()),
      thrust::raw_pointer_cast(device_Frame_id.data()),
      device_Frame_data.size(),
      thrust::raw_pointer_cast(device_line_data.data()),
      thrust::raw_pointer_cast(device_line_id_data.data()),
      thrust::raw_pointer_cast(device_nearPoints_data.data()), ransacMax);

  //コピー
  cudaMemcpy(thrust::raw_pointer_cast(host_line_data.data()),
             thrust::raw_pointer_cast(device_line_data.data()),
             device_line_data.size() * sizeof(tLine), cudaMemcpyDeviceToHost);

  cudaMemcpy(thrust::raw_pointer_cast(host_nearPoints_data.data()),
             thrust::raw_pointer_cast(device_nearPoints_data.data()),
             device_nearPoints_data.size() * sizeof(int),
             cudaMemcpyDeviceToHost);

  cudaMemcpy(thrust::raw_pointer_cast(host_line_id_data.data()),
             thrust::raw_pointer_cast(device_line_id_data.data()),
             device_line_id_data.size() * sizeof(int), cudaMemcpyDeviceToHost);

  //一番精度のいい直線を検出
  int max_diff[fieldLine.size()];
  int max_id[fieldLine.size()];
  for (int i = 0; i < fieldLine.size(); i++)
  {
    max_diff[i] = 0;
    max_id[i] = -1;
  }

  for (int i = 0; i < ransacMax; i++)
  {
    int tgtLineId = host_line_id_data[i];
    if (tgtLineId < 0)
      continue;
    if (host_nearPoints_data[i] > max_diff[tgtLineId])
    {
      max_diff[tgtLineId] = host_nearPoints_data[i];
      max_id[tgtLineId] = i;
    }
  }

  for (int i = 0; i < fieldLine.size(); i++)
  {
    if (max_diff[i] < diffThresh)
      continue;

    pushDataLine.push_back(host_line_data[max_id[i]]);
    pushDataLineId.push_back(i);
  }
}

/**
 * @brief 点群(length,angke)をfieldのx-y座標に変換
 * 
 * @param urg           urgから得た点群データ
 * @param urgNum 　　　　点群の数
 * @param fieldLine     fieldの直線
 * @param fieldLineNum  fieldの直線の数
 * @param fieldFrame    fieldの角のデータ
 * @param robotPos      機体の自己位置
 * @param out           x-y平面に直したもの
 * @param frame_id      点群に一番近い木枠のid
 */
__global__ void urg_kernel(tUrg *urg, int urgNum, tLine *fieldLine,
                           int fieldLineNum, tFrame *fieldFrame, tPos robotPos,
                           tFrame *out, int *frame_id)
{
  //threadのid
  int id = blockIdx.x * blockDim.x + threadIdx.x;

  //idは32の倍数にするのであまりが出る
  if (id > urgNum)
    return;

  //urgデータを機体中心座標に変換
  double x, y;
  x = -urg[id].length * sin(urg[id].angle);
  y = -urg[id].length * cos(urg[id].angle) - 370;

  //x = urg[id].length * cos(-urg[id].angle) + 370;
  //y = urg[id].length * sin(-urg[id].angle);

  //printf("%d, %lf, %lf\n", id, y, robotPos.angle);

  //機体中心座標からfield座標に変換
  out[id].x = x * cos(robotPos.angle) - y * sin(robotPos.angle) + robotPos.x;
  out[id].y = x * sin(robotPos.angle) + y * cos(robotPos.angle) + robotPos.y;

  //点群に一番近い木枠を決める
  double minLengthThresh = 400; //鬪値.
  int out_id = -1;
  for (int i = 0; i < fieldLineNum; i++)
  {
    double length =
        fabs(fieldLine[i].a * out[id].x + fieldLine[i].b * out[id].y +
             fieldLine[i].c) /
        sqrt(fieldLine[i].a * fieldLine[i].a + fieldLine[i].b * fieldLine[i].b);

    if (out[id].x < fieldFrame[i * 2].x - minLengthThresh ||
        out[id].x > fieldFrame[i * 2 + 1].x + minLengthThresh)
      continue;
    if (out[id].y < fieldFrame[i * 2].y - minLengthThresh ||
        out[id].y > fieldFrame[i * 2 + 1].y + minLengthThresh)
      continue;

    if (length < minLengthThresh)
    {
      minLengthThresh = length;
      out_id = i;
    }
  }
  frame_id[id] = out_id;
}

/**
 * @brief ransac(直線検知を実行)
 * 
 * @param urg             urgの点群データ
 * @param urg_id          点群がどこの木枠に所属してるか計算
 * @param urgNum          点群の数
 * @param line            直線のパラメータ
 * @param nearPointsId    直線がどの木枠に所属してるか
 * @param nearPointsNum 　直線に近い点群の数
 * @param ransacMax       ransacの施行回数
 */
__global__ void ransac_kernel(tFrame *urg, int *urg_id, int urgNum, tLine *line,
                              int *nearPointsId, int *nearPointsNum,
                              int ransacMax)
{
  // idを計算.
  int id = blockIdx.x * blockDim.x + threadIdx.x;
  if (id > ransacMax)
    return;

  // cuda版rand関数
  curandState_t state;
  curand_init(clock(), id, 0, &state);

  // ransacに必要な変数.
  const double lengthThresh = 20; //鬪値.
  const int sampleNum = 2;        // sample数.

  // randamな位置を取得.
  unsigned int randam[sampleNum]; // randamな値.
  tFrame randamPoints[sampleNum]; // ramdamな位置.
  int tgtLineId = -1;

  for (int i = 0; i < sampleNum; i++)
  {
    // randamな値を入手.
    randam[i] = curand(&state) % urgNum;

    randamPoints[i].x = urg[randam[i]].x;
    randamPoints[i].y = urg[randam[i]].y;
  }

  if (urg_id[randam[0]] == -1 || urg_id[randam[1]] == -1)
  {
    nearPointsId[id] = -1;
    nearPointsNum[id] = 0;
    return;
  }

  if (urg_id[randam[0]] != urg_id[randam[1]])
  {
    nearPointsId[id] = -1;
    nearPointsNum[id] = 0;
    return;
  }

  tgtLineId = urg_id[randam[0]];

  // ax + by + c = 0;(2点の場合).
  //※３点の場合最小２乗法
  tLine randamLine;
  randamLine.a = randamPoints[1].y - randamPoints[0].y;
  randamLine.b = -(randamPoints[1].x - randamPoints[0].x);
  randamLine.c = -randamPoints[1].y * randamPoints[0].x +
                 randamPoints[1].x * randamPoints[0].y;

  //最小２乗法で直線計算(ax + by + c = 0).
  //double xySum = 0, xSum = 0, ySum = 0, xPowSum = 0;
  int near = 0;
  for (int i = 0; i < urgNum; i++)
  {
    //直線と点の距離を計算.
    double length =
        fabs(randamLine.a * urg[i].x + randamLine.b * urg[i].y + randamLine.c) /
        sqrt(randamLine.a * randamLine.a + randamLine.b * randamLine.b);
    if (length < lengthThresh)
    {
      //直線と点の距離を計算し鬪値以内ならpointを足す.
      near++;

      //最小２乗法.
      /*
      xySum += urg[i].x * urg[i].y / 1000000;
      xSum += urg[i].x / 1000;
      ySum += urg[i].y / 1000;
      xPowSum += urg[i].x * urg[i].x / 1000000;
      */
    }
  }

  //最小２乗法.
  /*
  tLine leastLine;
  leastLine.a = near * xySum - xSum * ySum;
  leastLine.b = xSum * xSum - near * xPowSum;
  leastLine.c = xPowSum * ySum - xySum * xSum;
  if (leastLine.a > leastLine.b) {
    leastLine.b /= leastLine.a;
    leastLine.c /= leastLine.a / 1000;
    leastLine.a = 1;
  }
  else {
    leastLine.a /= leastLine.b;
    leastLine.c /= leastLine.b / 1000;
    leastLine.b = 1;
  }*/

  //line[id] = leastLine;
  line[id] = randamLine;
  nearPointsNum[id] = near;
  nearPointsId[id] = tgtLineId;
}
