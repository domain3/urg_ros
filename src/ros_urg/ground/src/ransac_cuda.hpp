
#include <vector>

#include <cuda.h>
#include <curand_kernel.h>

#include "module.hpp"

extern void excuse_ransac_cuda(
    std::vector<tUrg> &urg,
    tPos robotPos,
    double warp,
    std::vector<tLine> &fieldLine,
    std::vector<tFrame> &fieldFrame,
    std::vector<tLine> &pushDataLine,
    std::vector<int> &pushDataLineId,
    std::vector<tFrame> &pushUrgData);


__global__ void urg_kernel(
    tUrg *urg,
    int urgNum,
    tLine *fieldLine,
    int fieldLineNum,
    tFrame *fieldFrame,
    tPos robotPos,
    tFrame *out,
    int *frame_id);

__global__ void ransac_kernel(
    tFrame *urg,
    int *urg_id,
    int urgNum,
    tLine *line,
    int *nearPointsId,
    int *nearPointsNum,
    int ransacMax);
