#ifndef PARTICLESCOREGPU_H
#define PARTICLESCOREGPU_H

#include "define.h"

#include <vector>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

using namespace Eigen;

class ParticleScoreGPU
{
public:
    ParticleScoreGPU(){}
    ParticleScoreGPU(int numParticles_, int numSceneP_, int numModelF_, float gridSize_);
    ~ParticleScoreGPU();
    std::vector<float> compute(float3 *crspdFacesPos);

    // input memory
    int numParticles;
    int numSceneP;
    int numModelF;
    float3 *maxBound;
    float3 *minBound;
    float gridSize;
    int numGrid;
    int numGridX;
    int numGridY;
    int numGridZ;

    float3 *scenePos;
    float3 *sceneNorm;
    float3 *modelVertPos;    // 3 times than # of faces. x,y,z order
    float3 *modelVertNorm;
    float3 *particlePos;
    float3 *particleRPY;
    float3 *modelGridMapPos;
    float3 *modelGridMapNorm;
    int *modelGridMapNVert;

    // output memory
    float *weights;
    int *nCrsp;
//    float3 *crspdFacesPos;

};

#endif // PARTICLESCOREGPU_H
