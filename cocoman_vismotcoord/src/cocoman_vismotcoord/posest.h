#ifndef POSEST_H
#define POSEST_H

#include "model.h"
#include "scene.h"
#include "particlescoregpu.h"

#include <pcl/point_types.h>
#include <pcl/tracking/tracking.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include <pcl/filters/filter.h>
#include <pcl/common/time.h>
#include <functional>

#include <pcl/gpu/octree/octree.hpp>
#include <pcl/gpu/containers/device_array.hpp>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <algorithm>
#include <numeric>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

using namespace pcl;
using namespace Eigen;
using namespace std;

class PosEst
{
public:
    PosEst();
    ~PosEst();
    void setNumParticles(int numParticles_);
    void initPos(float x, float y, float z, float roll, float pitch, float yaw);
    void estimate(Scene *scene, Model *model, MatrixXf boundBox, float gridSize, int cnt);

    void getPos(Vector3f &org, Vector3f &rpy);
    vector<int> getCrspdFacePos(){return out_assScenePtrs;};

private:
    tracking::ParticleXYZRPY sampleWithVar(tracking::ParticleXYZRPY part, float varDist, float varAng);
    void initSample();
    void resample(float varDist, float varAng );
    void weight(MatrixXf boundBox, float gridSize);
    std::vector<float> weightGPU(MatrixXf boundBox, float gridSize);

private:
    Scene *scene;
    Model *model;
    tracking::ParticleXYZRPY pos_init;
    tracking::ParticleXYZRPY pos_updated;
    tracking::ParticleXYZRPY finalParticle;
    vector<tracking::ParticleXYZRPY> particles;
    int numParticles;
    float var_trans, var_rot; // dist in m, ang in degrees
    float motionratio;
    boost::mt19937 generator;
    float minDist, maxDist;
    std::vector<int> out_assScenePtrs;

    float eval;

};

#endif // POSEST_H
