#include "particlescoregpu.h"

#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <iostream>
#include "stdio.h"
using namespace std;

ParticleScoreGPU::ParticleScoreGPU(int numParticles_, int numSceneP_, int numModelF_, float gridSize_){
    numParticles = numParticles_;
    numSceneP = numSceneP_;
    numModelF = numModelF_;
    gridSize = gridSize_;

    scenePos = (float3 *)malloc(numSceneP * sizeof(float3));
    sceneNorm = (float3 *)malloc(numSceneP * sizeof(float3));
    modelVertPos = (float3 *)malloc(numModelF * 3 * sizeof(float3));
    modelVertNorm = (float3 *)malloc(numModelF * 3 * sizeof(float3));

    particlePos = (float3 *)malloc(numParticles * sizeof(float3));
    particleRPY = (float3 *)malloc(numParticles * sizeof(float3));

    weights = (float *)malloc(numParticles * sizeof(float));
    nCrsp = (int *)malloc(numParticles * sizeof(int));
    //    crspdFacesPos = (float3 *)malloc(numSceneP * sizeof(float3));

    minBound = (float3 *)malloc(sizeof(float3));
    maxBound = (float3 *)malloc(sizeof(float3));


}

ParticleScoreGPU::~ParticleScoreGPU()
{

}


__global__ void run(int numParticles, int numSceneP, int numModelF,
                    float3 *minBound, float3 *maxBound, float gridSize, int numGridX, int numGridY, int numGridZ,
                    float3 *scenePos, float3 *sceneNorm, float3 *modelVertPos,
                    float3 *particlePos, float3 *particleRPY, float3 *crspdFacesPos, float *weights, int *nCrsp,
                    float3 *modelGridMapPos, float3 *modelGridMapNorm, int *modelGridMapNVert)
{
    int n =  blockIdx.x;
    int pId = threadIdx.x;
    if (n+1>numParticles)  {return;}

    // transformation matrix
    float px = particlePos[n].x;
    float py = particlePos[n].y;
    float pz = particlePos[n].z;
    float roll = particleRPY[n].x;
    float pitch = particleRPY[n].y;
    float yaw = particleRPY[n].z;
    float t[4][4];

    float A = cosf (yaw), B = sinf (yaw), C = cosf (pitch), D = sinf (pitch),
            E = cosf (roll), F = sinf (roll), DE = D*E, DF = D*F;
    t[0][0] = A*C; t[0][1] = A*DF - B*E; t[0][2] = B*F + A*DE; t[0][3] = px;
    t[1][0] = B*C; t[1][1] = A*E + B*DF; t[1][2] = B*DE - A*F; t[1][3] = py;
    t[2][0] = -D;  t[2][1] = C*F;        t[2][2] = C*E;        t[2][3] = pz;
    t[3][0] = 0.f; t[3][1] = 0.f;        t[3][2] = 0.f;        t[3][3] = 1.f;

    float likelihood = 0.;
    int nrPerThr = (int)ceil( ((float)numSceneP) / ((float)blockDim.x) );
    // for each scene point
    for(int i=nrPerThr*pId;i<min(nrPerThr*(pId+1),numSceneP);i++){
        float3 sceneP = scenePos[i];
        float3 sceneN = sceneNorm[i];

        // transform pos and norm
        float3 transPos, transNorm;
        transPos.x = t[0][0]*sceneP.x + t[0][1]*sceneP.y + t[0][2]*sceneP.z + t[0][3];
        transPos.y = t[1][0]*sceneP.x + t[1][1]*sceneP.y + t[1][2]*sceneP.z + t[1][3];
        transPos.z = t[2][0]*sceneP.x + t[2][1]*sceneP.y + t[2][2]*sceneP.z + t[2][3];
        transNorm.x = t[0][0]*sceneN.x + t[0][1]*sceneN.y + t[0][2]*sceneN.z + t[0][3];
        transNorm.y = t[1][0]*sceneN.x + t[1][1]*sceneN.y + t[1][2]*sceneN.z + t[1][3];
        transNorm.z = t[2][0]*sceneN.x + t[2][1]*sceneN.y + t[2][2]*sceneN.z + t[2][3];

        //        transPos = sceneP;
        /*
        // find the closest face and show the correspondence?
        float min = 0.;
        int argmin = 0;
        for(int j=0;j<numModelF;j++){
            float dist = 0.;
            for(int k=0;k<3;k++){
                dist += sqrt(pow((transPos.x - modelVertPos[j*3+k].x),2)
                        + pow((transPos.y - modelVertPos[j*3+k].y),2)
                        + pow((transPos.z - modelVertPos[j*3+k].z),2));
            }
            dist /= 3.;
            if(j == 0){
                min = dist;
                argmin = j;
            }
            else{
                if(dist < min){
                    min = dist;
                    argmin = j;
                }
            }
        }
        //            cout<<argmin<<" ";
        crspdFacesPos[i].x = (modelVertPos[argmin*3+0].x + modelVertPos[argmin*3+1].x + modelVertPos[argmin*3+2].x) / 3.;
        crspdFacesPos[i].y = (modelVertPos[argmin*3+0].y + modelVertPos[argmin*3+1].y + modelVertPos[argmin*3+2].y) / 3.;
        crspdFacesPos[i].z = (modelVertPos[argmin*3+0].z + modelVertPos[argmin*3+1].z + modelVertPos[argmin*3+2].z) / 3.;
*/
        float likelihood_point = 0.;
        float dist_error;
        // find index of transPos in the gridmap
        int idx_x = (transPos.x-minBound->x)/gridSize;
        int idx_y = (transPos.y-minBound->y)/gridSize;
        int idx_z = (transPos.z-minBound->z)/gridSize;

        if(idx_x < 0 || idx_x >= numGridX || idx_y < 0 || idx_y >= numGridY ||idx_z < 0 || idx_z >= numGridZ){
            //            crspdFacesPos[i].x = 0;
            //            crspdFacesPos[i].y = 0;
            //            crspdFacesPos[i].z = 0;
            dist_error = GRIDSIZE;
        }
        else{
            int idx = (idx_x * numGridY + idx_y) * numGridZ + idx_z;
            if(modelGridMapNVert[idx] != 0){
                atomicAdd(&nCrsp[n], 1);
                //                crspdFacesPos[i].x = modelGridMapPos[idx].x;
                //                crspdFacesPos[i].y = modelGridMapPos[idx].y;
                //                crspdFacesPos[i].z = modelGridMapPos[idx].z;
                if(!USENORM)
                    dist_error = pow(transPos.x-modelGridMapPos[idx].x,2) + pow(transPos.y-modelGridMapPos[idx].y,2) + pow(transPos.z-modelGridMapPos[idx].z,2);
//                        dist_error = 0;
                else
                    dist_error = pow((transPos.x-modelGridMapPos[idx].x)*modelGridMapNorm[idx].x + (transPos.y-modelGridMapPos[idx].y)*modelGridMapNorm[idx].y + (transPos.z-modelGridMapPos[idx].z)*modelGridMapNorm[idx].z,2);
//                    dist_error = 0;
                //                float norm1 = sqrt(pow(modelGridMapNorm[idx].x,2)+pow(modelGridMapNorm[idx].y,2)+pow(modelGridMapNorm[idx].z,2));
                //                float norm2 = sqrt(pow(transNorm.x,2)+pow(transNorm.y,2)+pow(transNorm.z,2));
                //                float dot = modelGridMapNorm[idx].x*transNorm.x + modelGridMapNorm[idx].y*transNorm.y + modelGridMapNorm[idx].z*transNorm.z;
                //                dist_angle = acos(dot/(norm1*norm2)) / 3.141592;

            }
            else{
                //                crspdFacesPos[i].x = 0;
                //                crspdFacesPos[i].y = 0;
                //                crspdFacesPos[i].z = 0;
                dist_error = GRIDSIZE;
            }
        }
        likelihood_point = (1. * 1 * dist_error);
//        if(USENORM) likelihood_point += (1. * 1 * dist_angle);
        likelihood += likelihood_point;
    }

    // calculate weight
    atomicAdd(&weights[n], likelihood);

}

std::vector<float> ParticleScoreGPU::compute(float3 *crspdFacesPos)
{
    numGridX = (maxBound->x - minBound->x) / gridSize;
    numGridY = (maxBound->y - minBound->y) / gridSize;
    numGridZ = (maxBound->z - minBound->z) / gridSize;

    modelGridMapPos = (float3 *)malloc(numGridX*numGridY*numGridZ * sizeof(float3));
    modelGridMapNorm = (float3 *)malloc(numGridX*numGridY*numGridZ * sizeof(float3));
    modelGridMapNVert = (int *)malloc(numGridX*numGridY*numGridZ * sizeof(int));
    memset(modelGridMapPos, 0.f, numGridX*numGridY*numGridZ*sizeof(float3));
    memset(modelGridMapNorm, 0.f, numGridX*numGridY*numGridZ*sizeof(float3));
    memset(modelGridMapNVert, 0, numGridX*numGridY*numGridZ*sizeof(int));

    // memory set on the device
    float *d_weights;
    cudaMalloc((void **) &d_weights, numParticles*sizeof(float));
    cudaMemset(d_weights, 0.f, numParticles*sizeof(float));

    int *d_nCrsp;
    cudaMalloc((void **) &d_nCrsp, numParticles*sizeof(int));
    cudaMemset(d_nCrsp, 0, numParticles*sizeof(int));

    float3 *d_crspdFacesPos;
    cudaMalloc((void **) &d_crspdFacesPos, numSceneP*sizeof(float3));
    cudaMemset(d_crspdFacesPos, 0.f, numSceneP*sizeof(float3));

    // input
    float3 *d_scenePos, *d_sceneNorm, *d_modelVertPos, *d_particlePos, *d_particleRPY, *d_modelGridMapPos, *d_modelGridMapNorm;
    float3 *d_minBound, *d_maxBound;

    int *d_modelGridMapNVert;
    cudaMalloc((void **) &d_scenePos, numSceneP*sizeof(float3));
    cudaMalloc((void **) &d_sceneNorm, numSceneP*sizeof(float3));
    cudaMalloc((void **) &d_modelVertPos, numModelF * 3 *sizeof(float3));
    cudaMalloc((void **) &d_particlePos, numParticles*sizeof(float3));
    cudaMalloc((void **) &d_particleRPY, numParticles*sizeof(float3));
    cudaMalloc((void **) &d_modelGridMapPos, numGridX*numGridY*numGridZ*sizeof(float3));
    cudaMalloc((void **) &d_modelGridMapNorm, numGridX*numGridY*numGridZ*sizeof(float3));
    cudaMalloc((void **) &d_modelGridMapNVert, numGridX*numGridY*numGridZ*sizeof(int));
    cudaMalloc((void **) &d_minBound, sizeof(float3));
    cudaMalloc((void **) &d_maxBound, sizeof(float3));

    cudaMemset(d_modelGridMapPos, 0.f, numGridX*numGridY*numGridZ*sizeof(float3));
    cudaMemset(d_modelGridMapNorm, 0.f, numGridX*numGridY*numGridZ*sizeof(float3));
    cudaMemset(d_modelGridMapNVert, 0, numGridX*numGridY*numGridZ*sizeof(int));

    // compute grid space
    //    modelGridMapNVert[3200688] ++;
//    cout<<"# gridsize: "<<numGridX*numGridY*numGridZ<<endl;

    // for each face
    for(int i=0;i<numModelF;i++){
        // make a boundary
        float3 min = modelVertPos[i*3+0];
        float3 max = modelVertPos[i*3+0];
        for(int k=1;k<3;k++){
            if(modelVertPos[i*3+k].x < min.x) min.x = modelVertPos[i*3+k].x;
            if(modelVertPos[i*3+k].x > max.x) max.x = modelVertPos[i*3+k].x;
            if(modelVertPos[i*3+k].y < min.y) min.y = modelVertPos[i*3+k].y;
            if(modelVertPos[i*3+k].y > max.y) max.y = modelVertPos[i*3+k].y;
            if(modelVertPos[i*3+k].z < min.z) min.z = modelVertPos[i*3+k].z;
            if(modelVertPos[i*3+k].z > max.z) max.z = modelVertPos[i*3+k].z;
        }
        //convert minmax to idxes of grid
        int idx_min_x = (min.x-minBound->x)/gridSize;
        int idx_max_x = (max.x-minBound->x)/gridSize;
        int idx_min_y = (min.y-minBound->y)/gridSize;
        int idx_max_y = (max.y-minBound->y)/gridSize;
        int idx_min_z = (min.z-minBound->z)/gridSize;
        int idx_max_z = (max.z-minBound->z)/gridSize;

        //        cout<<idx_min_x<<" "<<idx_max_x<<" "<<idx_min_y<<" "<<idx_max_y<<" "<<idx_min_z<<" "<<idx_max_z<<endl;

        if(idx_max_x >= numGridX) idx_max_x = numGridX-1;
        if(idx_max_y >= numGridY) idx_max_y = numGridY-1;
        if(idx_max_z >= numGridZ) idx_max_z = numGridZ-1;
        if(idx_min_x < 0) idx_min_x = 0;
        if(idx_min_y < 0) idx_min_y = 0;
        if(idx_min_z < 0) idx_min_z = 0;

        // find a grid to fill
        for(int idx_x = idx_min_x; idx_x <= idx_max_x; idx_x ++)
            for(int idx_y = idx_min_y; idx_y <= idx_max_y; idx_y ++)
                for(int idx_z = idx_min_z; idx_z <= idx_max_z; idx_z ++){
                    int idx = (idx_x * numGridY + idx_y) * numGridZ + idx_z;
                    if(idx >= numGridX*numGridY*numGridZ) cout<<"!!!!!!!!!!!!!!!!!!!!!";

                    // find the closest vertex of three
                    float3 gridPos;
                    gridPos.x = minBound->x + idx_x * gridSize + gridSize/2.;
                    gridPos.y = minBound->y + idx_y * gridSize + gridSize/2.;
                    gridPos.z = minBound->z + idx_z * gridSize + gridSize/2.;

                    float distMin = sqrt(pow(gridPos.x-modelVertPos[i*3+0].x,2)+pow(gridPos.y-modelVertPos[i*3+0].y,2)+pow(gridPos.z-modelVertPos[i*3+0].z,2));
                    int minVert = 0;

                    for(int k=1;k<3;k++){
                        float dist = sqrt(pow(gridPos.x-modelVertPos[i*3+k].x,2)+pow(gridPos.y-modelVertPos[i*3+k].y,2)+pow(gridPos.z-modelVertPos[i*3+k].z,2));
                        if(dist < distMin) {
                            distMin = dist;
                            minVert = k;
                        }
                    }
                    //                    cout<<idx<<" ";
                    if(modelGridMapNVert[idx] == 0){
                        modelGridMapPos[idx] = modelVertPos[i*3+minVert];
                        if(USENORM) modelGridMapNorm[idx] = modelVertNorm[i*3+minVert];
                    }
                    else{
                        modelGridMapPos[idx].x = (modelGridMapPos[idx].x*modelGridMapNVert[idx] + modelVertPos[i*3+minVert].x) / (modelGridMapNVert[idx]+1.);
                        modelGridMapPos[idx].y = (modelGridMapPos[idx].y*modelGridMapNVert[idx] + modelVertPos[i*3+minVert].y) / (modelGridMapNVert[idx]+1.);
                        modelGridMapPos[idx].z = (modelGridMapPos[idx].z*modelGridMapNVert[idx] + modelVertPos[i*3+minVert].z) / (modelGridMapNVert[idx]+1.);

                        if(USENORM){
                            modelGridMapNorm[idx].x = (modelGridMapNorm[idx].x*modelGridMapNVert[idx] + modelVertNorm[i*3+minVert].x) / (modelGridMapNVert[idx]+1.);
                            modelGridMapNorm[idx].y = (modelGridMapNorm[idx].y*modelGridMapNVert[idx] + modelVertNorm[i*3+minVert].y) / (modelGridMapNVert[idx]+1.);
                            modelGridMapNorm[idx].z = (modelGridMapNorm[idx].z*modelGridMapNVert[idx] + modelVertNorm[i*3+minVert].z) / (modelGridMapNVert[idx]+1.);
                        }
                    }

                    modelGridMapNVert[idx] ++;

                }

        // for each grid
        // update normalvec and # of vert for the selected grid

    }

    // memory copy from host to device
    cudaMemcpy(d_scenePos, scenePos, numSceneP*sizeof(float3), cudaMemcpyHostToDevice);
    cudaMemcpy(d_sceneNorm, sceneNorm, numSceneP*sizeof(float3), cudaMemcpyHostToDevice);
    cudaMemcpy(d_modelVertPos, modelVertPos, numModelF * 3 *sizeof(float3), cudaMemcpyHostToDevice);
    cudaMemcpy(d_particlePos, particlePos, numParticles*sizeof(float3), cudaMemcpyHostToDevice);
    cudaMemcpy(d_particleRPY, particleRPY, numParticles*sizeof(float3), cudaMemcpyHostToDevice);
    cudaMemcpy(d_modelGridMapPos, modelGridMapPos, numGridX*numGridY*numGridZ*sizeof(float3), cudaMemcpyHostToDevice);
    cudaMemcpy(d_modelGridMapNorm, modelGridMapNorm, numGridX*numGridY*numGridZ*sizeof(float3), cudaMemcpyHostToDevice);
    cudaMemcpy(d_modelGridMapNVert, modelGridMapNVert, numGridX*numGridY*numGridZ*sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(d_minBound, minBound, sizeof(float3), cudaMemcpyHostToDevice);
    cudaMemcpy(d_maxBound, maxBound, sizeof(float3), cudaMemcpyHostToDevice);


    // computation time
    cudaEvent_t start, stop;
    float time;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);
    cudaEventRecord(start, 0);

    /// computation on gpu
//    cout<<minBound->x<<" "<<minBound->y<<" "<<minBound->z<<endl;
    run<<<numParticles, 1024>>>(numParticles, numSceneP, numModelF, d_minBound, d_maxBound, gridSize, numGridX, numGridY, numGridZ,
                                d_scenePos, d_sceneNorm, d_modelVertPos, d_particlePos, d_particleRPY, d_crspdFacesPos, d_weights, d_nCrsp,
                                d_modelGridMapPos, d_modelGridMapNorm, d_modelGridMapNVert);

    /// computation on cpu

    //    // for each particle
    //    for(int n=0;n<numParticles;n++){
    //        cout<<"# particles: "<<n<<endl;
    //        // transformation matrix
    //        float px = particlePos[n].x;
    //        float py = particlePos[n].y;
    //        float pz = particlePos[n].z;
    //        float roll = particleRPY[n].x;
    //        float pitch = particleRPY[n].y;
    //        float yaw = particleRPY[n].z;
    //        float t[4][4];

    //        float A = cosf (yaw), B = sinf (yaw), C = cosf (pitch), D = sinf (pitch),
    //                E = cosf (roll), F = sinf (roll), DE = D*E, DF = D*F;
    //        t[0][0] = A*C; t[0][1] = A*DF - B*E; t[0][2] = B*F + A*DE; t[0][3] = px;
    //        t[1][0] = B*C; t[1][1] = A*E + B*DF; t[1][2] = B*DE - A*F; t[1][3] = py;
    //        t[2][0] = -D;  t[2][1] = C*F;        t[2][2] = C*E;        t[2][3] = pz;
    //        t[3][0] = 0.f; t[3][1] = 0.f;        t[3][2] = 0.f;        t[3][3] = 1.f;

    //        // for each scene point
    //        for(int i=0;i<numSceneP;i++){
    //            float3 sceneP = scenePos[i];
    //            float3 sceneN = sceneNorm[i];

    //            // transform pos and norm
    //            float3 transPos, transNorm;
    //            transPos.x = t[0][0]*sceneP.x + t[0][1]*sceneP.y + t[0][2]*sceneP.z + t[0][3];
    //            transPos.y = t[1][0]*sceneP.x + t[1][1]*sceneP.y + t[1][2]*sceneP.z + t[1][3];
    //            transPos.z = t[2][0]*sceneP.x + t[2][1]*sceneP.y + t[2][2]*sceneP.z + t[2][3];
    //            transNorm.x = t[0][0]*sceneN.x + t[0][1]*sceneN.y + t[0][2]*sceneN.z + t[0][3];
    //            transNorm.y = t[1][0]*sceneN.x + t[1][1]*sceneN.y + t[1][2]*sceneN.z + t[1][3];
    //            transNorm.z = t[2][0]*sceneN.x + t[2][1]*sceneN.y + t[2][2]*sceneN.z + t[2][3];

    //            // find the closest face and show the correspondence?
    //            float min = 0.;
    //            int argmin;
    //            for(int j=0;j<numModelF;j++){
    //                float dist = 0.;
    //                for(int k=0;k<3;k++){
    //                    dist += sqrt(pow((transPos.x - modelVertPos[j*3+k].x),2)
    //                            + pow((transPos.y - modelVertPos[j*3+k].y),2)
    //                            + pow((transPos.z - modelVertPos[j*3+k].z),2));
    //                }
    //                dist /= 3.;
    //                if(j == 0){
    //                    min = dist;
    //                    argmin = j;
    //                }
    //                else{
    //                    if(dist < min){
    //                        min = dist;
    //                        argmin = j;
    //                    }
    //                }
    //            }
    //            //            cout<<argmin<<" ";
    //            crspdFacesPos[i].x = (modelVertPos[argmin*3+0].x + modelVertPos[argmin*3+1].x + modelVertPos[argmin*3+2].x) / 3.;
    //            crspdFacesPos[i].y = (modelVertPos[argmin*3+0].y + modelVertPos[argmin*3+1].y + modelVertPos[argmin*3+2].y) / 3.;
    //            crspdFacesPos[i].z = (modelVertPos[argmin*3+0].z + modelVertPos[argmin*3+1].z + modelVertPos[argmin*3+2].z) / 3.;
    //        }
    //    }


    //    // end computation
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&time, start, stop);
    if(DEBUG_ALGORITHM) cout<<"Time for the kernel: "<<time<<" ms"<<endl;
    cudaEventDestroy(start);
    cudaEventDestroy(stop);

    // memory copy from device to host
    cudaMemcpy(weights, d_weights, numParticles*sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(nCrsp, d_nCrsp, numParticles*sizeof(int), cudaMemcpyDeviceToHost);
    cudaMemcpy(crspdFacesPos, d_crspdFacesPos, numSceneP*sizeof(float3), cudaMemcpyDeviceToHost);

    std::vector<float> out;
    out.resize(numParticles);
    for (size_t i=0; i<numParticles; i++){
//        out[i] = weights[i] / (float)nCrsp[i];
           out[i] = weights[i];
    }
    if(DEBUG_ALGORITHM) cout<<"min weight: ";
    float min;
    for(int i=0;i<out.size();i++){
        if(i==0) min = out[i];
        else if(out[i] < min) min = out[i];
    }
    if(DEBUG_ALGORITHM) cout<<min<<endl;

    // release
    cudaFree(d_weights);
    cudaFree(d_nCrsp);
    cudaFree(d_crspdFacesPos);
    cudaFree(d_scenePos);
    cudaFree(d_sceneNorm);
    cudaFree(d_modelVertPos);
    cudaFree(d_particlePos);
    cudaFree(d_particleRPY);
    cudaFree(d_modelGridMapPos);
    cudaFree(d_modelGridMapNorm);
    cudaFree(d_modelGridMapNVert);
    cudaFree(d_minBound);
    cudaFree(d_maxBound);

    free(scenePos);
    free(sceneNorm);
    free(modelVertPos);
    free(particlePos);
    free(particleRPY);
    free(weights);
    free(nCrsp);
    //    free(crspdFacesPos);
    free(modelGridMapPos);
    free(modelGridMapNorm);
    free(modelGridMapNVert);
    free(minBound);
    free(maxBound);

    return out;
}
