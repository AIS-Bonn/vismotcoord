#include "posest.h"

PosEst::PosEst()
{
    //    var_trans = 0.001f;
    //    var_rot = 0.1f;

    var_trans = VARTRANS;
    var_rot = VARROT;
}

PosEst::~PosEst()
{

}

void PosEst::setNumParticles(int numParticles_)
{
    numParticles = numParticles_;
}

void PosEst::initPos(float x, float y, float z, float roll, float pitch, float yaw)
{
    pos_init.x = x;
    pos_init.y = y;
    pos_init.z = z;
    pos_init.roll = roll;
    pos_init.pitch = pitch;
    pos_init.yaw = yaw;
    pos_updated = pos_init;

    particles.resize(numParticles);
    initSample();
}

void PosEst::getPos(Vector3f &org, Vector3f &rpy)
{
    org[0] = pos_updated.x;
    org[1] = pos_updated.y;
    org[2] = pos_updated.z;
    rpy[0] = pos_updated.roll;
    rpy[1] = pos_updated.pitch;
    rpy[2] = pos_updated.yaw;
}

void PosEst::initSample()
{
    for (size_t i=0; i<numParticles; i++){
        tracking::ParticleXYZRPY sample;

        //        m_lastspeed.push_back(temp);
        //        m_bestspeed.push_back(temp);
        sample = sampleWithVar(pos_updated, var_trans, var_rot);
        sample.weight = 1.f/numParticles;


        //        sample.x = tracking::sampleNormal(pos_init.x, pow(fabs(m_vel.x)+0.003f, 2) );
        //        sample.y = tracking::sampleNormal(pos_init.y, pow(fabs(m_vel.y)+0.003f, 2) );
        //        sample.z = tracking::sampleNormal(pos_init.z, pow(fabs(m_vel.z)+0.003f, 2) );
        //        sample.roll  = tracking::sampleNormal(pos_init.roll,  pow(fabs(m_vel.roll)+0.5f/180.f*M_PI, 2) );
        //        sample.pitch = tracking::sampleNormal(pos_init.pitch, pow(fabs(m_vel.pitch)+0.5f/180.f*M_PI, 2) );
        //        sample.yaw   = tracking::sampleNormal(pos_init.yaw ,  pow(fabs(m_vel.yaw)+0.5f/180.f*M_PI, 2) );
        particles[i] = sample;
    }
}


tracking::ParticleXYZRPY PosEst::sampleWithVar(tracking::ParticleXYZRPY part, float varDist, float varAng)
{
    tracking::ParticleXYZRPY temp;
    temp.x = tracking::sampleNormal(part.x, pow(varDist, 2) );
    temp.y = tracking::sampleNormal(part.y, pow(varDist, 2) );
    temp.z = tracking::sampleNormal(part.z, pow(varDist, 2) );
    temp.roll  = tracking::sampleNormal(part.roll,  pow(varAng/180.f*M_PI, 2) );
    temp.pitch = tracking::sampleNormal(part.pitch, pow(varAng/180.f*M_PI, 2) );
    temp.yaw   = tracking::sampleNormal(part.yaw ,  pow(varAng/180.f*M_PI, 2) );

    temp.weight = 0.f;
    return temp;
}


void PosEst::estimate(Scene *scene_, Model *model_, MatrixXf boundBox, float gridSize, int cnt)
{
    if(DEBUG_ALGORITHM) cout<<cnt<<endl;
    model = model_;
    scene = scene_;

    double lastT = pcl::getTime();

    resample(var_trans, var_rot);
    if(DEBUG_ALGORITHM) cout<<"resample time : "<<pcl::getTime()-lastT<<endl;

    lastT = pcl::getTime();
    weight(boundBox, gridSize);
    if(DEBUG_ALGORITHM) cout<<"weight time : "<<pcl::getTime()-lastT<<endl;


    //    if (!(m_finalParticle.x==m_finalParticle.x && m_finalParticle.y==m_finalParticle.y && m_finalParticle.z == m_finalParticle.z &&
    //          m_finalParticle.roll == m_finalParticle.roll && m_finalParticle.pitch == m_finalParticle.pitch && m_finalParticle.yaw == m_finalParticle.yaw))
    //        m_finalParticle = m_lastFinalParticle;
    //    Eigen::Affine3f finaltrans = finalParticle.toEigenMatrix();
    if(DEBUG_ALGORITHM) cout<<"finaltrans is : " <<finalParticle<<endl;
    //    CloudPtr out (new Cloud);
    //    transformPointCloud(*m_modelOrig, *out, finaltrans);
    //    return out;

    // convert finalparticle to pos_updated

    pos_updated = finalParticle;

}


void PosEst::resample(float varDist, float varAng )
{
    double probabilities[numParticles];
    for (size_t i=0; i<numParticles; i++){
        probabilities[i] = particles[i].weight;
    }
    std::vector<double> cumulative;
    std::partial_sum(&probabilities[0], &probabilities[0] + numParticles,
            std::back_inserter(cumulative));

    //    std::lower_bound(cumulative.begin(), cumulative.end(), die()) - cumulative.begin();


    boost::uniform_real<> dist(0, cumulative.back());
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > die(generator, dist);
    std::vector<tracking::ParticleXYZRPY> templist;
    templist = particles;
    //    cout<<"die: ";
    for (size_t i=0; i<numParticles; i++){
        //        cout<<die()<<" ";
        int index = std::lower_bound(cumulative.begin(), cumulative.end(), die()) - cumulative.begin();

        templist[i] = sampleWithVar(particles[index], varDist, varAng); // dist in m, ang in degrees
        templist[i].weight = 1.f/numParticles;
        //        cout<<"resampled particle "<<templist[i]<<endl;

        //                tracking::ParticleXYZRPY temp;
        //                temp.x = -0.0071759;
        //                temp.y = 0.00560463;
        //                temp.z = -0.0248353;
        //                temp.roll  = -0.00428659;
        //                temp.pitch = -.000928194;
        //                temp.yaw   = 0.00286869;
        //                temp.weight = 0.f;

        //                templist[i] = temp;


        //        tracking::ParticleXYZRPY temp;
        //        temp.x = 0.0;
        //        temp.y = 0.0;
        //        temp.z = 0.0;
        //        temp.roll  = 0.0;
        //        temp.pitch = 0.0;
        //        temp.yaw   = 3.141592/2.;
        //        temp.weight = 0.f;

        //        templist[i] = temp;
    }
    particles = templist;

}


void PosEst::weight(MatrixXf boundBox, float gridSize)
{
    double lastT = pcl::getTime();
    //    CsFeature  m_modelF;
    //    csest.initBox(m_modelOrig);27696
    //    double lastT = pcl::getTime();
    //    m_modelF = csest.compute(m_modelOrig);


    std::vector<float> partweights = weightGPU(boundBox, gridSize);


    minDist = 1e10; maxDist=-1e10;

    for(size_t i=0; i<numParticles; i++){
        particles[i].weight = partweights[i];

        if (particles[i].weight > maxDist)
            maxDist = particles[i].weight;
        if (particles[i].weight < minDist)
            minDist = particles[i].weight;
    }

    ///////////////////////////////normarlize////////////////////////////////////
    float weightSum = 0.f;
    for (size_t i=0; i<numParticles; i++){
        float value;
        value= pow((particles[i].weight - minDist)/(maxDist-minDist), 1);
        //                m_particles[i].weight =1.f-value;

        particles[i].weight = exp(- 10.f * value);
        //                if (m_particles[i].weight<0.8f) {m_particles[i].weight=0.f;}
        if (minDist == 1e10){particles[i].weight=1.f;}
        weightSum += particles[i].weight;
    }
    float maxWeight = 0.;
    int maxPartID;

    //            cout<<"particle weights: [";

    for (size_t i=0; i<numParticles; i++){
        //                        cout<<particles[i].weight<<", ";
        particles[i].weight = particles[i].weight/weightSum;
        if(particles[i].weight > maxWeight){
            maxWeight = particles[i].weight;
            maxPartID = i;
        }

        //        particles[i].weight = 1;

    }
    //            cout<<" ] "<<endl;
    finalParticle.zero();

    //    Quaternion<float> finalq; finalq=  Quaternion<float>::Identity();
    for (size_t i=0; i<numParticles; i++){
        //        Quaternion<float> q;
        //        Eigen::AngleAxis<float> aaX(particles[i].roll, Eigen::Vector3f::UnitX());
        //        Eigen::AngleAxis<float> aaY(particles[i].pitch, Eigen::Vector3f::UnitY());
        //        Eigen::AngleAxis<float> aaZ(particles[i].yaw, Eigen::Vector3f::UnitZ());

        //        q = aaZ * aaY * aaX;
        //        finalq = Quaternion<float>(finalq.w()+q.w()*particles[i].weight, finalq.x()+q.x()*particles[i].weight,
        //                                   finalq.y()+q.y()*particles[i].weight, finalq.z()+q.z()*particles[i].weight);
        finalParticle = finalParticle + particles[i] * particles[i].weight;
    }
    if(DEBUG_ALGORITHM) cout<<"particle: "<<finalParticle<<endl;
    //finalParticle = finalParticle*(1.f/10000.f);
    //        finalParticle = particles[maxPartID];


    //    finalq = Quaternion<float>(finalq.w(), finalq.x(),
    //                               finalq.y(), finalq.z());
    //    finalq.normalize();
    //    Affine3f rot(finalq);

    //    pcl::getEulerAngles(rot, finalParticle.roll, finalParticle.pitch, finalParticle.yaw);


    //    if (!(finalParticle.x == finalParticle.x && m_finalParticle.y==m_finalParticle.y && m_finalParticle.z == m_finalParticle.z &&
    //          m_finalParticle.roll == m_finalParticle.roll && m_finalParticle.pitch == m_finalParticle.pitch && m_finalParticle.yaw == m_finalParticle.yaw))
    //        finalParticle = m_lastFinalParticle;

    //    finalParticle = particles[0];

    //    // evaluation: maximum weight value
    //    float max = 0;
    //    for(int i=0;i<partweights.size();i++){
    //        if(partweights.at(i) > max) max = partweights.at(i);
    //    }
    //    cout<<endl<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!eval: "<<max<<endl<<endl;
}

std::vector<float> PosEst::weightGPU(MatrixXf boundBox, float gridSize)
{
    int numSceneP, numModelF;
    numSceneP = scene->numPC();
    if(DEBUG_ALGORITHM) cout<<"# of all faces: "<<model->numFace()<<endl;
    if(DEBUG_ALGORITHM) cout<<"# of all filtered faces: "<<model->numFilteredFace()<<endl;
    numModelF = model->numFilteredFace();

    float3 maxBound, minBound;
    maxBound.x = boundBox(0,0);
    minBound.x = boundBox(0,1);
    maxBound.y = boundBox(1,0);
    minBound.y = boundBox(1,1);
    maxBound.z = boundBox(2,0);
    minBound.z = boundBox(2,1);

    ParticleScoreGPU scoreGPU(numParticles, numSceneP, numModelF, gridSize);
    *scoreGPU.minBound = minBound;
    *scoreGPU.maxBound = maxBound;

    // copy data
    if(DEBUG_ALGORITHM) cout<<"copy scene data: "<<numSceneP<<endl;
    for(int i=0;i<numSceneP;i++){
        scoreGPU.scenePos[i].x = scene->getPCFocus_back()->points.at(i).x;
        scoreGPU.scenePos[i].y = scene->getPCFocus_back()->points.at(i).y;
        scoreGPU.scenePos[i].z = scene->getPCFocus_back()->points.at(i).z;

        //        scoreGPU.sceneNorm[i].x = scene->getPCNorm()->points.at(i).x;
        //        scoreGPU.sceneNorm[i].y = scene->getPCNorm()->points.at(i).y;
        //        scoreGPU.sceneNorm[i].z = scene->getPCNorm()->points.at(i).z;
    }

    int nFace = 0;
    int nVert = 0;
    if(DEBUG_ALGORITHM) cout<<"copy model data"<<endl;
    for(int n=0;n<model->filteredParts.size();n++){
        BodyPart *part = model->bodyPart(model->filteredParts.at(n));
        Mesh* mesh = part->meshFocused();
        for(int i=0;i<mesh->numMesh();i++){
            for(int j=0;j<mesh->numFilteredFaces(i);j++){
                int faceId = mesh->filteredFaceId(i,j);
                //                Vector3f faceNorm = mesh->faceNorm(i,faceId);
                for(int k=0;k<3;k++){
                    scoreGPU.modelVertPos[nVert].x = mesh->vertex(i,mesh->face(i,faceId)->mIndices[k])->m_pos[0];
                    scoreGPU.modelVertPos[nVert].y = mesh->vertex(i,mesh->face(i,faceId)->mIndices[k])->m_pos[1];
                    scoreGPU.modelVertPos[nVert].z = mesh->vertex(i,mesh->face(i,faceId)->mIndices[k])->m_pos[2];
                    if(USENORM){
                        scoreGPU.modelVertNorm[nVert].x = mesh->vertex(i,mesh->face(i,faceId)->mIndices[k])->m_normal[0];
                        scoreGPU.modelVertNorm[nVert].y = mesh->vertex(i,mesh->face(i,faceId)->mIndices[k])->m_normal[1];
                        scoreGPU.modelVertNorm[nVert].z = mesh->vertex(i,mesh->face(i,faceId)->mIndices[k])->m_normal[2];
                    }
                    nVert ++;
                }
                //                scoreGPU.modelFaceNorm[nFace].x = faceNorm[0];
                //                scoreGPU.modelFaceNorm[nFace].y = faceNorm[1];
                //                scoreGPU.modelFaceNorm[nFace].z = faceNorm[2];
                nFace ++;
            }
        }
    }

    for(int i=0;i<numParticles;i++){
        scoreGPU.particlePos[i].x = particles[i].x;
        scoreGPU.particlePos[i].y = particles[i].y;
        scoreGPU.particlePos[i].z = particles[i].z;
        scoreGPU.particleRPY[i].x = particles[i].roll;
        scoreGPU.particleRPY[i].y = particles[i].pitch;
        scoreGPU.particleRPY[i].z = particles[i].yaw;
    }

    float3 *crspdFacesPos = (float3 *)malloc(numSceneP * sizeof(float3));
    if(DEBUG_ALGORITHM) cout<<"particle filtering"<<endl;
    std::vector<float> out = scoreGPU.compute(crspdFacesPos);

    //    out_crspdFacesPos.resize(numSceneP);
    //    for (size_t i=0; i<numSceneP; i++){
    //        out_crspdFacesPos[i] = crspdFacesPos[i];
    //    }

    free(crspdFacesPos);

    //    crspdFacesPos = scoreGPU.compute();
    return (out);
}
/*
std::vector<float> PosEst::weightGPU(float gridsize, int xnr, int ynr, int znr,
                              Vector4f min, Vector4f max, std::vector<float> refhist)
{
    CSGPU gpuest(gridsize, xnr, ynr, znr, m_particlenum, m_scene->points.size());
//    gpuest.cloudSize = cloud->points.size();
    gpuest.minPt.x=min[0];
    gpuest.minPt.y=min[1];
    gpuest.minPt.z=min[2];
    gpuest.maxPt.x=max[0];
    gpuest.maxPt.y=max[1];
    gpuest.maxPt.z=max[2];
    float h,s,v;
    for (size_t i=0; i<m_scene->points.size(); i++){
        PointT pt = m_scene->points[i];
        gpuest.cloudpos[i].x = pt.x;
        gpuest.cloudpos[i].y = pt.y;
        gpuest.cloudpos[i].z = pt.z;
        rgbTohsv(pt.r, pt.g, pt.b, h, s, v);
        gpuest.cloudhsv[i].x = h;
        gpuest.cloudhsv[i].y = s;
        gpuest.cloudhsv[i].z = v;
    }
//    for (size_t i=0; i<m_model->points.size(); i++){
//        PointT pt = m_model->points[i];
//        gpuest.cloudpos[i].x = pt.x;
//        gpuest.cloudpos[i].y = pt.y;
//        gpuest.cloudpos[i].z = pt.z;
//        rgbTohsv(pt.r, pt.g, pt.b, h, s, v);
//        gpuest.cloudhsv[i].x = h;
//        gpuest.cloudhsv[i].y = s;
//        gpuest.cloudhsv[i].z = v;
//    }

    for (size_t i=0; i<m_particlenum; i++){
        Affine3f temp = m_particles[i].toEigenMatrix().inverse();
        float x_,y_,z_,roll_,pitch_,yaw_;
        pcl::getTranslationAndEulerAngles (temp,x_,y_,z_,roll_,pitch_,yaw_);
        gpuest.partpos[i].x = x_;
        gpuest.partpos[i].y = y_;
        gpuest.partpos[i].z = z_;
        gpuest.partrot[i].x = roll_;
        gpuest.partrot[i].y = pitch_;
        gpuest.partrot[i].z = yaw_;
    }
//    for (size_t i=0; i<m_particlenum; i++){
//        gpuest.partpos[i].x = m_particles[i].x;
//        gpuest.partpos[i].y = m_particles[i].y;
//        gpuest.partpos[i].z = m_particles[i].z;
//        gpuest.partrot[i].x = m_particles[i].roll;
//        gpuest.partrot[i].y = m_particles[i].pitch;
//        gpuest.partrot[i].z = m_particles[i].yaw;
//    }
    for (size_t i=0; i<refhist.size(); i++){
        gpuest.refhist[i] = refhist[i];
    }
    return (gpuest.compute());
}

*/
