#include "vismotcoord.h"

VisMotCoord::VisMotCoord(tf::TransformListener *tf_listener_, string baseFrame_, string camFrame_init_, string camFrame_updated_)
    :tf_listener(tf_listener_), baseFrame(baseFrame_), camFrame_init(camFrame_init_), camFrame_updated(camFrame_updated_)
{
    cnt = 0;
    tf_listener->waitForTransform(baseFrame, camFrame_init, ros::Time(0), ros::Duration(3.0));
    tf_listener->lookupTransform(baseFrame, camFrame_init, ros::Time(0), camPos_init);

    // given initial cam pos
    cross_rot.setValue(0.0617378,  -0.845461,   0.530457,
                      -0.993293,  -0.104102, -0.0503159,
                      0.0977617,  -0.523793,  -0.846217);
    cross_org.setValue(0.264621, 0.121787, 0.797234);

    stretch_rot.setValue(0.0979697,  -0.843423,   0.528242,
                        -0.995167, -0.0865961,  0.0463028,
                        0.00669084,  -0.530225,  -0.847831);
    stretch_org.setValue(0.285899, 0.0438362, 0.750304);

    upward_rot.setValue(0.0710548,  -0.860826,   0.503914,
                        -0.997294, -0.0708611,  0.0195738,
                        0.0188583,  -0.503941,  -0.863532);
    upward_org.setValue(0.294009, 0.0531354, 0.776613);

    // for manual initialization ---------------------------------
//    cam_experiment.setBasis(upward_rot);
//    cam_experiment.setOrigin(upward_org);

//    transformAsMatrix(cam_experiment, eigen_camFromBase_exp);
//    camPos_init = cam_experiment;
    // -----------------------------------------------------------

    transformAsMatrix(camPos_init, eigen_camFromOrg_updated);
    camPos_updated = camPos_init;

    double roll, pitch, yaw;
    tf::Quaternion q(camPos_init.getRotation().getX(), camPos_init.getRotation().getY(), camPos_init.getRotation().getZ(), camPos_init.getRotation().getW());
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    posest.setNumParticles(NPARTICLE);
    //    posest.initPos(camPos_init.getOrigin().x(), camPos_init.getOrigin().y(), camPos_init.getOrigin().z(), roll, pitch, yaw);
    posest.initPos(0.0, 0., 0., 0., 0., 0.);


    filterFrameNames.push_back("/right_lower_forearm");
    filterFrameNames.push_back("/right_upper_forearm");

    isCamUpdate = 0;

    eigen_virtualFromFocus_old << 1, 0, 0, 0,
                                  0, 1, 0, 0,
                                  0, 0, 1, 0,
                                  0, 0, 0, 1;
}

bool VisMotCoord::setBodyModel(const vector<string> &vecPartFrame, const vector<string> &vecPartPath)
{
    if(vecPartFrame.size() != vecPartPath.size()){
        if(DEBUG_ALGORITHM) cout<<"vecPartFrame and vecPartPath should have same size!";
        return 0;
    }
    for(int i=0;i<vecPartPath.size();i++){
        model.setPart(vecPartFrame.at(i), vecPartPath.at(i));
    }
    return true;
}

void VisMotCoord::bodyTransform()
{
    for(int i=0;i<model.numPart();i++){
        tf::StampedTransform transform;
        // get T_i|o
        tf_listener->waitForTransform(baseFrame, model.partFrame(i), ros::Time(0), ros::Duration(3.0));
        tf_listener->lookupTransform(baseFrame, model.partFrame(i), ros::Time(0), transform);
        Eigen::Matrix4f eigen_transform;
        transformAsMatrix (transform, eigen_transform);
        model.transformPart(i, eigen_transform);
    }
}

void VisMotCoord::bodyFiltering()
{
    Vector3f camz;
    Eigen::Matrix4f eigen_transform;
    transformAsMatrix (camPos_updated, eigen_transform);
    camz[0] = eigen_transform(0,2);
    camz[1] = eigen_transform(1,2);
    camz[2] = eigen_transform(2,2);
    model.numFilteredFaces = 0;
    for(int i=0;i<model.numPart();i++){
        model.filteringPart(i, camz);
    }
}

void VisMotCoord::bodyTransformSet(string focusFrame_)
{
    model.numFilteredFaces = 0;
    focusFrame = focusFrame_;
    model.filteredParts.clear();
    vector<tf::StampedTransform> transforms;
    for(int i=0;i<filterFrameNames.size();i++){
        tf::StampedTransform transform;
        tf_listener->waitForTransform(focusFrame, filterFrameNames.at(i), ros::Time(0), ros::Duration(3.0));
        tf_listener->lookupTransform(focusFrame, filterFrameNames.at(i), ros::Time(0), transform);
        transforms.push_back(transform);
    }

    // transform
    for(int i=0;i<model.numPart();i++){
        for(int j=0;j<filterFrameNames.size();j++){
            if(model.partFrame(i) == filterFrameNames.at(j)){
                // get T_i|f
                tf::StampedTransform transform = transforms.at(j);
                Eigen::Matrix4f eigen_transform;
                transformAsMatrix (transform, eigen_transform);
                model.transformPartSet(i, eigen_transform);
            }
        }
    }
}

void VisMotCoord::setBoundBox(float margin)
{

    gridspace = model.getBoundBox();
    if(DEBUG_ALGORITHM) cout<<gridspace<<endl;

    boundBox.resize(3,2);
    boundBox << gridspace(0,0) + margin, gridspace(0,1)-margin,
           gridspace(1,0) + margin, gridspace(1,1)-margin,
           gridspace(2,0) + margin, gridspace(2,1)-margin;
}

void VisMotCoord::bodyTransformOrg()
{
    // transform back to org
    tf::StampedTransform transform;
    tf_listener->waitForTransform(baseFrame, focusFrame, ros::Time(0), ros::Duration(3.0));
    tf_listener->lookupTransform(baseFrame, focusFrame, ros::Time(0), transform);
    // get T_f|b
    Eigen::Matrix4f eigen_transform;
    transformAsMatrix (transform, eigen_transform);
    for(int i=0;i<model.numPart();i++){
        for(int j=0;j<filterFrameNames.size();j++){
            if(model.partFrame(i) == filterFrameNames.at(j)){
                model.transformPartOrg(i, eigen_transform);
            }
        }
    }
}

//void VisMotCoord::setModel()
//{
//    model.clear();
//    for(int n=0;n<vecBodyPart.size();n++){
//        BodyPart *part = bodyPart(n);
//        Mesh* mesh = part->meshTransformed();
//        model.setPart(mesh);

//    }
//}

void VisMotCoord::setRefFrames(vector<string> &refFrames, float boundMargin)
{
    int n = refFrames.size();
    VectorXf x(n);
    VectorXf y(n);
    VectorXf z(n);
    for(int i=0;i<n;i++){
//        for(int j=0;j<filterFrameNames.size();j++){
//            if(refFrames.at(i) == filterFrameNames.at(j)){
                tf::StampedTransform transform;
                tf_listener->waitForTransform(baseFrame, refFrames.at(i), ros::Time(0), ros::Duration(5.0));
                tf_listener->lookupTransform(baseFrame, refFrames.at(i), ros::Time(0), transform);

                x[i] = transform.getOrigin().x();
                y[i] = transform.getOrigin().y();
                z[i] = transform.getOrigin().z();
//            }
//        }
    }

    // minmax
    boundBox.resize(3,2);
    boundBox << x.maxCoeff()+boundMargin, x.minCoeff()-boundMargin,
            y.maxCoeff()+boundMargin, y.minCoeff()-boundMargin,
            z.maxCoeff()+boundMargin, z.minCoeff()-boundMargin;
    if(DEBUG_ALGORITHM) cout<<"boundBox"<<endl;
    if(DEBUG_ALGORITHM) cout<<boundBox<<endl;
}

void VisMotCoord::getPC(CloudPtr &pc_out)
{
    pc_out.reset(new Cloud);
    *pc_out = *scene.getPC();
}

void VisMotCoord::getPCFocus(CloudPtr &pc_out)
{
    pc_out.reset(new Cloud);
    *pc_out = *scene.getPCFocus();
}


void VisMotCoord::getPCScene(CloudPtr &pc_out)
{
    pc_out.reset(new Cloud);
    *pc_out = *scene.getPCScene();
}

void VisMotCoord::updateScene(CloudPtr &pc_in, CloudPtr &pc_updated)
{
//    pc_updated.reset(new Cloud);
//    pcl_ros::transformPointCloud(*pc_in, *pc_updated, camPos_updated);

    // get T_o|f
    tf_listener->waitForTransform(focusFrame, baseFrame, ros::Time(0), ros::Duration(3.0));
    tf_listener->lookupTransform(focusFrame, baseFrame, ros::Time(0), T_baseFromFocus);




    scene.focusSceneUpdate(pc_in, camPos_updated, T_baseFromFocus, boundBox, eigen_virtualFromFocus_old);

//    scene.initScene(pc_in, pc_updated, boundBox);


    Eigen::Matrix4f eigen_camFromBase;
    transformAsMatrix (T_baseFromFocus, eigen_baseFromFocus);

    transformAsMatrix (camPos_updated, eigen_camFromBase);
    eigen_camFromFocus = eigen_baseFromFocus * eigen_camFromBase;

    eigen_baseFromFocus_old = eigen_baseFromFocus;

}

void VisMotCoord::setScene(CloudPtr &pc_in, ros::Time currentT)
{
    // output cutted original pointcloud
    // cut using boundBox
    // set


    // transform pc_in from previous camera pos
    CloudPtr pc_transformed;
    pc_transformed.reset(new Cloud);

    tf_listener->waitForTransform(focusFrame, camFrame_init, currentT, ros::Duration(5.0));
    tf_listener->lookupTransform(focusFrame, camFrame_init, ros::Time(0), T_camFromFocus);
    transformAsMatrix (T_camFromFocus, eigen_camFromFocus);

    tf_listener->waitForTransform(focusFrame, baseFrame, ros::Time(0), ros::Duration(3.0));
    tf_listener->lookupTransform(focusFrame, baseFrame, ros::Time(0), T_baseFromFocus);
    transformAsMatrix (T_baseFromFocus, eigen_baseFromFocus);
    eigen_baseFromFocus_old = eigen_baseFromFocus;

    // for manual initialization -------------------------------------------------------------------------------------
//    tf_listener->waitForTransform(focusFrame, baseFrame, currentT, ros::Duration(5.0));
//    tf_listener->lookupTransform(focusFrame, baseFrame, ros::Time(0), T_baseFromFocus);
//    transformAsMatrix (T_baseFromFocus, eigen_baseFromFocus);
//    eigen_camFromFocus = eigen_baseFromFocus * eigen_camFromBase_exp;
//    T_camFromFocus.setBasis(tf::Matrix3x3(eigen_camFromFocus(0,0), eigen_camFromFocus(0,1), eigen_camFromFocus(0,2),
//                                          eigen_camFromFocus(1,0), eigen_camFromFocus(1,1), eigen_camFromFocus(1,2),
//                                          eigen_camFromFocus(2,0), eigen_camFromFocus(2,1), eigen_camFromFocus(2,2)));
//    T_camFromFocus.setOrigin(tf::Vector3(eigen_camFromFocus(0,3), eigen_camFromFocus(1,3), eigen_camFromFocus(2,3)));
    // ----------------------------------------------------------------------------------------------------------------

    pcl_ros::transformPointCloud(*pc_in, *pc_transformed, T_camFromFocus);


    scene.initScene(pc_in, pc_transformed, boundBox, eigen_virtualFromFocus_old);
    //    scene.transform



}


void VisMotCoord::setScene2(CloudPtr &pc_in, ros::Time currentT)
{
    // output cutted original pointcloud
    // cut using boundBox
    // set

    // transform pc_in from previous camera pos
    CloudPtr pc_transformed;
    pc_transformed.reset(new Cloud);

    tf_listener->waitForTransform(focusFrame, camFrame_init, currentT, ros::Duration(5.0));
    tf_listener->lookupTransform(focusFrame, camFrame_init, ros::Time(0), T_camFromFocus);
    transformAsMatrix (T_camFromFocus, eigen_camFromFocus);

    tf_listener->waitForTransform(focusFrame, baseFrame, ros::Time(0), ros::Duration(3.0));
    tf_listener->lookupTransform(focusFrame, baseFrame, ros::Time(0), T_baseFromFocus);
    transformAsMatrix (T_baseFromFocus, eigen_baseFromFocus);
    eigen_baseFromFocus_old = eigen_baseFromFocus;

    // for manual initialization -------------------------------------------------------------------------------------
    tf_listener->waitForTransform(focusFrame, baseFrame, currentT, ros::Duration(5.0));
    tf_listener->lookupTransform(focusFrame, baseFrame, ros::Time(0), T_baseFromFocus);
    transformAsMatrix (T_baseFromFocus, eigen_baseFromFocus);

    eigen_camFromFocus = eigen_baseFromFocus * eigen_camFromOrg_updated;
    T_camFromFocus.setBasis(tf::Matrix3x3(eigen_camFromFocus(0,0), eigen_camFromFocus(0,1), eigen_camFromFocus(0,2),
                                          eigen_camFromFocus(1,0), eigen_camFromFocus(1,1), eigen_camFromFocus(1,2),
                                          eigen_camFromFocus(2,0), eigen_camFromFocus(2,1), eigen_camFromFocus(2,2)));
    T_camFromFocus.setOrigin(tf::Vector3(eigen_camFromFocus(0,3), eigen_camFromFocus(1,3), eigen_camFromFocus(2,3)));
    // ----------------------------------------------------------------------------------------------------------------

    pcl_ros::transformPointCloud(*pc_in, *pc_transformed, T_camFromFocus);


    scene.initScene(pc_in, pc_transformed, boundBox, eigen_virtualFromFocus_old);
    //    scene.transform



}


void VisMotCoord::focusBody(string focusFrame_){
    focusFrame = focusFrame_;

    // get T_o|f
    tf_listener->waitForTransform(focusFrame, baseFrame, ros::Time(0), ros::Duration(3.0));
    tf_listener->lookupTransform(focusFrame, baseFrame, ros::Time(0), T_baseFromFocus);

    transformAsMatrix (T_baseFromFocus, eigen_baseFromFocus);


    for(int i=0;i<model.numPart();i++){
        model.focusPart(i, eigen_baseFromFocus);
    }
}

void VisMotCoord::focusScene(ros::Time currentT)
{
    // get T_c|f
    tf_listener->waitForTransform(focusFrame, camFrame_init, ros::Time(0), ros::Duration(5.0));
    tf_listener->lookupTransform(focusFrame, camFrame_init, ros::Time(0), T_camFromFocus);

    transformAsMatrix (T_camFromFocus, eigen_camFromFocus);

    scene.focusScene(T_camFromFocus);
}

void VisMotCoord::coordinate()
{
    if(DEBUG_ALGORITHM) cout<<"gridspace"<<endl;
    if(DEBUG_ALGORITHM) cout<<gridspace<<endl;

    // grid space from the model
    posest.estimate(&scene, &model, gridspace, GRIDSIZE, cnt);

    // output
    Vector3f org, rpy;
    posest.getPos(org, rpy);

    tf::Vector3 o;
    o[0] = org[0];
    o[1] = org[1];
    o[2] = org[2];
    tf::Quaternion q;
    tf::Matrix3x3 mat;
    mat.setEulerYPR(rpy[2], rpy[1], rpy[0]);
    mat.getRotation(q);

    virtualPos_updated.setOrigin(o);
    virtualPos_updated.setRotation(q);




    Eigen::Matrix4f eigen_virtualFromFocus_updated;
    float t[4][4];
    float roll = rpy[0];
    float pitch = rpy[1];
    float yaw = rpy[2];
    float px = org[0];
    float py = org[1];
    float pz = org[2];

    float A = cosf (yaw), B = sinf (yaw), C = cosf (pitch), D = sinf (pitch),
            E = cosf (roll), F = sinf (roll), DE = D*E, DF = D*F;
    eigen_virtualFromFocus_updated(0,0) = A*C; eigen_virtualFromFocus_updated(0,1) = A*DF - B*E; eigen_virtualFromFocus_updated(0,2) = B*F + A*DE; eigen_virtualFromFocus_updated(0,3) = px;
    eigen_virtualFromFocus_updated(1,0) = B*C; eigen_virtualFromFocus_updated(1,1) = A*E + B*DF; eigen_virtualFromFocus_updated(1,2) = B*DE - A*F; eigen_virtualFromFocus_updated(1,3) = py;
    eigen_virtualFromFocus_updated(2,0) = -D;  eigen_virtualFromFocus_updated(2,1) = C*F;        eigen_virtualFromFocus_updated(2,2) = C*E;        eigen_virtualFromFocus_updated(2,3) = pz;
    eigen_virtualFromFocus_updated(3,0) = 0.f; eigen_virtualFromFocus_updated(3,1) = 0.f;        eigen_virtualFromFocus_updated(3,2) = 0.f;        eigen_virtualFromFocus_updated(3,3) = 1.f;

    //    camPos_update, focus to cam

    tf_listener->waitForTransform(focusFrame, baseFrame, ros::Time(0), ros::Duration(3.0));
    tf_listener->lookupTransform(focusFrame, baseFrame, ros::Time(0), T_baseFromFocus);
    transformAsMatrix (T_baseFromFocus, eigen_baseFromFocus);

    // eigen_baseFromFocus: at t-1
    // eigen_virtualFromFocus_updated: at t
    // eigen_virtualFromFocus_old: at t-1
    // eigen_camFromFocus: at t
    eigen_camFromOrg_updated = eigen_baseFromFocus_old.inverse() * eigen_virtualFromFocus_updated * eigen_virtualFromFocus_old.inverse() * eigen_camFromFocus;


    virtualPos_old = virtualPos_updated;
    transformAsMatrix (virtualPos_old, eigen_virtualFromFocus_old);

    if(DEBUG_ALGORITHM) cout<<"-------------camupdated2"<<endl;
    if(DEBUG_ALGORITHM) cout<<eigen_camFromOrg_updated<<endl;

    tf::Vector3 origin;
    origin.setValue(static_cast<double>(eigen_camFromOrg_updated(0,3)),static_cast<double>(eigen_camFromOrg_updated(1,3)),static_cast<double>(eigen_camFromOrg_updated(2,3)));


    tf::Matrix3x3 tf3d;
    tf3d.setValue(static_cast<double>(eigen_camFromOrg_updated(0,0)), static_cast<double>(eigen_camFromOrg_updated(0,1)), static_cast<double>(eigen_camFromOrg_updated(0,2)),
                  static_cast<double>(eigen_camFromOrg_updated(1,0)), static_cast<double>(eigen_camFromOrg_updated(1,1)), static_cast<double>(eigen_camFromOrg_updated(1,2)),
                  static_cast<double>(eigen_camFromOrg_updated(2,0)), static_cast<double>(eigen_camFromOrg_updated(2,1)), static_cast<double>(eigen_camFromOrg_updated(2,2)));

    tf::Quaternion tfqt;
    tf3d.getRotation(tfqt);

    camPos_updated.setOrigin(origin);
    camPos_updated.setRotation(tfqt);
    isCamUpdate = 1;



    Eigen::Matrix4f eigen_FocusFromBase_scene = eigen_baseFromFocus.inverse() * eigen_virtualFromFocus_updated;



    origin.setValue(static_cast<double>(eigen_FocusFromBase_scene(0,3)),static_cast<double>(eigen_FocusFromBase_scene(1,3)),static_cast<double>(eigen_FocusFromBase_scene(2,3)));


    tf3d.setValue(static_cast<double>(eigen_FocusFromBase_scene(0,0)), static_cast<double>(eigen_FocusFromBase_scene(0,1)), static_cast<double>(eigen_FocusFromBase_scene(0,2)),
                  static_cast<double>(eigen_FocusFromBase_scene(1,0)), static_cast<double>(eigen_FocusFromBase_scene(1,1)), static_cast<double>(eigen_FocusFromBase_scene(1,2)),
                  static_cast<double>(eigen_FocusFromBase_scene(2,0)), static_cast<double>(eigen_FocusFromBase_scene(2,1)), static_cast<double>(eigen_FocusFromBase_scene(2,2)));


    tf3d.getRotation(tfqt);

    camPos_updated_scene.setOrigin(origin);
    camPos_updated_scene.setRotation(tfqt);




    /*

    Eigen::Matrix4f eigen_camFromFocus;
    transformAsMatrix (T_camFromFocus, eigen_camFromFocus);
    Eigen::Matrix4f eigen_focusFromCam = eigen_camFromFocus.inverse();

    Eigen::Matrix4f eigen_virtualFromFocus_updated;
    float t[4][4];
    float roll = rpy[0];
    float pitch = rpy[1];
    float yaw = rpy[2];
    float px = org[0];
    float py = org[1];
    float pz = org[2];

    float A = cosf (yaw), B = sinf (yaw), C = cosf (pitch), D = sinf (pitch),
            E = cosf (roll), F = sinf (roll), DE = D*E, DF = D*F;
    eigen_virtualFromFocus_updated(0,0) = A*C; eigen_virtualFromFocus_updated(0,1) = A*DF - B*E; eigen_virtualFromFocus_updated(0,2) = B*F + A*DE; eigen_virtualFromFocus_updated(0,3) = px;
    eigen_virtualFromFocus_updated(1,0) = B*C; eigen_virtualFromFocus_updated(1,1) = A*E + B*DF; eigen_virtualFromFocus_updated(1,2) = B*DE - A*F; eigen_virtualFromFocus_updated(1,3) = py;
    eigen_virtualFromFocus_updated(2,0) = -D;  eigen_virtualFromFocus_updated(2,1) = C*F;        eigen_virtualFromFocus_updated(2,2) = C*E;        eigen_virtualFromFocus_updated(2,3) = pz;
    eigen_virtualFromFocus_updated(3,0) = 0.f; eigen_virtualFromFocus_updated(3,1) = 0.f;        eigen_virtualFromFocus_updated(3,2) = 0.f;        eigen_virtualFromFocus_updated(3,3) = 1.f;

    eigen_camFromOrg_updated = eigen_focusFromCam * eigen_virtualFromFocus_updated * eigen_camFromFocus;

*/

    cnt++;



}
