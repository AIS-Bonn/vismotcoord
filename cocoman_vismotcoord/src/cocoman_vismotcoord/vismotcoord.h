#ifndef VISMOTCOORD_H
#define VISMOTCOORD_H

#include "common.h"
#include "scene.h"
#include "model.h"
#include "posest.h"

class VisMotCoord
{
public:
    VisMotCoord(tf::TransformListener *tf_listener_, string baseFrame_, string camFrame_init_, string camFrame_updated_);
    bool setBodyModel(const vector<string> &vecPartFrame, const vector<string> &vecPartPath);
    void bodyTransform();
    void bodyFiltering();
    void bodyTransformSet(string focusFrame_);
    void bodyTransformOrg();

//    void setModel();
    void setScene(CloudPtr &pc_in, ros::Time currentT);
    void setScene2(CloudPtr &pc_in, ros::Time currentT);
    void setRefFrames(vector<string> &refFrames, float margin);
    void setBoundBox(float boundMargin);
    void coordinate();
    void focusBody(string focusFrame_);
    void focusScene(ros::Time currentT);

    void getPC(CloudPtr &pc_out);
    void getPCFocus(CloudPtr &pc_out);
    void getPCScene(CloudPtr &pc_out);
    MatrixXf getBoundBox(){return boundBox;};
    int numScenePC(){return scene.numPC();};
    Model* getModel(){return &model;};
    tf::StampedTransform getPosUpdated_virtual(){return virtualPos_updated;};
    tf::StampedTransform getPosUpdated_cam(){return camPos_updated;};
    tf::StampedTransform getPosUpdated_cam_scene(){return camPos_updated_scene;};
    vector<int> getCrspdFacePos(){return posest.getCrspdFacePos();};

    bool isCamUpdated(){return isCamUpdate;};
    void updateScene(CloudPtr &pc_in, CloudPtr &pc_updated);

public:
        vector<string> filterFrameNames;
        bool isCamUpdate;
private:
//    void transformAsMatrix (const tf::Transform& bt, Eigen::Matrix4f &out_mat);

private:
    string baseFrame, camFrame_init, camFrame_updated, focusFrame;
    tf::TransformListener *tf_listener;

    tf::StampedTransform camPos_init;
    tf::StampedTransform camPos_updated;
    tf::StampedTransform camPos_updated_scene;
    tf::StampedTransform virtualPos_updated, virtualPos_old;

    tf::StampedTransform T_baseFromFocus, T_camFromFocus;
    Eigen::Matrix4f eigen_camFromOrg_updated;
    Eigen::Matrix4f eigen_baseFromFocus;
    Eigen::Matrix4f eigen_baseFromFocus_old;

    Eigen::Matrix4f eigen_camFromFocus;
    Eigen::Matrix4f eigen_virtualFromFocus_old;

    tf::StampedTransform cam_experiment;
    tf::StampedTransform T_camFromFocus_exp;
    Eigen::Matrix4f eigen_camFromBase_exp;

    // initial param
    tf::Matrix3x3 cross_rot;
    tf::Vector3 cross_org;
    tf::Matrix3x3 stretch_rot;
    tf::Vector3 stretch_org;
    tf::Matrix3x3 upward_rot;
    tf::Vector3 upward_org;

    Model model;
    Scene scene;
    MatrixXf boundBox, gridspace;
    PosEst posest;


    int cnt;
};

#endif // VISMOTCOORD_H
