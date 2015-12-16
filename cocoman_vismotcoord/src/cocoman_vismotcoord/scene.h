#ifndef SCENE_H
#define SCENE_H

#include "common.h"

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/tracking/tracker.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d_omp.h>

class Scene
{
public:
    Scene();

    void initScene(CloudPtr &pc_in, CloudPtr &pc_transformed, MatrixXf boundBox, Eigen::Matrix4f eigen_virtualFromFocus_old);
    void focusScene(tf::Transform &transform);
    void focusSceneUpdate(CloudPtr &pc_in, tf::Transform &transform_camFromBase, tf::Transform &transform_baseFromfocus, MatrixXf boundBox, Eigen::Matrix4f eigen_virtualFromFocus_old);
    CloudPtr getPC(){return pc_scene_base;};
    CloudPtr getPCFocus(){return pc_scene_focus;};
    CloudPtr getPCFocus_back(){return pc_scene_focus_back;};        // only for particle filtering
    CloudPtr getPCScene(){return pc_scene;};
    CloudNPtr getPCNorm(){return pcNorm_scene;};
    int numPC(){return pc_scene_focus->points.size();};

private:
    void calcNormal();

private:
    pcl::KdTreeFLANN<PointNT>::Ptr kdtree_sceneNorm;
    CloudPtr pc_scene;
    CloudPtr pc_scene_focus;
    CloudPtr pc_scene_focus_back;
    CloudPtr pc_scene_base;
    CloudNPtr pcNorm_scene;
};

#endif // SCENE_H
