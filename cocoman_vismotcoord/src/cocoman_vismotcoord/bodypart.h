#ifndef BODYPART_H
#define BODYPART_H

#include "mesh.h"
#include "common.h"

class BodyPart
{
public:
    BodyPart(string frame);
    bool importModel(string modelPath);
    void transformMesh(const Eigen::Matrix4f &transform);
    void focusMesh(const Eigen::Matrix4f &transform);
    void filteringMesh(Vector3f camz);
    void transformSetMesh(const Eigen::Matrix4f &transform);
    void transformOrgMesh(const Eigen::Matrix4f &transform);

    string frame(){return m_frame;};
    string modelPath(){return m_modelPath;};
    Mesh* meshModel(){return &m_mesh;};
    Mesh* meshTransformed(){return &m_mesh_transformed;};
    Mesh* meshFocused(){return &m_mesh_focused;};
    Mesh* meshOrg(){return &m_mesh_org;};
    int numFace(){return numFaces;};
    int numFilteredFace(){return numFilteredFaces;};
    MatrixXf boundBox(){return m_boundBox;};
private:
    const aiScene* model;
    string m_frame;
    string m_modelPath;
    Mesh m_mesh;
    Mesh m_mesh_transformed;
    Mesh m_mesh_focused;
    Mesh m_mesh_org;
    int numFaces;
    int numFilteredFaces;
    MatrixXf m_boundBox;
};

#endif // BODYPART_H
