#ifndef MODEL_H
#define MODEL_H

#include "bodypart.h"

class Model
{
public:
    Model();
//    void setPart(Mesh *partMesh);
    void setPart(const string &frame, const string &path);
    void clear();
    void transformPart(int nPart, Eigen::Matrix4f transform);
    void focusPart(int nPart, Eigen::Matrix4f transform);
    void filteringPart(int nPart, Vector3f camz);
    void transformPartSet(int nPart, Eigen::Matrix4f transform);
    void transformPartOrg(int nPart, Eigen::Matrix4f transform);

    int numPart(){return vecBodyPart.size();};
    string partFrame(int i){return vecBodyPart.at(i)->frame();};
    string partModelPath(int i){return vecBodyPart.at(i)->modelPath();};
    int numFace(){return numFaces;};
    int numFilteredFace(){return numFilteredFaces;};
    BodyPart* bodyPart(int i){return vecBodyPart.at(i);};

    aiFace* getFaceGlobIndex(int i);
    MatrixXf getBoundBox();
//    Vertex* vertex(int n){return &mesh.Vertices.at(n);};
//    aiFace* face(int n){return &mesh.faces.at(n);};
//    int numVertex(){return mesh.Vertices.size();};
//    int numFace(){return mesh.faces.size();};

public:
    vector<BodyPart*> vecBodyPart;
    int numFaces;
    int numFilteredFaces;
    MatrixXf boundBox;
    vector<int> filteredParts;
};

#endif // MODEL_H
