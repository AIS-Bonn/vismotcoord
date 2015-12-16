#include "model.h"

Model::Model()
{
    numFaces = 0;
    numFilteredFaces = 0;
    boundBox.resize(3,2);
}

void Model::clear()
{
}

//void Model::setPart(Mesh *partMesh)
//{
//    int cnt = 0;
//    for(int i=0;i<partMesh->numMesh();i++){
//        for(int j=0;j<partMesh->numFaces(i);j++){
//            Face face;
//            for(int k=0;k<3;k++){
//                face.vertexes[k] = *(partMesh->vertex(i,partMesh->face(i,j)->mIndices[k]));
//                cnt ++;
//            }
//            faces.push_back(face);
//        }
//    }
//    cout<<cnt<<endl;
//}


void Model::setPart(const string &frame, const string &path)
{
    BodyPart* part = new BodyPart(frame);
    if(part->importModel(path)){
        numFaces += part->numFace();
        vecBodyPart.push_back(part);
    }
//    cout<<vecBodyPart.size()<<endl;
}

void Model::transformPart(int nPart, Eigen::Matrix4f transform)
{
    vecBodyPart.at(nPart)->transformMesh(transform);
}

void Model::transformPartSet(int nPart, Eigen::Matrix4f transform)
{
    filteredParts.push_back(nPart);
    vecBodyPart.at(nPart)->transformSetMesh(transform);
    numFilteredFaces += vecBodyPart.at(nPart)->numFilteredFace();
}

void Model::transformPartOrg(int nPart, Eigen::Matrix4f transform)
{
    vecBodyPart.at(nPart)->transformOrgMesh(transform);
}

void Model::focusPart(int nPart, Eigen::Matrix4f transform)
{
    vecBodyPart.at(nPart)->focusMesh(transform);
}

aiFace* Model::getFaceGlobIndex(int i)
{

}

void Model::filteringPart(int nPart, Vector3f camz)
{
    vecBodyPart.at(nPart)->filteringMesh(camz);
    numFilteredFaces += vecBodyPart.at(nPart)->numFilteredFace();
}

MatrixXf Model::getBoundBox()
{
    for(int n=0;n<filteredParts.size();n++){
        MatrixXf boundBox_part = bodyPart(filteredParts.at(n))->boundBox();
        if(n==0){
            boundBox = boundBox_part;
        }
        else{
            if(boundBox_part(0,0) > boundBox(0,0))  boundBox(0,0) = boundBox_part(0,0);
            if(boundBox_part(0,1) < boundBox(0,1))  boundBox(0,1) = boundBox_part(0,1);
            if(boundBox_part(1,0) > boundBox(1,0))  boundBox(1,0) = boundBox_part(1,0);
            if(boundBox_part(1,1) < boundBox(1,1))  boundBox(1,1) = boundBox_part(1,1);
            if(boundBox_part(2,0) > boundBox(2,0))  boundBox(2,0) = boundBox_part(2,0);
            if(boundBox_part(2,1) < boundBox(2,1))  boundBox(2,1) = boundBox_part(2,1);
        }
    }/*
    int cnt=0;
    for(int n=0;n<numPart();n++){
        BodyPart *part = bodyPart(n);
        Mesh* mesh = part->meshFocused();
        for(int i=0;i<mesh->numMesh();i++){
            for(int j=0;j<mesh->numFilteredFaces(i);j++){
                int faceId = mesh->filteredFaceId(i,j);
                for(int k=0;k<3;k++){
                    Vector3f p = mesh->vertex(i,mesh->face(i,faceId)->mIndices[k])->m_pos;
                    if(cnt == 0){
                        boundBox(0,0) = boundBox(0,1) = p[0];
                        boundBox(1,0) = boundBox(1,1) = p[1];
                        boundBox(2,0) = boundBox(2,1) = p[2];
                    }
                    else{
                        if(p[0] > boundBox(0,0))    boundBox(0,0) = p[0];
                        if(p[0] < boundBox(0,1))    boundBox(0,1) = p[0];
                        if(p[1] > boundBox(1,0))    boundBox(1,0) = p[1];
                        if(p[1] < boundBox(1,1))    boundBox(1,1) = p[1];
                        if(p[2] > boundBox(2,0))    boundBox(2,0) = p[2];
                        if(p[2] < boundBox(2,1))    boundBox(2,1) = p[2];
                    }
                    cnt++;
                }
            }
        }
    }*/

    return boundBox;
}
