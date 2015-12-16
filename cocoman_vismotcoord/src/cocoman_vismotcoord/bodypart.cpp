#include "bodypart.h"

BodyPart::BodyPart(string frame)
    :m_frame(frame)
{
    numFaces = 0;
    numFilteredFaces = 0;
    m_boundBox.resize(3,2);
}

bool BodyPart::importModel(string modelPath)
{
    m_modelPath = modelPath;
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile( m_modelPath,
                                              aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);
    if( !scene)
    {
        if(DEBUG_ALGORITHM) cout<<importer.GetErrorString();
        return false;
    }

    m_mesh.InitFromScene(scene);
    m_mesh_transformed.InitFromScene(scene);
    m_mesh_focused.InitFromScene(scene);
    m_mesh_org.InitFromScene(scene);

    for(int i=0;i<m_mesh.numMesh();i++){
        numFaces += m_mesh.numFaces(i);
        if(DEBUG_ALGORITHM) cout<<m_mesh.numVertex(i)<<" "<<m_mesh.numFaces(i)<<endl;
    }
    if(DEBUG_ALGORITHM) cout<<endl;
    return true;
}

// need to be computed on GPU
void BodyPart::transformMesh(const Eigen::Matrix4f &transform)
{
    for(int i=0;i<m_mesh.numMesh();i++){
        for(int j=0;j<m_mesh.numVertex(i);j++){
            Eigen::Vector4f pt(m_mesh.vertex(i,j)->m_pos[0], m_mesh.vertex(i,j)->m_pos[1], m_mesh.vertex(i,j)->m_pos[2], 1);
            Eigen::Vector4f pt_out = transform * pt;
            m_mesh_transformed.vertex(i,j)->m_pos[0] = pt_out[0];
            m_mesh_transformed.vertex(i,j)->m_pos[1] = pt_out[1];
            m_mesh_transformed.vertex(i,j)->m_pos[2] = pt_out[2];

            //            Eigen::Vector4f ptn(m_mesh.vertex(i,j)->m_normal[0], m_mesh.vertex(i,j)->m_normal[1], m_mesh.vertex(i,j)->m_normal[2], 1);
            //            Eigen::Vector4f ptn_out = transform * ptn;
            //            m_mesh_transformed.vertex(i,j)->m_normal[0] = ptn_out[0];
            //            m_mesh_transformed.vertex(i,j)->m_normal[1] = ptn_out[1];
            //            m_mesh_transformed.vertex(i,j)->m_normal[2] = ptn_out[2];
        }
    }
}

// need to be computed on GPU
void BodyPart::focusMesh(const Eigen::Matrix4f &transform)
{
    int cnt = 0;
    for(int i=0;i<m_mesh_transformed.numMesh();i++){
        // copy face from transformed mesh
        m_mesh_focused.m_meshes[i].filteredFaceId = m_mesh_transformed.m_meshes[i].filteredFaceId;
        m_mesh_focused.m_meshes[i].faceNorms = m_mesh_transformed.m_meshes[i].faceNorms;

        for(int j=0;j<m_mesh_transformed.numVertex(i);j++){
            Eigen::Vector4f pt(m_mesh_transformed.vertex(i,j)->m_pos[0], m_mesh_transformed.vertex(i,j)->m_pos[1], m_mesh_transformed.vertex(i,j)->m_pos[2], 1);
            Eigen::Vector4f pt_out = transform * pt;
            m_mesh_focused.vertex(i,j)->m_pos[0] = pt_out[0];
            m_mesh_focused.vertex(i,j)->m_pos[1] = pt_out[1];
            m_mesh_focused.vertex(i,j)->m_pos[2] = pt_out[2];

            //            Eigen::Vector4f ptn(m_mesh_transformed.vertex(i,j)->m_normal[0], m_mesh_transformed.vertex(i,j)->m_normal[1], m_mesh_transformed.vertex(i,j)->m_normal[2], 1);
            //            Eigen::Vector4f ptn_out = transform * ptn;
            //            m_mesh_focused.vertex(i,j)->m_normal[0] = ptn_out[0];
            //            m_mesh_focused.vertex(i,j)->m_normal[1] = ptn_out[1];
            //            m_mesh_focused.vertex(i,j)->m_normal[2] = ptn_out[2];
        }
    }
}

void BodyPart::filteringMesh(Vector3f camz)
{
    numFilteredFaces = 0;
    for(int i=0;i<m_mesh.numMesh();i++){
        m_mesh_transformed.clearFaceNorm(i);
        m_mesh_transformed.clearFilteredFaceId(i);

        for(int j=0;j<m_mesh_transformed.numFaces(i);j++){
            //            Vector3f faceNorm;
            //            faceNorm[0] = faceNorm[1] = faceNorm[2] = 0.f;
            //            for(int k=0;k<3;k++){
            //                faceNorm += m_mesh_transformed.vertex(i,m_mesh_transformed.face(i,j)->mIndices[k])->m_normal;
            //            }
            //            faceNorm = faceNorm / faceNorm.norm();
            //            m_mesh_transformed.pushFaceNorm(i,faceNorm);

            // filtering
            if(m_frame.compare("/left_lower_forearm") == 0 || m_frame.compare("/left_upper_forearm") == 0){
                //                float ang = acos(faceNorm.dot(camz) / (faceNorm.norm()*camz.norm()));
                //                if(ang > 3.141592 / 2.) {
                m_mesh_transformed.pushFilteredFaceId(i, j);
                numFilteredFaces ++;
                //                }
            }
        }
    }
}

void BodyPart::transformSetMesh(const Eigen::Matrix4f &transform)
{
    numFilteredFaces = 0;
    int cnt = 0;
    // transform
    for(int i=0;i<m_mesh.numMesh();i++){
        m_mesh_focused.clearFaceNorm(i);
        m_mesh_focused.clearFilteredFaceId(i);

        for(int j=0;j<m_mesh.numVertex(i);j++){
            Eigen::Vector4f pt(m_mesh.vertex(i,j)->m_pos[0], m_mesh.vertex(i,j)->m_pos[1], m_mesh.vertex(i,j)->m_pos[2], 1);
            Eigen::Vector4f pt_out = transform * pt;
            m_mesh_focused.vertex(i,j)->m_pos[0] = pt_out[0];
            m_mesh_focused.vertex(i,j)->m_pos[1] = pt_out[1];
            m_mesh_focused.vertex(i,j)->m_pos[2] = pt_out[2];

            if(USENORM) {
                Eigen::Vector4f ptn(m_mesh.vertex(i,j)->m_normal[0], m_mesh.vertex(i,j)->m_normal[1], m_mesh.vertex(i,j)->m_normal[2], 1);
                Eigen::Vector4f ptn_out = transform * ptn;
                m_mesh_focused.vertex(i,j)->m_normal[0] = ptn_out[0];
                m_mesh_focused.vertex(i,j)->m_normal[1] = ptn_out[1];
                m_mesh_focused.vertex(i,j)->m_normal[2] = ptn_out[2];
            }

            if(cnt == 0){
                m_boundBox(0,0) = m_boundBox(0,1) = pt_out[0];
                m_boundBox(1,0) = m_boundBox(1,1) = pt_out[1];
                m_boundBox(2,0) = m_boundBox(2,1) = pt_out[2];
            }
            else{
                if(pt_out[0] > m_boundBox(0,0))    m_boundBox(0,0) = pt_out[0];
                if(pt_out[0] < m_boundBox(0,1))    m_boundBox(0,1) = pt_out[0];
                if(pt_out[1] > m_boundBox(1,0))    m_boundBox(1,0) = pt_out[1];
                if(pt_out[1] < m_boundBox(1,1))    m_boundBox(1,1) = pt_out[1];
                if(pt_out[2] > m_boundBox(2,0))    m_boundBox(2,0) = pt_out[2];
                if(pt_out[2] < m_boundBox(2,1))    m_boundBox(2,1) = pt_out[2];
            }
            cnt++;
        }
        for(int j=0;j<m_mesh.numFaces(i);j++){
            m_mesh_focused.pushFilteredFaceId(i, j);
            numFilteredFaces ++;
//            if(USENORM) {
//                Vector3f faceNorm;
//                faceNorm[0] = faceNorm[1] = faceNorm[2] = 0.f;
//                for(int k=0;k<3;k++){
//                    faceNorm += m_mesh_focused.vertex(i,m_mesh_focused.face(i,j)->mIndices[k])->m_normal;
//                }
//                faceNorm = faceNorm / faceNorm.norm();
//                m_mesh_focused.pushFaceNorm(i, faceNorm);
//            }

        }
//        m_mesh_focused.m_meshes[i].filteredFaceId = m_mesh_transformed.m_meshes[i].filteredFaceId;
    }
}


void BodyPart::transformOrgMesh(const Eigen::Matrix4f &transform)
{
    // transform
    for(int i=0;i<m_mesh.numMesh();i++){
        for(int j=0;j<m_mesh.numVertex(i);j++){
            //            Eigen::Vector4f pt(m_mesh_focused.vertex(i,j)->m_pos[0], m_mesh_focused.vertex(i,j)->m_pos[1], m_mesh_focused.vertex(i,j)->m_pos[2], 1);
            //            Eigen::Vector4f pt_out = transform * pt;
            //            m_mesh_org.vertex(i,j)->m_pos[0] = pt_out[0];
            //            m_mesh_org.vertex(i,j)->m_pos[1] = pt_out[1];
            //            m_mesh_org.vertex(i,j)->m_pos[2] = pt_out[2];

            m_mesh_org.vertex(i,j)->m_pos[0] = transform(0,0)*m_mesh_focused.vertex(i,j)->m_pos[0] + transform(0,1)*m_mesh_focused.vertex(i,j)->m_pos[1] + transform(0,2)*m_mesh_focused.vertex(i,j)->m_pos[2] + transform(0,3);
            m_mesh_org.vertex(i,j)->m_pos[1] = transform(1,0)*m_mesh_focused.vertex(i,j)->m_pos[0] + transform(1,1)*m_mesh_focused.vertex(i,j)->m_pos[1] + transform(1,2)*m_mesh_focused.vertex(i,j)->m_pos[2] + transform(1,3);
            m_mesh_org.vertex(i,j)->m_pos[2] = transform(2,0)*m_mesh_focused.vertex(i,j)->m_pos[0] + transform(2,1)*m_mesh_focused.vertex(i,j)->m_pos[1] + transform(2,2)*m_mesh_focused.vertex(i,j)->m_pos[2] + transform(2,3);

        }
        m_mesh_org.m_meshes[i].filteredFaceId = m_mesh_focused.m_meshes[i].filteredFaceId;
    }
}
