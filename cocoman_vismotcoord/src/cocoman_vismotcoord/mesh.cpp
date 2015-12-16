#include "mesh.h"


Mesh::Mesh()
{
}


Mesh::~Mesh()
{
    for(int i=0;i<numMeshes;i++){
        delete[] m_meshes[i].faces;
        delete[] m_meshes[i].vertices;
    }
    delete[] m_meshes;
}


bool Mesh::InitFromScene(const aiScene* pScene)
{
    numMeshes = pScene->mNumMeshes;
    m_meshes = new MeshEntry[numMeshes];
    // Initialize the meshes in the scene one by one
    for (unsigned int i = 0 ; i < numMeshes ; i++) {
        const aiMesh* paiMesh = pScene->mMeshes[i];
        initMesh(i, paiMesh);
    }

    return 1;
}

void Mesh::initMesh(unsigned int index, const aiMesh* paiMesh)
{
    m_meshes[index].numVertices = paiMesh->mNumVertices;
    m_meshes[index].vertices = new Vertex[m_meshes[index].numVertices];
    const aiVector3D Zero3D(0.0f, 0.0f, 0.0f);
    for (unsigned int i = 0 ; i < paiMesh->mNumVertices ; i++) {
        const aiVector3D* pPos      = &(paiMesh->mVertices[i]);
        const aiVector3D* pNormal   = &(paiMesh->mNormals[i]);
        const aiVector3D* pTexCoord = paiMesh->HasTextureCoords(0) ? &(paiMesh->mTextureCoords[0][i]) : &Zero3D;

        Vertex v(Vector3f(pPos->x, pPos->y, pPos->z),
                 Vector2f(pTexCoord->x, pTexCoord->y),
                 Vector3f(pNormal->x, pNormal->y, pNormal->z));

        m_meshes[index].vertices[i] = v;
    }

    m_meshes[index].numFaces = paiMesh->mNumFaces;
    m_meshes[index].faces = new aiFace[m_meshes[index].numFaces];
    for (unsigned int i = 0 ; i < paiMesh->mNumFaces ; i++) {
        const aiFace& face = paiMesh->mFaces[i];
        m_meshes[index].faces[i] = face;

//        assert(Face.mNumIndices == 3);
//        m_meshes[index].Indices.push_back(Face.mIndices[0]);
//        m_meshes[index].Indices.push_back(Face.mIndices[1]);
//        m_meshes[index].Indices.push_back(Face.mIndices[2]);

    }
}

void Mesh::pushFilteredFaceId(int nMesh, int faceId)
{
    m_meshes[nMesh].filteredFaceId.push_back(faceId);
}

void Mesh::pushFaceNorm(int nMesh, Vector3f faceNorm)
{
    m_meshes[nMesh].faceNorms.push_back(faceNorm);
}
