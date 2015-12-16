#ifndef MESH_H
#define MESH_H

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

struct Vertex
{
    Vector3f m_pos;
    Vector2f m_tex;
    Vector3f m_normal;

    Vertex() {}

    Vertex(const Vector3f& pos, const Vector2f& tex, const Vector3f& normal)
    {
        m_pos    = pos;
        m_tex    = tex;
        m_normal = normal;
    }
};


struct MeshEntry {
    unsigned int numVertices;
    unsigned int numFaces;
//    unsigned int numIndices;

    Vertex *vertices;
    aiFace *faces;    
    vector<Vector3f> faceNorms;
    vector<int> filteredFaceId;
//    unsigned int *indices;
};

class Mesh
{
public:
    Mesh();
    ~Mesh();

    bool LoadMesh(const std::string& Filename);
    bool InitFromScene(const aiScene* pScene);

    int numMesh(){return numMeshes;};
    int numVertex(int nMesh){return m_meshes[nMesh].numVertices;};
    int numFaces(int nMesh){return m_meshes[nMesh].numFaces;};
    int numFilteredFaces(int nMesh){return m_meshes[nMesh].filteredFaceId.size();};
//    int numIndex(int nMesh){return m_meshes[nMesh].numIndices;};

    MeshEntry mesh(int nMesh){return m_meshes[nMesh];};
    Vertex* vertex(int nMesh, int nVertex){return &m_meshes[nMesh].vertices[nVertex];};
    aiFace* face(int nMesh, int nFace){return &m_meshes[nMesh].faces[nFace];};
//    int index(int nMesh, int nIndex){return m_meshes[nMesh].indices[nIndex];};
    Vector3f faceNorm(int nMesh, int nFace){return m_meshes[nMesh].faceNorms.at(nFace);};
    void pushFaceNorm(int nMesh, Vector3f faceNorm);
    void pushFilteredFaceId(int nMesh, int faceId);
    int filteredFaceId(int nMesh, int ind){return m_meshes[nMesh].filteredFaceId.at(ind);};
    void clearFaceNorm(int nMesh){m_meshes[nMesh].faceNorms.clear();};
    void clearFilteredFaceId(int nMesh){m_meshes[nMesh].filteredFaceId.clear();};

public:
    void initMesh(unsigned int index, const aiMesh* paiMesh);
    MeshEntry* m_meshes;
    int numMeshes;

};

#endif // MESH_H
