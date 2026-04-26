#ifndef MESH_H
#define MESH_H

#include <string>
#include <memory>

#include "geometrycentral/surface/surface_mesh.h"
#include "geometrycentral/surface/vertex_position_geometry.h"
#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/surface_point.h"
#include "glm/glm.hpp"

using namespace geometrycentral;
using namespace geometrycentral::surface;

enum class RollType
{
    STABLE, // FO
    WHEEL,  // F1 / E1
    HINGE   // F2 / E2
};

struct Roll
{
    RollType type;
    SurfacePoint next;
};

class Mesh
{
public:
    // constructor
    Mesh(const std::string& meshPath, const Vector3& com, bool computeCom);

    // getters and setters
    inline const SurfaceMesh& getMesh() const { return *_mesh; }

    // WARNING: non-const getter due to non-const iterators (e.g. faces(), edges())
    inline ManifoldSurfaceMesh& getHull() { return *_hull; }

    inline const Vector3& getCenterOfMass() const { return _com; }

    inline const VertexPositionGeometry& getMeshGeom() const { return *_meshGeom; }
    inline const VertexPositionGeometry& getHullGeom() const { return *_hullGeom; }

    inline const EdgeData<Roll>& getEdgeRoll() const { return _edgeRoll; }
    inline const FaceData<Roll>& getFaceRoll() const { return _faceRoll; }


    inline const EdgeData<RollType>& getEdgeTypes() const { return _edgeTypes; }
    inline const FaceData<RollType>& getFaceTypes() const { return _faceTypes; }


    // viewing func
    void show();

private:
    std::unique_ptr<SurfaceMesh> _mesh;
    std::unique_ptr<VertexPositionGeometry> _meshGeom;

    std::unique_ptr<ManifoldSurfaceMesh> _hull;
    std::unique_ptr<VertexPositionGeometry> _hullGeom;
    Eigen::Matrix4d normalToTransform(const Eigen::Vector3d& n);
    static glm::mat4 eigenToGlm(const Eigen::Matrix4d &T);
    Vector3 _com;

    EdgeData<Roll> _edgeRoll;
    FaceData<Roll> _faceRoll;
    EdgeData<RollType> _edgeTypes;
    FaceData<RollType> _faceTypes;

    void computeCenterOfMass();
    Face nextFaceFromEdge(Edge e, const Eigen::Vector3d& nHat);
    Vertex nextVertex(SurfacePoint curr, Vector3 projCOM);
    void classify();
    void classifyEdges();
    void classifyFaces();
};

#endif // MESH_H
