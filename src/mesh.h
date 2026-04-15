#ifndef MESH_H
#define MESH_H

#include <string>
#include <memory>

#include "geometrycentral/surface/surface_mesh.h"
#include "geometrycentral/surface/vertex_position_geometry.h"
#include "geometrycentral/surface/manifold_surface_mesh.h"

using namespace geometrycentral;
using namespace geometrycentral::surface;

enum class RollType
{
    STABLE, // FO
    WHEEL,  // F1 / E1
    HINGE   // F2 / E2
};

class Mesh
{
public:
    // constructor
    Mesh(const std::string& meshPath, const Vector3& com, bool computeCom);

    // getters and setters
    inline const SurfaceMesh& getMesh() const { return *_mesh; }
    inline const ManifoldSurfaceMesh& getHull() const { return *_hull; }

    inline const VertexPositionGeometry& getMeshGeom() const { return *_meshGeom; }
    inline const VertexPositionGeometry& getHullGeom() const { return *_hullGeom; }

    inline const EdgeData<RollType>& getEdgeTypes() const { return _edgeTypes; }
    inline const FaceData<RollType>& getFaceTypes() const { return _faceTypes; }

    // viewing func
    void show();

private:
    std::unique_ptr<SurfaceMesh> _mesh;
    std::unique_ptr<ManifoldSurfaceMesh> _hull;
    std::unique_ptr<VertexPositionGeometry> _meshGeom, _hullGeom;

    Vector3 _com;

    EdgeData<RollType> _edgeTypes;
    FaceData<RollType> _faceTypes;

    void computeCenterOfMass();

    void classify();
    void classifyEdges();
    void classifyFaces();
};

#endif // MESH_H
