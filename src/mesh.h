#ifndef MESH_H
#define MESH_H

#include <string>
#include <memory>

#include "geometrycentral/surface/surface_mesh.h"
#include "geometrycentral/surface/vertex_position_geometry.h"
#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/surface_point.h"
#include <btBulletDynamicsCommon.h>

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

struct TraceStep
{
    SurfacePoint elem;
    Vector3 n;
};

struct FaceResult;

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
    const std::vector<FaceResult>& getFaceResults() const { return _faceResults; }

    // Initialise a fresh Bullet world (gravity, ground plane).
    // Caller owns the returned pointer and must delete all sub-objects.
    btDiscreteDynamicsWorld* initSim();

    // Add the convex hull as a dynamic rigid body (COM shifted to origin).
    // Returns the btRigidBody; it is already added to 'world'.
    btRigidBody* createBulletBody(btDiscreteDynamicsWorld* world);

    // viewing func
    void show(std::vector<TraceStep> N);
    void buildCoplanarGroups(double angleTolerance);
    void showFaceProbabilities();
    void setFaceResults(const std::vector<FaceResult>& results);


    std::vector<int> _faceGroup;      // maps triangle index -> group index
    int _numGroups = 0;
    int _numTrials = 0;
    void setNumTrials(int n) { _numTrials = n; }
    std::vector<Eigen::Vector3d> _stableFaceNormals;  // indexed by face
    std::vector<int> _stableFaceIndices;

private:
    std::unique_ptr<SurfaceMesh> _mesh;
    std::unique_ptr<VertexPositionGeometry> _meshGeom;

    std::unique_ptr<ManifoldSurfaceMesh> _hull;
    std::unique_ptr<VertexPositionGeometry> _hullGeom;

    Vector3 _com;
    std::vector<FaceResult> _faceResults;

    EdgeData<Roll> _edgeRoll;
    FaceData<Roll> _faceRoll;

    void computeCenterOfMass();
    void visualizeEdgeTypes();

    void classify();
    void classifyEdges();
    void classifyFaces();

    Eigen::Matrix4d normalToTransform(const Vector3& n);
    static glm::mat4 eigenToGlm(const Eigen::Matrix4d& T);

    static constexpr double RECIP_6 = 1.0 / 6.0;
};

#endif // MESH_H
