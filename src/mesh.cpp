#include "mesh.h"

#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/surface_mesh_factories.h"
#include "geometrycentral/utilities/eigen_interop_helpers.h"

#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"

#include "util/quickhull/QuickHull.hpp"

Mesh::Mesh(const std::string& meshPath, const Vector3& com, bool computeCom) :
    _com(com)
{
    // load mesh
    std::tie(_mesh, _meshGeom) = readSurfaceMesh(meshPath);

    // map out as flat array
    auto flatMap = FlattenedEigenMap<double, 3>(_meshGeom->vertexPositions);

    // invoke quickhull algorithm
    quickhull::QuickHull<double> qh;
    auto hull = qh.getConvexHull(flatMap.data(), flatMap.size() / 3, true, false);

    // fetch faces and vertices of convex hull
    const auto& iBuf = hull.getIndexBuffer();
    const auto& vBuf = hull.getVertexBuffer();

    // build polygons and vertices from convex hull
    std::vector<std::vector<size_t>> poly(iBuf.size() / 3);
    std::vector<Vector3> vert(vBuf.size());

    size_t upper_bound = std::max(poly.size(), vert.size());

    // copy data from temp hull obj to containers in single loop
    for (size_t i = 0; i < upper_bound; i++) {
        if (i < poly.size()) {
            poly[i] = { iBuf[3 * i], iBuf[3 * i + 1], iBuf[3 * i + 2] };
        }
        if (i < vert.size()) {
            const auto& vec = vBuf[i];
            vert[i] = { vec.x, vec.y, vec.z };
        }
    }

    // build manifold convex hull
    std::tie(_hull, _hullGeom) = makeManifoldSurfaceMeshAndGeometry(poly, vert);

    // pre-compute all indices
    // TODO: would this help with fetching coords by speedup?
    _meshGeom->requireVertexIndices();
    _meshGeom->requireEdgeIndices();
    _meshGeom->requireFaceIndices();
    _hullGeom->requireVertexIndices();
    _hullGeom->requireEdgeIndices();
    _hullGeom->requireFaceIndices();

    // pre-compute face and vertex normals of hull
    _hullGeom->requireVertexNormals();
    _hullGeom->requireFaceNormals();

    // compute center of mass if not provided
    if (computeCom) computeCenterOfMass();

    // classify all mesh elements
    classify();
}

void Mesh::showMesh()
{
    polyscope::init();
    polyscope::registerSurfaceMesh("mesh",
                                   _meshGeom->vertexPositions,
                                   _mesh->getFaceVertexList());
    polyscope::show();
}

void Mesh::showHull()
{
    polyscope::init();
    polyscope::registerSurfaceMesh("hull",
                                   _hullGeom->vertexPositions,
                                   _hull->getFaceVertexList());
    polyscope::show();
}

void Mesh::computeCenterOfMass()
{
    // TODO: compute using mesh, NOT hull

}

void Mesh::classify()
{
    _edgeTypes = EdgeData<ElemType>(*_hull, ElemType::WHEEL);
    _faceTypes = FaceData<ElemType>(*_hull, ElemType::STABLE);

    classifyEdges();
    classifyFaces();
}

void Mesh::classifyEdges()
{
    // TODO: classify edges as E1/E2
    for (Edge e : _hull->edges()) {

    }
}

void Mesh::classifyFaces()
{
    // TODO: classify faces as F0/F1/F2
    for (Face f : _hull->faces()) {

    }
}
