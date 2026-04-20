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

    // build polygon and vertex containers for convex hull data
    std::vector<std::vector<size_t>> poly(iBuf.size() / 3);
    std::vector<Vector3> vert(vBuf.size());

    size_t upper_bound = std::max(poly.size(), vert.size());

    // copy data from temp hull obj to containers in one pass
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

    // pre-compute vertex positions on hull
    _hullGeom->requireVertexPositions();

    // pre-compute vertex and face normals of hull
    _hullGeom->requireVertexNormals();
    _hullGeom->requireFaceNormals();

    // pre-compute edge dihedral angles (angles between faces)
    _hullGeom->requireEdgeDihedralAngles();

    // compute center of mass if not provided
    if (computeCom) computeCenterOfMass();

    // classify all mesh elements
    classify();
}

void Mesh::show()
{
    polyscope::init();

    /** TODO: sequence the outputs
     * for (size_t i = 0; i < normals.size(); i++) {
        polyscope::SurfaceMesh * ps = polyscope::registerSurfaceMesh(
            "mesh_" + std::to_string(i),          // unique name per orientation
            _meshGeom->vertexPositions,
            _mesh->getFaceVertexList()
        );
        ps->setTransform(eigenToGlm(normalToTransform(normals[i])));
    }
     */
    polyscope::SurfaceMesh * psmesh = polyscope::registerSurfaceMesh("mesh",
                                   _meshGeom->vertexPositions,
                                   _mesh->getFaceVertexList());
    polyscope::SurfaceMesh * pshull =
    polyscope::registerSurfaceMesh("hull",
                                   _hullGeom->vertexPositions,
                                   _hull->getFaceVertexList());
    glm::mat4 t = eigenToGlm(normalToTransform(Eigen::Vector3d(0.5,0.5, 0.5)));
    psmesh->setTransform(t);
    pshull->setTransform(t);
    polyscope::show();
}

void Mesh::computeCenterOfMass()
{
    // TODO: compute using mesh, NOT hull

}

//Helper function to convert a normal vector into a mesh orientation
Eigen::Matrix4d Mesh::normalToTransform(const Eigen::Vector3d& n){
    Eigen::Vector3d norm = n.normalized();
    Eigen::Vector3d up(0,0,1);
    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(up, norm);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = q.toRotationMatrix();
    return T;
}

//Helper to turn an eigen matrix to a glm one
glm::mat4 Mesh::eigenToGlm(const Eigen::Matrix4d &T){
    glm::mat4 m;
    for (int col = 0; col < 4; col++)
        for (int row = 0; row < 4; row++)
            m[col][row] = static_cast<float>(T(row, col)); // Eigen=row-major, glm=col-major
    return m;
}

void Mesh::classify()
{
    // link element lists to hull
    _edgeRoll = EdgeData<Roll>(*_hull);
    _faceRoll = FaceData<Roll>(*_hull);

    // classify edges and faces
    classifyEdges();
    classifyFaces();
}

void Mesh::classifyEdges()
{
    // TODO: classify edges as E1/E2
    for (const Edge& e : _hull->edges()) {

    }
}

void Mesh::classifyFaces()
{
    // TODO: classify faces as F0/F1/F2
    for (const Face& f : _hull->faces()) {

    }
}
