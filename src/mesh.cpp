#include "mesh.h"

#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/surface_mesh_factories.h"
#include "geometrycentral/utilities/eigen_interop_helpers.h"

#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/point_cloud.h"
#include "polyscope/curve_network.h"

#include "util/quickhull/QuickHull.hpp"
#include <Eigen/Geometry>

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

    // pre-compute face and vertex normals of hull
    _hullGeom->requireVertexNormals();
    _hullGeom->requireFaceNormals();

    // compute center of mass if not provided
    if (computeCom) computeCenterOfMass();

    // classify all mesh elements
    classify();
}

void Mesh::visualizeEdgeTypes() {
    // 1. Clear existing structures to avoid buffer conflicts
    polyscope::removeStructure("Wheel Edges");
    polyscope::removeStructure("Hinge Edges");

    std::vector<glm::vec3> nodePositions;
    for (Vertex v : _hull->vertices()) {
        Vector3 p = _hullGeom->vertexPositions[v];
        nodePositions.push_back({(float)p.x, (float)p.y, (float)p.z});
    }

    std::vector<std::array<size_t, 2>> wheelEdges;
    std::vector<std::array<size_t, 2>> hingeEdges;

    for (Edge e : _hull->edges()) {
        size_t u = e.halfedge().vertex().getIndex();
        size_t v = e.halfedge().twin().vertex().getIndex();

        if (_edgeTypes[e] == RollType::WHEEL) {
            wheelEdges.push_back({u, v});
        } else if (_edgeTypes[e] == RollType::HINGE) {
            hingeEdges.push_back({u, v});
        }
    }

    // 2. Only register if there is actually data to show
    // Polyscope sometimes throws 'inconsistent size' errors if you pass an empty edge list
    if (!wheelEdges.empty()) {
        auto* psWheel = polyscope::registerCurveNetwork("Wheel Edges", nodePositions, wheelEdges);
        psWheel->setEnabled(true);
        psWheel->setColor({1.0, 0.0, 0.0});
        psWheel->setRadius(0.005);
    }

    if (!hingeEdges.empty()) {
        auto* psHinge = polyscope::registerCurveNetwork("Hinge Edges", nodePositions, hingeEdges);
        psHinge->setEnabled(true);
        psHinge->setColor({0.0, 1.0, 0.0});
        psHinge->setRadius(0.005);
    }
}

void Mesh::show()
{
    polyscope::init();
    // polyscope::registerSurfaceMesh("mesh",
    //                                _meshGeom->vertexPositions,
    //                                _mesh->getFaceVertexList());
    std::vector<glm::vec3> points;
    computeCenterOfMass();
    points.push_back(glm::vec3(_com.x, _com.y, _com.z));
    polyscope::PointCloud* psCloud = polyscope::registerPointCloud("really great points", points);
    psCloud->setPointRadius(0.02);
    psCloud->setPointRenderMode(polyscope::PointRenderMode::Quad);
    polyscope::registerSurfaceMesh("hull",
                                   _hullGeom->vertexPositions,
                                   _hull->getFaceVertexList());
    classifyEdges();
    classifyFaces();
    std::vector<glm::vec3> faceColors;
    for (Face f: _hull->faces()) {
        if (_faceTypes[f] == RollType::STABLE) {
            faceColors.push_back({1.0, 0.0, 0.0});
        } else if (_faceTypes[f] == RollType::HINGE) {
            faceColors.push_back({0.0, 1.0, 0.0});
        } else if (_faceTypes[f] == RollType::WHEEL) {
            faceColors.push_back({0.0, 0.0, 1.0});
        }
    }
    polyscope::getSurfaceMesh("hull")->addFaceColorQuantity("faceTypes", faceColors);
    polyscope::getSurfaceMesh("hull")->setTransparency(0.5);
    visualizeEdgeTypes();
    polyscope::show();
}

void Mesh::computeCenterOfMass()
{
    // TODO: compute using mesh, NOT hull
    double totalVolume = 0;
    _com = Vector3::zero();

    for (Face f : _mesh->faces()) {
        // Get the 3 vertices of this triangle via halfedge traversal
        Halfedge he = f.halfedge();
        Vector3 a = _meshGeom->vertexPositions[he.vertex()];
        Vector3 b = _meshGeom->vertexPositions[he.next().vertex()];
        Vector3 c = _meshGeom->vertexPositions[he.next().next().vertex()];

        // Signed volume of tetrahedron (a, b, c, origin)
        // = (1/6) * a · (b × c)
        double signedVol = dot(a, cross(b, c)) / 6.0;
        totalVolume += signedVol;

        // Centroid of this tet is (a + b + c) / 4
        // weighted by its signed volume
        _com += (a + b + c) * signedVol;
    }

    // divide by 4 (tet centroid denominator) and totalVolume
    _com /= (4.0 * totalVolume);

}

void Mesh::classify()
{
    // link lists to mesh instances
    _edgeTypes = EdgeData<RollType>(*_hull, RollType::WHEEL);
    _faceTypes = FaceData<RollType>(*_hull, RollType::STABLE);

    // classify edges and faces
    classifyEdges();
    classifyFaces();
}

void Mesh::classifyEdges()
{
    // TODO: classify edges as E1/E2
    Eigen::Vector3d eigenCom(_com.x, _com.y, _com.z);
    for (Edge e : _hull->edges()) {
        Vertex a = e.firstVertex();
        Vertex b = e.secondVertex();

        Vector3 v1Pos = _hullGeom->vertexPositions[a];
        Vector3 v2Pos = _hullGeom->vertexPositions[b];
        // convert to eigen
        Eigen::Vector3d v1(v1Pos.x, v1Pos.y, v1Pos.z);
        Eigen::Vector3d v2(v2Pos.x, v2Pos.y, v2Pos.z);


        Eigen::Hyperplane<double, 3> plane = Eigen::Hyperplane<double, 3>::Through(v1, v2);
        Eigen::Vector3d projectedCom = plane.projection(eigenCom);
        Eigen::Vector3d edgeDir = v2 - v1;
        Eigen::ParametrizedLine<double, 3> edge = Eigen::ParametrizedLine<double, 3>::Through(v1, edgeDir.normalized());
        Eigen::Vector3d projectedComOnEdge = edge.projection(projectedCom);
        double t = (projectedComOnEdge - v1).norm() / (v2 - v1).norm();
        if (0.0 < t && t < 1.0) {
            _edgeTypes[e] = RollType::HINGE;
        } else {
            _edgeTypes[e] = RollType::WHEEL;
        }
    }
}

void Mesh::classifyFaces()
{
    // TODO: classify faces as F0/F1/F2
    Eigen::Vector3d eigenCom(_com.x, _com.y, _com.z);
    for (Face f : _hull->faces()) {
        std::vector<Vertex> vertices;
        for (Vertex v : f.adjacentVertices()) {
            vertices.push_back(v);
        }
        Vector3 v1Pos = _hullGeom->vertexPositions[vertices[0]];
        Vector3 v2Pos = _hullGeom->vertexPositions[vertices[1]];
        Vector3 v3Pos = _hullGeom->vertexPositions[vertices[2]];

        Eigen::Vector3d v1(v1Pos[0], v1Pos[1], v1Pos[2]);
        Eigen::Vector3d v2(v2Pos[0], v2Pos[1], v2Pos[2]);
        Eigen::Vector3d v3(v3Pos[0], v3Pos[1], v3Pos[2]);
        std::vector<Eigen::Vector3d> eigenVertices{v1, v2, v3};


        Eigen::Hyperplane<double, 3> plane = Eigen::Hyperplane<double, 3>::Through(v1, v2, v3);
        Eigen::Vector3d projectedCom = plane.projection(eigenCom);
        Eigen::Vector3d edge12 = v2 - v1;
        Eigen::Vector3d edge13 = v3 - v1;
        Eigen::Vector3d crossProduct = edge12.cross(edge13);
        double A = 0.5 * crossProduct.norm();

        // 2. Calculate signed areas of the 3 sub-triangles formed with P
        // Sub-triangles: (P, v2, v1), (P, v3, v2), (P, v1, v3)
        double A1 = (v2 - projectedCom).cross(v1 - projectedCom).norm() * 0.5;
        double A2 = (v3 - projectedCom).cross(v2 - projectedCom).norm() * 0.5;
        double A3 = (v1 - projectedCom).cross(v3 - projectedCom).norm() * 0.5;
        // TODO: add epsilon
        // 3. Check if P is inside using the dot product with the main normal
        // If the sub-triangle normal points the same way as the main normal, the point is "inside" that edge

        bool check = (A1 / A) + (A2 / A) + (A3 / A) <= 1.0;
        bool inside1 = (A1 / A) >= 0.0;
        bool inside2 = (A2 / A) >= 0.0;
        bool inside3 = (A3 / A) >= 0.0;
        if (check && inside1 && inside2 && inside3) {
            _faceTypes[f] = RollType::STABLE; // F0: Inside
        } else {
            // 4. Determine if it's a Hinge (F1) or Wheel (F2)
            // It is a Hinge if it's "outside" an edge but within the Voronoi stripe of that edge
            bool inStripe = false;

            for (int i = 0; i < 3; i++) {
                Eigen::Vector3d a = eigenVertices[i];
                Eigen::Vector3d b = eigenVertices[(i + 1) % 3];
                Eigen::Vector3d edgeDir = b - a;

                // Scalar projection to find position along the infinite line of the edge
                Eigen::ParametrizedLine<double, 3> edge = Eigen::ParametrizedLine<double, 3>::Through(v1, edgeDir.normalized());
                Eigen::Vector3d projectedComOnEdge = edge.projection(projectedCom);
                double t = (projectedComOnEdge - a).norm() / (b - a).norm();

                // If it's outside this specific edge and 0 < t < 1, it's in the stripe
                // We check if it's outside this edge specifically by re-checking the cross product sign
                if ((t > 0 && t < 1)) {
                    inStripe = true;
                    break;
                }
            }
            _faceTypes[f] = inStripe ? RollType::HINGE : RollType::WHEEL;
        }
    }
}
