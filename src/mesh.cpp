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

void Mesh::visualizeEdgeTypes() {
    // 1. Clear existing structures to avoid buffer conflicts
    polyscope::removeStructure("wheel edges");
    polyscope::removeStructure("hinge edges");

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

        if (_edgeRoll[e].type == RollType::WHEEL) {
            wheelEdges.push_back({u, v});
        } else if (_edgeRoll[e].type == RollType::HINGE) {
            hingeEdges.push_back({u, v});
        }
    }

    // 2. Only register if there is actually data to show
    // Polyscope sometimes throws 'inconsistent size' errors if you pass an empty edge list
    if (!wheelEdges.empty()) {
        auto *psWheel = polyscope::registerCurveNetwork("wheel edges", nodePositions, wheelEdges);
        psWheel->setEnabled(true);
        psWheel->setColor({1.0, 0.0, 0.0});
        psWheel->setRadius(0.005);
    }

    if (!hingeEdges.empty()) {
        auto *psHinge = polyscope::registerCurveNetwork("hinge edges", nodePositions, hingeEdges);
        psHinge->setEnabled(true);
        psHinge->setColor({0.0, 1.0, 0.0});
        psHinge->setRadius(0.005);
    }
}

void Mesh::show(std::vector<TraceStep> steps) {
    polyscope::init();

    for (size_t i = 0; i < steps.size(); i++) {
        std::string name = "mesh_step_" + std::to_string(i);
        auto* ps = polyscope::registerSurfaceMesh(name, _hullGeom->vertexPositions, _hull->getFaceVertexList());

        // 1. Position the mesh based on the normal
        ps->setTransform(eigenToGlm(normalToTransform(steps[i].n)));
        Eigen::Matrix4d T = normalToTransform(steps[i].n);
        // Shift each step 2 units to the right along the X axis
        T(0, 3) = i * 3.0;
        ps->setTransform(eigenToGlm(T));

        // 2. Prepare highlighting arrays
        // We initialize these to a "neutral" color (light gray)
        std::vector<glm::vec3> fColors(_hull->nFaces(), {0.95, 0.95, 0.95});
        std::vector<double> vWeights(_hull->nVertices(), 0.0);

        // 3. Highlight based on element type
        SurfacePoint p = steps[i].elem;
        if (p.type == SurfacePointType::Face) {
            fColors[p.face.getIndex()] = {1.0, 0.2, 0.2}; // Bright Red for Face
        }
        else if (p.type == SurfacePointType::Edge) {
            // Color both faces adjacent to the edge in Orange
            fColors[p.edge.halfedge().face().getIndex()] = {1.0, 0.6, 0.0};
            fColors[p.edge.halfedge().twin().face().getIndex()] = {1.0, 0.6, 0.0};
        }
        else if (p.type == SurfacePointType::Vertex) {
            vWeights[p.vertex.getIndex()] = 1.0; // Highlight vertex
            // Also color all surrounding faces in Blue to make the vertex visible
            for(Face f : p.vertex.adjacentFaces()) {
                fColors[f.getIndex()] = {0.2, 0.2, 1.0};
            }
        }

        // 4. Register the colors to the mesh instance
        ps->addFaceColorQuantity("Contact Element", fColors)->setEnabled(true);
        if (p.type == SurfacePointType::Vertex) {
            ps->addVertexScalarQuantity("Contact Vertex", vWeights)->setEnabled(true);
        }
    }

    // // CLASSIFICATION HIGHLIGHTING

    // polyscope::registerSurfaceMesh("mesh",
    //                                _meshGeom->vertexPositions,
    //                                _mesh->getFaceVertexList());
    // std::vector<glm::vec3> points;
    // points.push_back(glm::vec3(_com.x, _com.y, _com.z));
    // polyscope::PointCloud *psCloud = polyscope::registerPointCloud("center of mass", points);
    // psCloud->setPointRadius(0.02);
    // psCloud->setPointRenderMode(polyscope::PointRenderMode::Quad);
    // polyscope::registerSurfaceMesh("hull",
    //                                _hullGeom->vertexPositions,
    //                                _hull->getFaceVertexList());

    // std::vector<glm::vec3> faceColors;
    // for (Face f: _hull->faces()) {
    //     if (_faceRoll[f].type == RollType::STABLE) {
    //         faceColors.push_back({1.0, 0.0, 0.0});
    //     } else if (_faceRoll[f].type == RollType::HINGE) {
    //         faceColors.push_back({0.0, 1.0, 0.0});
    //     } else if (_faceRoll[f].type == RollType::WHEEL) {
    //         faceColors.push_back({0.0, 0.0, 1.0});
    //     }
    // }

    // polyscope::getSurfaceMesh("hull")->addFaceColorQuantity("face types", faceColors);
    // polyscope::getSurfaceMesh("hull")->setTransparency(0.5);
    // visualizeEdgeTypes();
    
    polyscope::show();
}

void Mesh::computeCenterOfMass()
{
    double totalVolume = 0.0;
    _com = Vector3::zero();

    for (const Face& f : _mesh->faces()) {
        // Get the 3 vertices of this triangle via halfedge traversal
        Halfedge he = f.halfedge();
        const Vector3& a = _meshGeom->vertexPositions[he.vertex()];
        const Vector3& b = _meshGeom->vertexPositions[he.next().vertex()];
        const Vector3& c = _meshGeom->vertexPositions[he.next().next().vertex()];

        // Signed volume of tetrahedron (a, b, c, origin)
        // = (1 / 6) * a · (b × c)
        double signedVol = dot(a, cross(b, c)) / 6.0;
        totalVolume += signedVol;

        // Centroid of this tet is (a + b + c) / 4
        // weighted by its signed volume
        _com += (a + b + c) * signedVol;
    }

    // divide by 4 (tet centroid denominator) and totalVolume
    _com /= (4.0 * totalVolume);

}

// Helper function to convert a normal vector into a mesh orientation
Eigen::Matrix4d Mesh::normalToTransform(const Vector3& norm)
{
    Eigen::Vector3d n(norm.x, norm.y, norm.z);
    Eigen::Quaterniond q =
        Eigen::Quaterniond::FromTwoVectors(n, Eigen::Vector3d(0, 1, 0));
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = q.toRotationMatrix();
    return T;
}

// Helper to turn an eigen matrix to a glm one
glm::mat4 Mesh::eigenToGlm(const Eigen::Matrix4d& T)
{
    glm::mat4 m;
    for (int col = 0; col < 4; col++)
        for (int row = 0; row < 4; row++)
            m[col][row] = static_cast<float>(T(row, col)); // Eigen=row-major, glm=col-major
    return m;
}

void Mesh::classify()
{
    // link lists of roll data to hull
    _edgeRoll = EdgeData<Roll>(*_hull);
    _faceRoll = FaceData<Roll>(*_hull);

    // classify edges and faces
    classifyEdges();
    classifyFaces();
}

void Mesh::classifyEdges()
{
    for (const Edge& e : _hull->edges()) {
        Vertex a = e.firstVertex();
        Vertex b = e.secondVertex();

        Face fA = e.halfedge().face();
        Face fB = e.halfedge().twin().face();

        const Vector3& tempNormA = _hullGeom->faceNormals[fA];
        const Vector3& tempNormB = _hullGeom->faceNormals[fB];

        Eigen::Vector3d nfA{ tempNormA.x, tempNormA.y, tempNormA.z };
        Eigen::Vector3d nfB{ tempNormB.x, tempNormB.y, tempNormB.z };

        const Vector3& v1 = _hullGeom->vertexPositions[a];
        const Vector3& v2 = _hullGeom->vertexPositions[b];

        // convert to Eigen
        Eigen::Vector3d vA{ v1.x, v1.y, v1.z };
        Eigen::Vector3d vB{ v2.x, v2.y, v2.z };

        Eigen::Hyperplane plane = Eigen::Hyperplane<double, 3>::Through(vA, vB);
        Eigen::Vector3d projCom = plane.projection({ _com.x, _com.y, _com.z });
        Eigen::Vector3d edgeDir = vB - vA;
        Eigen::ParametrizedLine edge =
                Eigen::ParametrizedLine<double, 3>::Through(vA, edgeDir.normalized());
        Eigen::Vector3d projComOnEdge = edge.projection(projCom);
        double t = (projComOnEdge - vA).norm() / edgeDir.norm();

        if (0.0 <= t && t <= 1.0) {
            // // HINGE TYPE
            _edgeRoll[e] = { RollType::HINGE, { e, t } };
        } else {
            // // WHEEL TYPE
            // next vertex is closest to projection of center of mass
            Vertex next = (vA - projCom).norm() > (vB - projCom).norm() ? b : a;
            _edgeRoll[e] = {RollType::WHEEL, next};
        }
    }
}

void Mesh::classifyFaces()
{
    for (const Face& f : _hull->faces()) {
        std::vector<Vertex> vertices;
        for (Vertex v : f.adjacentVertices()) {
            vertices.push_back(v);
        }

        std::vector<Edge> edges;
        for (Edge e : f.adjacentEdges()) {
            edges.push_back(e);
        }

        const Vector3& vA = _hullGeom->vertexPositions[vertices[0]];
        const Vector3& vB = _hullGeom->vertexPositions[vertices[1]];
        const Vector3& vC = _hullGeom->vertexPositions[vertices[2]];

        Eigen::Vector3d v1{ vA.x, vA.y, vA.z };
        Eigen::Vector3d v2{ vB.x, vB.y, vB.z };
        Eigen::Vector3d v3{ vC.x, vC.y, vC.z };
        std::vector<Eigen::Vector3d> vPos{v1, v2, v3};

        Eigen::Hyperplane plane = Eigen::Hyperplane<double, 3>::Through(v1, v2, v3);
        Eigen::Vector3d projCom = plane.projection({ _com.x, _com.y, _com.z });
        Eigen::Vector3d edge12 = v2 - v1;
        Eigen::Vector3d edge13 = v3 - v1;
        Eigen::Vector3d crossProd = edge12.cross(edge13);
        double A = 0.5 * crossProd.norm();

        // 2. Calculate signed areas of the 3 sub-triangles formed with P
        // Sub-triangles: (P, v2, v1), (P, v3, v2), (P, v1, v3)
        double A1 = (v2 - projCom).cross(v1 - projCom).norm() * 0.5;
        double A2 = (v3 - projCom).cross(v2 - projCom).norm() * 0.5;
        double A3 = (v1 - projCom).cross(v3 - projCom).norm() * 0.5;

        // 3. Check if P is inside using the dot product with the main normal
        // If the sub-triangle normal points the same way as the main normal, the point is "inside" that edge
        if (A1 / A >= 0.0 && A2 / A >= 0.0 && A3 / A >= 0.0) {
            // f is placeholder for next as stable faces don't roll
            _faceRoll[f] = { RollType::STABLE, {f, Vector3::constant(1.0 / 3.0)} }; // F0: Inside
        } else {
            // 4. Determine if it's a Hinge (F1) or Wheel (F2)
            // It is a Hinge if it's "outside" an edge but within the Voronoi stripe of that edge
            double closest = std::numeric_limits<double>::infinity();

            for (int i = 0; i < 3; i++) {
                Eigen::Vector3d a = vPos[i];
                Eigen::Vector3d b = vPos[(i + 1) % 3];
                Eigen::Vector3d edgeDir = b - a;

                // Vector projection to find position along the infinite line of the edge
                Eigen::ParametrizedLine edge =
                        Eigen::ParametrizedLine<double, 3>::Through(a, edgeDir.normalized());
                Eigen::Vector3d projComOnEdge = edge.projection(projCom);
                double t = (projComOnEdge - a).norm() / edgeDir.norm();

                // If it's outside this specific edge and 0 < t < 1, it's in the stripe
                // We check if it's outside this edge specifically by re-checking the cross product sign
                if (t >= 0.0 && t <= 1.0) {
                    // sets the hinge edge to the one that is in this stripe
                    _faceRoll[f] = { RollType::HINGE, { edges[i], t } };
                    break;
                } else {
                    // rolls onto a vertex
                    double vertDist = (projCom - a).norm();
                    if (vertDist < closest) {
                        closest = vertDist;
                        _faceRoll[f] = { RollType::WHEEL, vertices[i] };
                    }
                }

            }

        }

    }
}
