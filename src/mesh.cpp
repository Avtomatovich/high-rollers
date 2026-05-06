#include "mesh.h"

#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/surface_mesh_factories.h"
#include "geometrycentral/utilities/eigen_interop_helpers.h"

// #include "polyscope/polyscope.h"
// #include "polyscope/surface_mesh.h"
// #include "polyscope/point_cloud.h"
// #include "polyscope/curve_network.h"

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
    // // 1. Clear existing structures to avoid buffer conflicts
    // polyscope::removeStructure("wheel edges");
    // polyscope::removeStructure("hinge edges");

    // std::vector<glm::vec3> nodePositions;
    // for (Vertex v : _hull->vertices()) {
    //     Vector3 p = _hullGeom->vertexPositions[v];
    //     nodePositions.push_back({(float)p.x, (float)p.y, (float)p.z});
    // }

    // std::vector<std::array<size_t, 2>> wheelEdges, hingeEdges;

    // for (Edge e : _hull->edges()) {
    //     size_t u = e.halfedge().vertex().getIndex();
    //     size_t v = e.halfedge().twin().vertex().getIndex();

    //     if (_edgeRoll[e].type == RollType::WHEEL) {
    //         wheelEdges.push_back({u, v});
    //     } else if (_edgeRoll[e].type == RollType::HINGE) {
    //         hingeEdges.push_back({u, v});
    //     }
    // }

    // // 2. Only register if there is actually data to show
    // // Polyscope sometimes throws 'inconsistent size' errors if you pass an empty edge list
    // if (!wheelEdges.empty()) {
    //     auto *psWheel = polyscope::registerCurveNetwork("wheel edges", nodePositions, wheelEdges);
    //     psWheel->setEnabled(true);
    //     psWheel->setColor({1.0, 0.0, 0.0});
    //     psWheel->setRadius(0.005);
    // }

    // if (!hingeEdges.empty()) {
    //     auto *psHinge = polyscope::registerCurveNetwork("hinge edges", nodePositions, hingeEdges);
    //     psHinge->setEnabled(true);
    //     psHinge->setColor({0.0, 1.0, 0.0});
    //     psHinge->setRadius(0.005);
    // }
}

void Mesh::show()
{
    // polyscope::init();

    // // NORMAL TRANSFORMS

    // TODO: sequence the outputs
    // for (size_t i = 0; i < normals.size(); i++) {
    //     polyscope::SurfaceMesh * ps = polyscope::registerSurfaceMesh(
    //         "mesh_" + std::to_string(i),          // unique name per orientation
    //         _meshGeom->vertexPositions,
    //         _mesh->getFaceVertexList()
    //     );
    //     ps->setTransform(eigenToGlm(normalToTransform(normals[i])));
    // }

    // polyscope::SurfaceMesh *psmesh =
    //         polyscope::registerSurfaceMesh("mesh",
    //                                        _meshGeom->vertexPositions,
    //                                        _mesh->getFaceVertexList());
    // polyscope::SurfaceMesh *pshull =
    //         polyscope::registerSurfaceMesh("hull",
    //                                        _hullGeom->vertexPositions,
    //                                        _hull->getFaceVertexList());
    // glm::mat4 t = eigenToGlm(normalToTransform(Eigen::Vector3d::Constant(0.5)));
    // psmesh->setTransform(t);
    // pshull->setTransform(t);


    // // CLASSIFICATION HIGHLIGHTING

    // polyscope::registerSurfaceMesh("mesh",
    //                                _meshGeom->vertexPositions,
    //                                _mesh->getFaceVertexList());
    // std::vector<glm::vec3> points;
    // points.push_back({ _com.x, _com.y, _com.z });
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
    
    // polyscope::show();
}

void Mesh::computeCenterOfMass()
{
    _com = Vector3::zero();
    double totalVol = 0.0;

    // center of mass formula of masses m, positions p
    // com = \Sum^{n}_{i = 1} m_i * p_i / (\Sum^{n}_{i = 1} m_i)
    // with constant density rho, rho = m / vol, centroids c of volumes
    // com = \Sum^{n}_{i = 1} v_i * c_i / (\Sum^{n}_{i = 1} v_i)

    // init origin
    const Vector3 p1 = Vector3::zero();
    for (const Face& f : _mesh->faces()) {
        // fetch face vertices
        const Halfedge& h = f.halfedge();
        const Vector3& p2 = _meshGeom->vertexPositions[h.vertex()];
        const Vector3& p3 = _meshGeom->vertexPositions[h.next().vertex()];
        const Vector3& p4 = _meshGeom->vertexPositions[h.next().next().vertex()];

        // O'Brien-Hodgins tet vol = (1 / 6) * [(p2 - p1) x (p3 - p1)] . (p4 - p1)
        double vol = RECIP_6 * dot(cross(p2 - p1, p3 - p1), p4 - p1);
        // tet centroid = (1 / 4) * (p1 + p2 + p3 + p4)
        Vector3 centroid = 0.25 * (p1 + p2 + p3 + p4);

        // accumulate numerator and denominator
        _com += vol * centroid;
        totalVol += vol;
    }

    // divide by total volume
    _com /= totalVol;

    std::cout << "center of mass: " << _com << std::endl << std::endl;
}

// Helper function to convert a normal vector into a mesh orientation
Eigen::Matrix4d Mesh::normalToTransform(const Eigen::Vector3d& n)
{
    Eigen::Quaterniond q =
            Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitY(), n);
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

        const Vector3& vA = _hullGeom->vertexPositions[a];
        const Vector3& vB = _hullGeom->vertexPositions[b];

        // convert to Eigen
        Eigen::Vector3d A{ vA.x, vA.y, vA.z };
        Eigen::Vector3d B{ vB.x, vB.y, vB.z };

        Eigen::Hyperplane plane = Eigen::Hyperplane<double, 3>::Through(A, B);
        Eigen::Vector3d P = plane.projection({ _com.x, _com.y, _com.z });
        Eigen::Vector3d edgeDir = B - A;
        Eigen::ParametrizedLine edge =
                Eigen::ParametrizedLine<double, 3>::Through(A, edgeDir.normalized());
        Eigen::Vector3d pEdge = edge.projection(P);
        double t = (pEdge - A).norm() / edgeDir.norm();

        if (0.0 < t && t < 1.0) {
            // // HINGE TYPE
            _edgeRoll[e] = { RollType::HINGE, { e, t } };
        } else {
            // // WHEEL TYPE
            // next vertex is closest to projection of center of mass
            Vertex next = (A - P).norm() < (B - P).norm() ? a : b;
            _edgeRoll[e] = { RollType::WHEEL, next };
        }
    }
}

void Mesh::classifyFaces()
{
    for (const Face& f : _hull->faces()) {
        std::vector<Vertex> vertices;
        for (Vertex v : f.adjacentVertices()) vertices.push_back(v);

        std::vector<Edge> edges;
        for (Edge e : f.adjacentEdges()) edges.push_back(e);

        const Vector3& vA = _hullGeom->vertexPositions[vertices[0]];
        const Vector3& vB = _hullGeom->vertexPositions[vertices[1]];
        const Vector3& vC = _hullGeom->vertexPositions[vertices[2]];

        Eigen::Vector3d A{ vA.x, vA.y, vA.z };
        Eigen::Vector3d B{ vB.x, vB.y, vB.z };
        Eigen::Vector3d C{ vC.x, vC.y, vC.z };
        std::vector<Eigen::Vector3d> pos{ A, B, C };

        Eigen::Hyperplane plane = Eigen::Hyperplane<double, 3>::Through(A, B, C);
        Eigen::Vector3d P = plane.projection({ _com.x, _com.y, _com.z });
        Eigen::Vector3d crossProd = (B - A).cross(C - A);
        double ABC = 0.5 * crossProd.norm();

        // 2. Calculate barycentric coordinates using sub-triangle areas
        // Sub-triangles:       (P, A, B), (P, B, C), (P, C, A)
        // double ABC   = 0.5 * (B - A).cross(C - A).norm();
        double alpha    = 0.5 * (A - P).cross(B - P).norm() / ABC;
        double beta     = 0.5 * (B - P).cross(C - P).norm() / ABC;
        // double gamma = 0.5 * (C - P).cross(A - P).norm() / ABC;

        // 3. Check if P is inside by ensuring weights are positive and sum is less than 1
        if (alpha >= 0.0 && beta >= 0.0 && alpha + beta <= 1.0) {
            // NOTE: f is placeholder
            _faceRoll[f] = { RollType::STABLE, { f, { alpha, beta, 1.0 - alpha - beta } } };
        } else {
            // 4. Determine if hinge (F1) or wheel (F2)
            // It is a Hinge if it's within the Voronoi stripe of that edge
            double closest = std::numeric_limits<double>::infinity();

            for (int i = 0; i < 3; i++) {
                A = pos[i];
                B = pos[(i + 1) % 3];
                Eigen::Vector3d edgeDir = B - A;

                // Vector projection to find position along the infinite line of the edge
                Eigen::ParametrizedLine edge =
                        Eigen::ParametrizedLine<double, 3>::Through(A, edgeDir.normalized());
                Eigen::Vector3d pEdge = edge.projection(P);
                double t = (pEdge - A).norm() / edgeDir.norm();

                // If weight is between 0 and 1, projection is in stripe
                if (0.0 < t && t < 1.0) {
                    // rolls onto edge that subtends stripe
                    _faceRoll[f] = { RollType::HINGE, { edges[i], t } };
                    break;
                } else {
                    // rolls onto a vertex
                    double dist = (P - A).norm();
                    if (dist < closest) {
                        closest = dist;
                        _faceRoll[f] = { RollType::WHEEL, vertices[i] };
                    }
                }
            }

        }

    }
}
