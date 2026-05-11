#include "mesh.h"

#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/surface_mesh_factories.h"
#include "geometrycentral/utilities/eigen_interop_helpers.h"

#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/point_cloud.h"
#include "polyscope/curve_network.h"

#include "util/quickhull/QuickHull.hpp"

#include "sim.h"

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

    // pre-compute face indices of hull
    _hullGeom->requireFaceIndices();

    std::cout << "No of hull vertices: " << _hull->nVertices() << std::endl;
    std::cout << "No of hull faces: " << _hull->nFaces() << std::endl;

    // compute center of mass if not provided
    if (computeCom) computeCenterOfMass();

    // classify all mesh elements
    classify();
}

void Mesh::visualizeEdgeTypes() 
{
    // clear buffers
    polyscope::removeStructure("wheel edges");
    polyscope::removeStructure("hinge edges");

    std::vector<glm::vec3> nodePositions;
    for (Vertex v : _hull->vertices()) {
        const Vector3& p = _hullGeom->vertexPositions[v];
        nodePositions.push_back({(float)p.x, (float)p.y, (float)p.z});
    }

    std::vector<std::array<size_t, 2>> wheelEdges, hingeEdges;

    for (const Edge& e : _hull->edges()) {
        size_t u = e.halfedge().vertex().getIndex();
        size_t v = e.halfedge().twin().vertex().getIndex();

        if (_edgeRoll[e].type == RollType::WHEEL) {
            wheelEdges.push_back({u, v});
        } else if (_edgeRoll[e].type == RollType::HINGE) {
            hingeEdges.push_back({u, v});
        }
    }

    // register data if non-empty
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

void Mesh::show()
{
    polyscope::registerSurfaceMesh("mesh",
                                   _meshGeom->vertexPositions,
                                   _mesh->getFaceVertexList());
    polyscope::registerSurfaceMesh("hull",
                                   _hullGeom->vertexPositions,
                                   _hull->getFaceVertexList());
    std::vector<glm::vec3> points;
    points.push_back({ _com.x, _com.y, _com.z });
    polyscope::PointCloud *psCloud = polyscope::registerPointCloud("center of mass", points);
    // polyscope::PointCloud *psCloud = polyscope::registerPointCloud("center of mass", {{ _com.x, _com.y, _com.z }});
    psCloud->setPointRadius(0.02);
    psCloud->setPointRenderMode(polyscope::PointRenderMode::Quad);

    std::vector<glm::vec3> faceColors;
    for (const Face& f: _hull->faces()) {
        if (_faceRoll[f].type == RollType::STABLE) {
            faceColors.push_back({1.0, 0.0, 0.0});
        } else if (_faceRoll[f].type == RollType::HINGE) {
            faceColors.push_back({0.0, 1.0, 0.0});
        } else if (_faceRoll[f].type == RollType::WHEEL) {
            faceColors.push_back({0.0, 0.0, 1.0});
        }
    }

    polyscope::getSurfaceMesh("hull")->addFaceColorQuantity("face types", faceColors);
    polyscope::getSurfaceMesh("hull")->setTransparency(0.5);
    visualizeEdgeTypes();
    
    polyscope::show();
}

void Mesh::show(const std::vector<TraceStep>& steps) 
{
    polyscope::init();

    for (size_t i = 0; i < steps.size(); i++) {
        std::string name = "mesh_step_" + std::to_string(i);
        auto* ps = polyscope::registerSurfaceMesh(name, _hullGeom->vertexPositions, _hull->getFaceVertexList());

        // 1. Position the mesh based on the normal
        ps->setTransform(eigenToGlm(normalToTransform(steps[i].n)));
        Eigen::Matrix4d T = normalToTransform(steps[i].n);
        // Shift each step 2 units to the right along the X axis
        T(0, 3) = i * 2.0;
        ps->setTransform(eigenToGlm(T));

        // 2. Prepare highlighting arrays
        // We initialize these to a "neutral" color (light gray)
        std::vector<glm::vec3> fColors(_hull->nFaces(), {0.95, 0.95, 0.95});
        std::vector<double> vWeights(_hull->nVertices(), 0.0);

        // 3. Highlight based on element type
        SurfacePoint p = steps[i].elem;
        if (p.type == SurfacePointType::Face) {
            fColors[p.face.getIndex()] = {1.0, 0.2, 0.2}; // bright red for face
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

    polyscope::show();
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
Eigen::Matrix4d Mesh::normalToTransform(const Vector3& norm)
{
    Eigen::Quaterniond q =
            Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitY(), {n.x, n.y, n.z});
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = q.toRotationMatrix();
    return T;
}

// Helper to turn an eigen matrix to a glm one
glm::mat4 Mesh::eigenToGlm(const Eigen::Matrix4d& T)
{
    glm::mat4 m;
    // Eigen = row-major, glm = col-major
    for (int col = 0; col < 4; col++)
        for (int row = 0; row < 4; row++)
            m[col][row] = static_cast<float>(T(row, col));
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

    // group coplanar faces
    buildCoplanarGroups(1e-6);
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

void Mesh::showFaceProbabilities()
{
    polyscope::init();
    
    if (_faceResults.empty()) {
        std::cerr << "[Mesh::showFaceProbabilities] No results yet - "
        std::cerr << "run BulletSimulation::runTrials() first.\n";
        return;
    }

    const size_t nFaces = _hull->nFaces();

    // build prob array indexed by face
    std::vector<double> probs(nFaces, 0.0);
    double pMax = 0.0;
    double pMin = std::numeric_limits<double>::infinity();

    for (const auto& r : _faceResults) {
        if (r.faceIndex < nFaces) {
            probs[r.faceIndex] = r.probability;
            if (r.probability > 0.0) {
                pMax = std::max(pMax, r.probability);
                pMin = std::min(pMin, r.probability);
            }
        }
    }
    if (pMax == 0.0) pMax = 1.0;
    if (pMin == pMax) pMin = 0.0;

    // build RGBs for heatmap
    auto lerp3 = [](glm::vec3 a, glm::vec3 b, float t) -> glm::vec3 {
        return a * (1.f - t) + b * t;
    };
    const glm::vec3 colGrey  = { 0.82f, 0.82f, 0.82f };
    const glm::vec3 colBlue  = { 0.12f, 0.35f, 0.92f };
    const glm::vec3 colGreen = { 0.18f, 0.78f, 0.18f };
    const glm::vec3 colRed   = { 0.92f, 0.12f, 0.08f };

    std::vector<glm::vec3> faceColors(nFaces, colGrey);
    for (size_t i = 0; i < nFaces; ++i) {
        double p = probs[i];
        if (p > 0.0) {
            float t = static_cast<float>((p - pMin) / (pMax - pMin));
            t = std::clamp(t, 0.f, 1.f);
            faceColors[i] = (t < 0.5f)
                                ? lerp3(colBlue,  colGreen, t * 2.f)
                                : lerp3(colGreen, colRed,   (t - 0.5f) * 2.f);
        }
    }

    // register hull
    auto* ps = polyscope::registerSurfaceMesh(
        "hull_probabilities",
        _hullGeom->vertexPositions,
        _hull->getFaceVertexList());
    ps->setSurfaceColor({0.82f, 0.82f, 0.82f}); // base colour (overridden below)
    ps->setTransparency(1.0f);
    ps->setSmoothShade(false); // flat shading makes face boundaries crisp

    // Color quantity: ENABLED so it actually shows on the mesh
    auto* colorQ = ps->addFaceColorQuantity("Probability Heatmap", faceColors);
    colorQ->setEnabled(true);

    // Scalar quantity: disabled by default but available for hover inspection
    auto* scalarQ = ps->addFaceScalarQuantity("Landing Probability (%)",
                                              [&]() {
                                                  std::vector<double> pct(nFaces);
                                                  for (size_t i = 0; i < nFaces; ++i) pct[i] = probs[i] * 100.0;
                                                  return pct;
                                              }());
    scalarQ->setColorMap("coolwarm");
    // Leave scalar disabled — color quantity takes visual priority

    // ── Face-centroid point cloud for probability labels ───────────────────────
    // Polyscope has no text rendering, but a point cloud with a scalar quantity
    // lets the user hover over a face centre to read the exact percentage,
    // and the scalar colour on the points echoes the face heatmap.
    std::vector<glm::vec3> centroids(nFaces);
    std::vector<double>    centroidProbs(nFaces);

    for (const Face& f : _hull->faces()) {
        size_t fi = f.getIndex();
        Vector3 c = {0, 0, 0};
        int vCount = 0;
        for (Vertex v : f.adjacentVertices()) {
            c += _hullGeom->vertexPositions[v];
            ++vCount;
        }
        c /= static_cast<double>(vCount);

        // offset centroid slightly outward along face normal so it sits
        // just above the surface and is not z-fighting with it
        const Vector3& fn = _hullGeom->faceNormals[f];
        c += fn * 0.01;

        centroids[fi]     = { static_cast<float>(c.x),
                              static_cast<float>(c.y),
                              static_cast<float>(c.z) };
        centroidProbs[fi] = probs[fi] * 100.0; // store as percentage
    }

    auto* cloud = polyscope::registerPointCloud("face_labels", centroids);
    cloud->setPointRadius(0.008);

    auto* labelQ = cloud->addScalarQuantity("Probability (%)", centroidProbs);
    labelQ->setColorMap("coolwarm");
    labelQ->setEnabled(true);

    // print ranked table
    std::cout << "\n=== Face Landing Probabilities ===\n";
    std::cout << std::fixed << std::setprecision(2);

    std::vector<size_t> order(nFaces);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(),
              [&](size_t a, size_t b) { return probs[a] > probs[b]; });

    int totalTrials = 0;
    for (const auto& r : _faceResults) totalTrials += r.landCount;
    std::cout << "  " << nFaces << " faces,  " << totalTrials << " trials\n";
    std::cout << "----------------------------------\n";
    for (size_t rank = 0; rank < nFaces; ++rank) {
        size_t fi = order[rank];
        if (probs[fi] == 0.0) break;
        std::cout << "  Face " << std::setw(3) << fi
                  << "  ->  " << std::setw(6) << (probs[fi] * 100.0) << " %"
                  << "   (" << _faceResults[fi].landCount << " hits)\n";
    }
    std::cout << "==================================\n\n";

    polyscope::show();
}

void Mesh::buildCoplanarGroups(double angleTolerance)
{
    const int nFaces = static_cast<int>(_hull->nFaces());
    _faceGroup.assign(nFaces, -1);
    _numGroups = 0;

    for (const Face& f : _hull->faces()) {
        int fi = static_cast<int>(f.getIndex());
        if (_faceGroup[fi] != -1) continue;

        // BFS flood-fill over coplanar neighbors
        std::queue<Face> queue;
        queue.push(f);
        _faceGroup[fi] = _numGroups;
        const Vector3& refN = _hullGeom->faceNormals[f];

        while (!queue.empty()) {
            Face cur = queue.front(); queue.pop();
            for (Halfedge he : cur.adjacentHalfedges()) {
                Face nb = he.twin().face();
                int ni = static_cast<int>(nb.getIndex());
                if (_faceGroup[ni] != -1) continue;
                const Vector3& nbN = _hullGeom->faceNormals[nb];
                double dot = refN.x * nbN.x + refN.y * nbN.y + refN.z * nbN.z;
                if (std::abs(dot - 1.0) < angleTolerance) {
                    _faceGroup[ni] = _numGroups;
                    queue.push(nb);
                }
            }
        }
        _numGroups++;
    }

    // precompute stable face normals for fast nearest-stable remapping
    // NOTE: used by identifyRestingFace to resolve Bullet stuck cases
    _stableFaceNormals.clear();
    _stableFaceIndices.clear();
    for (const Face& f : _hull->faces()) {
        if (_faceRoll[f].type == RollType::STABLE) {
            const Vector3& fn = _hullGeom->faceNormals[f];
            _stableFaceNormals.push_back({fn.x, fn.y, fn.z});
            _stableFaceIndices.push_back(static_cast<int>(f.getIndex()));
        }
    }
}
