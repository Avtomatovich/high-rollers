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

Face nextFaceFromEdge(Edge e, const Eigen::Vector3d& nHat){

}

Vertex Mesh::nextVertex(SurfacePoint curr, Vector3 projectedCOM){

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
            Face next;

        } else {
            _edgeTypes[e] = RollType::WHEEL;
            //next vertex is closest to COM projection
            Vertex next = (v1 - projectedCom).norm() > (v2 - projectedCom).norm() ? b : a;
            _edgeRoll[e] = {RollType::WHEEL, next};
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
        std::vector<Edge> edges;
        for (Edge e : f.adjacentEdges()) {
            edges.push_back(e);
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
            Edge nextE;
            Vertex nextV;
            Roll r;
            double vertDist = INFINITY;

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
                    //sets the hinge edge to the one that is in this strip
                    nextE = edges[i];// TODO-- do we need to turn this into a face?
                    _faceRoll[f] = Roll(RollType::HINGE, SurfacePoint(edges[i], t));
                    break;
                }
                else{
                    double vToCOM = (projectedCom-a).norm();
                    //rolls onto a vertex -- which one? just the closest norm()?
                    if((projectedCom-a).norm()<vertDist){
                        vertDist = vToCOM;
                        nextV = vertices[i];
                    }
                }
            }
            _faceRoll[f] = Roll(RollType::WHEEL, nextV);
            _faceTypes[f] = inStripe ? RollType::HINGE : RollType::WHEEL;
        }
    }
}
