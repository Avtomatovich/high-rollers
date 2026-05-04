#include "gaussmap.h"



#include "geometrycentral/utilities/utilities.h"

GaussMap::GaussMap(Mesh& mesh) :
    _hull(mesh.getHull()),
    _geom(mesh.getHullGeom()),
    _c(mesh.getCenterOfMass()),
    _edgeRoll(mesh.getEdgeRoll()),
    _faceRoll(mesh.getFaceRoll())
{
    computeArcNormals();
    computeMinima();
    computeMaxima();
    computeSaddles();

    // mapping from hull face to Gauss map vertex
    hullToDual = FaceData<size_t>(_hull);
}

Vector3 GaussMap::randomGaussNormal()
{
    // generate random numbers within [0, 1]
    double r1 = unitRand(), r2 = unitRand();
    // compute azimuthal (horizontal) angle
    double phi = 2.0 * std::numbers::pi * r1;
    // compute xy-scaling factor based on theta = acos(1 - 2 * r2)
    double factor = 2.0 * std::sqrt(r2 * (1.0 - r2));
    // sample random point on Gauss map (i.e. normal)
    return {factor * std::cos(phi), factor * std::sin(phi), 1.0 - 2.0 * r2 };
}

std::vector<TraceStep> GaussMap::traceGradient(const Vector3& n0)
{
    // INPUT: A unit vector n0 as the initial orientation, a convex shape H
    //          uniquely determined by a point set {p1, p2, ..., pn} in R3,
    //          a point c in R3 inside H determining the center of mass.
    // OUTPUT: A sequence of unit vectors N = {n0, n2, n4, ..., nk} where
    //          ni, nj, where j = i + 1, determines a great arc segment that
    //          is along the gradient flow of U.

    std::vector<TraceStep> N;


    // Get the unique face, edge, or vertex that has n0 as its normal,
    //      prioritizing faces, then edges, when the normal is shared.
    // elem <- ElementWithNormal(n0)
    SurfacePoint elem = elementWithNormal(n0);

    // Initialize with the first orientation
    N.emplace_back(elem, n0);

    // n <- n0
    Vector3 n = n0;

    // define rolling lambda
    auto rolling = [&]() {
        Vertex next;
        if (elem.type == SurfacePointType::Vertex) {
            for (Edge e : elem.vertex.adjacentEdges()) {
                next = e.otherVertex(elem.vertex);
                break;
            }
        }
        return (elem.type == SurfacePointType::Edge) ?
                    _edgeRoll[elem.edge] : (elem.type == SurfacePointType::Face) ?
                        _faceRoll[elem.face] : Roll{ RollType::WHEEL, next };
    };

    // while elem is not stable face do
    for (Roll roll = rolling(); roll.type != RollType::STABLE; roll = rolling()) {
        // Hinge edge or face
        // if elem is hinge-type then
        if (roll.type == RollType::HINGE) {
            // Face that elem hinge-rolls onto
            // elem <- NextFace(elem)
            elem = nextFace(roll.next, n);
            // A face normal
            // n <- Normal(elem)
            n = _geom.faceNormals[elem.face];
            // N = N union {n}
            N.emplace_back(elem, n);

            // std::cout << "newly added normal: " << n << " for element " << elem << std::endl;
        } else { // elem is a vertex, or a cartwheel-type edge/face
            // elem <- NextVertex(elem)
            elem = roll.next;
            // Neighboring edges
            // for elemAdj in AdjEdges(elem) do
            for (const Edge& adjEdge : elem.vertex.adjacentEdges()) {
                // f1, f2 <- adjFaces(edgeAdj)
                const Face& f1 = adjEdge.halfedge().face();
                const Face& f2 = adjEdge.halfedge().twin().face();
                // nf1, nf2 <- Normal(f1), Normal(f2)
                const Vector3& nf1 = _geom.faceNormals[f1];
                const Vector3& nf2 = _geom.faceNormals[f2];
                // skip if coplanar
                if (1.0 - dot(nf1, nf2) <= EPS) continue;
                // Move along the gradient arc until elem's Gauss image boundary
                // nNext <- RayArcInt(n*Elem, n, nf1, nf2)
                Vector3 nNext = rayArcInt(_geom.vertexNormals[elem.vertex], n, nf1, nf2);
                // Intersection; next normal found
                if (nNext != Vector3::zero()) {
                    // n <- nNext
                    n = nNext;
                    // N = N union {n}
                    N.emplace_back(elem, n);

                    // std::cout << "newly added normal: " << n << " for element " << elem << std::endl;
                    // elem = ElementWithNormal(n);
                    elem = elementWithNormal(n);
                    // don't check other neighbor edges
                    break;
                }
            }
        }
    }

    return N;
}

SurfacePoint GaussMap::elementWithNormal(const Vector3& n)
{
    // face normal check
    for (const Face& f : _hull.faces()) {
        if (norm(_geom.faceNormals[f] - n) <= EPS) {
            return { f, Vector3::constant(RECIP_3) };
        }
    }

    // edge normal check (between face normals)
    for (const Edge& e : _hull.edges()) {
        if (onGaussEdge(e, n)) return { e, 0.5 };
    }

    // vertex normal check
    for (const Vertex& v : _hull.vertices()) {
        if (onGaussPatch(v, n)) return { v };
    }

    return {};
}

SurfacePoint GaussMap::nextFace(const SurfacePoint& next,
                                const Vector3& n)
{
    // fetch hinging edge
    Edge e = next.edge;
    
    // fetch adjacent faces
    const Face& f1 = e.halfedge().face();
    const Face& f2 = e.halfedge().twin().face();
    
    // fetch adjacent face normals
    const Vector3& nf1 = _geom.faceNormals[f1];
    const Vector3& nf2 = _geom.faceNormals[f2];

    // compute great circle normal
    Vector3 n1n2 = unit(cross(nf1, nf2));

    // return arbitrary face if faces are coplanar
    if (!isfinite(n1n2)) return { f1, Vector3::constant(RECIP_3) };

    // compute tangent of current normal and great circle normal
    Vector3 t = cross(n, n1n2);

    // fetch vertex endpoint coords
    const Vector3& x1 = _geom.vertexPositions[e.firstVertex()];
    const Vector3& x2 = _geom.vertexPositions[e.secondVertex()];

    // compute gradient (negate partial derivative of U wrt n)
    Vector3 grad = 0.5 * (x1 - _c) + 0.5 * (x2 - _c);

    // extract tangential component of gradient wrt current normal
    grad = grad - dot(grad, n) * n;

    // TODO: double-check this logic
    // check if gradient points towards nf1
    bool toF1 = (dot(nf1 - n, t) >= 0.0) == true
            && (dot(grad, t) >= 0.0) == true;

    // return face based on boolean check
    return SurfacePoint{toF1 ? f1 : f2,
                        Vector3::constant(RECIP_3)};
}

Vector3 GaussMap::rayArcInt(const Vector3& ns,
                            const Vector3& n,
                            const Vector3& n1,
                            const Vector3& n2)
{
    // Intersect an arc-ray that starts at n* and moves towards n,
    //      and the arc from n1 to n2.

    Vector3 nInt = Vector3::zero();
    Vector3 n1n2 = cross(n1, n2), n2n1 = -n1n2;
    // d is on the intersection of two great circles containing the input arcs
    Vector3 d = unit(cross(cross(ns, n), n1n2));
    // return early if intersection degenerates
    if (!isfinite(d)) return nInt;

    Vector3 dn2 = cross(d, n2), dn1 = cross(d, n1);
    if (dot(dn2, n1n2) >= 0.0 && dot(dn1, n2n1) >= 0.0) {
        nInt =  d; //  d inside the arc
    }
    if (dot(-dn2, n1n2) >= 0.0 && dot(-dn1, n2n1) >= 0.0) {
        nInt = -d; // -d inside the arc
    }
    return nInt;
}

bool GaussMap::onGaussEdge(const Edge& e, const Vector3& n)
{
    // fetch adjacent face normals
    const Face& f1 = e.halfedge().face();
    const Face& f2 = e.halfedge().twin().face();
    const Vector3& nf1 = _geom.faceNormals[f1];
    const Vector3& nf2 = _geom.faceNormals[f2];
    // compute great circle normal and possible normal with n
    Vector3 n1n2 = unit(cross(nf1, nf2)), n1n = unit(cross(nf1, n));
    // return false if great circle normal degenerates, i.e. coplanar
    if (!isfinite(n1n2)) return false;
    // return false if possible normal degenerates, i.e. n is at n1
    if (!isfinite(n1n)) {
        std::cout << "Normal n=" << n << " is at nf1=" << nf1 << std::endl;
        return false;
    }
    // return false if point is not on great circle
    if (1.0 - std::abs(dot(n1n2, n1n)) > EPS) return false;
    // compute slerp weight
    // double t = angle(nf1, n) / _geom.edgeDihedralAngles[e];
    double t = angle(nf1, n) / angle(nf1, nf2);
    // return true if weight is between 0 and 1 and is finite
    return 0.0 < t && t < 1.0;
}

bool GaussMap::onGaussPatch(const Vertex& v, const Vector3& n)
{
    // fetch arc normals
    const std::vector<Vector3>& na = _arcNormals[v];
    // return false if patch has no boundaries
    if (na.empty()) return false;
    // check dot prod of n with arc normals
    bool orientation = dot(na[0], n) > 0.0;
    for (int i = 1; i < na.size(); i++) {
        if ((dot(na[i], n) > 0.0) != orientation) {
            return false;
        }
    }
    return true;
}

void GaussMap::computeArcNormals()
{
    // link list of arc normals to hull
    _arcNormals = VertexData<std::vector<Vector3>>(_hull);

    for (const Vertex& v : _hull.vertices()) {
        // fetch ref to list of great circle normals
        std::vector<Vector3>& np = _arcNormals[v];
        np.reserve(v.faceDegree());

        // init face normal vars
        Vector3 na, nf0, nf1, nf2;
        nf0 = Vector3::undefined();

        // get cross prods of adjacent face normals
        for (const Face& f : v.adjacentFaces()) {
            // init first normal if undefined
            if (nf0 == Vector3::undefined()) {
                nf0 = nf1 = _geom.faceNormals[f];
                continue;
            }
            // get current face normal
            nf2 = _geom.faceNormals[f];
            // add great circle normal to list if not coplanar
            na = unit(cross(nf1, nf2));
            if (isfinite(na)) np.push_back(na);
            // update latest normals
            nf1 = nf2;
        }
        // add final great circle normal if not coplanar
        na = unit(cross(nf2, nf0));
        if (isfinite(na)) np.push_back(na);
    }
}

void GaussMap::computeMinima()
{
    // init all stable face areas to zero
    for (const Face& f : _hull.faces()) {
        if (_faceRoll[f].type == RollType::STABLE) {
            _minima[f] = 0.0;
        }
    }
}

void GaussMap::computeMaxima()
{
    for (const Vertex& v : _hull.vertices()) {
        // compute normalized gradient
        Vector3 nj = unit(_geom.vertexPositions[v] - _c);
        if (!isfinite(nj)) continue;
        // add maxima if in Gauss patch of vertex
        if (onGaussPatch(v, nj)) _maxima[v] = nj;
    }
}

void GaussMap::computeSaddles()
{
    for (const Edge& e : _hull.edges()) {
        // if edge is a hinge type
        if (_edgeRoll[e].type == RollType::HINGE) {
            // fetch adjacent face normals
            const Face& fi = e.halfedge().face();
            const Face& fj = e.halfedge().twin().face();
            const Vector3& nfi = _geom.faceNormals[fi];
            const Vector3& nfj = _geom.faceNormals[fj];
            // skip if normals are identical (i.e. faces are coplanar)
            if (1.0 - dot(nfi, nfj) <= EPS) continue;
            // fetch endpoint coords
            const Vector3& xi = _geom.vertexPositions[e.firstVertex()];
            const Vector3& xj = _geom.vertexPositions[e.secondVertex()];
            // compute edge vector
            Vector3 vij = xj - xi;
            // project center of mass onto edge
            Vector3 cij = xi + dot(_c - xi, vij) / dot(vij, vij) * vij;
            // compute vector from projection to center of mass
            Vector3 nij = unit(_c - cij);
            if (!isfinite(nij)) continue;
            // add as saddle point if on Gauss edge
            if (onGaussEdge(e, nij)) _saddles[e] = nij;
        }
    }
}

std::vector<Separatrix> GaussMap::buildSeparatrix()
{
    // INPUT: A convex shape H uniquely determined by a point
    //          set {x1, x2, ..., xn} in R3, a point c in R3 inside H
    //          determining the center of mass.
    // OUTPUT: A set of quadruples BN = {(ni1, ni2, fi1, fi2)}_i, where
    //          ni1, ni2 are unit vectors and fi1, fi2 are two stable faces.
    //          The great arc connecting ni1, ni2 is on the boundary of the basins
    //          of attraction of stable faces fi1, fi2.

    // BN <- {}
    std::vector<Separatrix> BN;

    try {

        // SE <- SaddleEdges(H)
        std::unordered_map<Edge, Vector3>& SE = _saddles;

        // for e0 in SE do
        for (const auto& [e0, n0] : SE) {
            // Two faces neighboring e0
            // (f1, f2) <- AdjFaces(e0)
            const Face& f1 = e0.halfedge().face();
            const Face& f2 = e0.halfedge().twin().face();

            // Normals of neighboring faces
            const Vector3& nf1 = _geom.faceNormals[f1];
            const Vector3& nf2 = _geom.faceNormals[f2];

            // Faces on same point on Gauss map, i.e. coplanar
            if (1.0 - dot(nf1, nf2) <= EPS) continue;

            // Resting face starting from f1
            // f*1 <- DestinedFace(f1)
            // f*2 <- DestinedFace(f2)
            Face fs1 = f1, fs2 = f2;
            if (!destinedFace(fs1)) throw std::logic_error("No stable face from f1");
            if (!destinedFace(fs2)) throw std::logic_error("No stable face from f2");

            // Not dividing two different basins
            // if f*1 == f*2 then
            //     break
            if (fs1 == fs2) continue;

            // n <- n*e0
            Vector3 n = n0;
            // nNext <- 0
            Vector3 nNext = Vector3::zero();

            // Two vertices adjacent to e
            // for pi in AdjVertices(e0) do
            for (Vertex p : e0.adjacentVertices()) {
                // p <- pi
                // while True do
                while (true) {
                    // if p is maximum vertex then
                    if (_maxima.contains(p)) {
                        // nNext <- n*p
                        nNext = _maxima[p];
                        // BN <- BN union (n, nNext, f*1, f*2)
                        BN.emplace_back(n, nNext, fs1, fs2);
                        // break
                        break;
                    }

                    // for e in AdjEdges(p) do
                    for (const Edge& e : p.adjacentEdges()) {
                        // f, f' <- AdjFaces(e)
                        const Face& f = e.halfedge().face();
                        const Face& fp = e.halfedge().twin().face();
                        // nf, nf' <- Normal(f), Normal(f')
                        const Vector3& nf  = _geom.faceNormals[f];
                        const Vector3& nfp = _geom.faceNormals[fp];
                        // skip if coplanar
                        if (1.0 - dot(nf, nfp) <= EPS) continue;
                        // nNext <- ArcsInt(n, nVertex, nf, nf')
                        nNext = rayArcInt(n, _geom.vertexNormals[p], nf, nfp);
                        // Intersection happens
                        // if nNext != 0 then
                        if (nNext != Vector3::zero()) {
                            // BN <- BN union (n, nNext, f*1, f*2)
                            BN.emplace_back(n, nNext, fs1, fs2);
                            // n <- nNext
                            n = nNext;
                            // p = OtherVertex(e, p)
                            p = e.otherVertex(p);
                            // break
                            break;
                        }
                    }
                }
            }
        }

    } catch (std::exception& e) {
        std::cout << "buildSeparatrix | ERROR | " << e.what() << std::endl;
    }

    // return BN
    return BN;
}

bool GaussMap::destinedFace(Face& f)
{
    // return true if face is already stable
    if (_faceRoll[f].type == RollType::STABLE) return true;
    // fetch face normal
    const Vector3& nf = _geom.faceNormals[f];
    // trace path from given normal
    std::vector<TraceStep> path = traceGradient(nf);
    // fetch last surface point and normal from path
    const auto& [sf, nsf] = path.back();
    // return false if element is not face
    if (sf.type != SurfacePointType::Face) return false;
    // return false if face is not stable
    if (_faceRoll[sf.face].type != RollType::STABLE) return false;
    // update ref with stable face and return true
    f = sf.face;
    return true;
}

void GaussMap::computeProb()
{
    std::vector<Separatrix> separatrices = buildSeparatrix();

    // accumulate signed spherical triangle areas for stable faces
    for (const Separatrix& s : separatrices) {
        const Vector3& nf1 = _geom.faceNormals[s.f1];
        const Vector3& nf2 = _geom.faceNormals[s.f2];
        _minima[s.f1] += spheTriArea(nf1, s.n1, s.n2);
        _minima[s.f2] += spheTriArea(nf2, s.n1, s.n2);
    }

    std::cout << "Odds of stability for each hull face: " << std::endl;

    // normalize area sums to get probabilities
    for (const auto& [f, a] : _minima) {
        double area = a;
        area /= SPHERE_AREA;
        std::cout << "  Prob of face " << f << ": " << area << std::endl;
    }

    // reset after probability computation
    computeMinima();
}

double GaussMap::spheTriArea(const Vector3& a,
                             const Vector3& b,
                             const Vector3& c)
{
    double N = dot(a, cross(b, c));
    double D = norm(a) * norm(b) * norm(c) +
                dot(a, b) * norm(c) +
                dot(a, c) * norm(b) +
                dot(b, c) * norm(a);
    return 2.0 * atan2(N, D);
}
