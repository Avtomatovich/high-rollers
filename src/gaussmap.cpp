#include "gaussmap.h"


#include "unionfind.h"

#include "geometrycentral/utilities/utilities.h"

GaussMap::GaussMap(Mesh& mesh) :
    _hull(mesh.getHull()),
    _geom(mesh.getHullGeom()),
    _c(mesh.getCenterOfMass()),
    _edgeRoll(mesh.getEdgeRoll()),
    _faceRoll(mesh.getFaceRoll())
{
    computeMappings();
    computeArcNormals();
    computeMinima();
    computeMaxima();
    computeSaddles();
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
    return unit({factor * std::cos(phi), factor * std::sin(phi), 1.0 - 2.0 * r2 });
}


// // GRADIENT TRACING

std::vector<TraceStep> GaussMap::traceGradient(const Vector3& n0, bool debug)
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

    N.push_back({elem, n0}); // Initialize with the first orientation

    // n <- n0
    Vector3 n = n0;

    // init previous element
    SurfacePoint prev;

    // define visited lambda
    auto visited = [&]() {
        return (prev.type != elem.type) ?
                    false : (elem.type == SurfacePointType::Edge) ?
                        prev.edge == elem.edge : (elem.type == SurfacePointType::Face) ?
                            prev.face == elem.face : prev.vertex == elem.vertex;
    };

    // define rolling lambda
    auto rolling = [&]() {
        return (elem.type == SurfacePointType::Edge) ?
                    _edgeRoll[elem.edge] : (elem.type == SurfacePointType::Face) ?
                        _faceRoll[elem.face] : Roll{ RollType::WHEEL, elem };
    };

    if (debug) std::cout << "Start trace" << std::endl;

    // init step counter
    int i = 0;

    // while elem is not stable face do
    for (Roll roll = rolling(); roll.type != RollType::STABLE; roll = rolling()) {

        if (debug) {
            std::cout << "  STEP " << i++ << std::endl;
            std::cout << "     prev normal: " << n << std::endl;
            std::cout << "    prev element: " << elem << std::endl;
        }

        // update previous element
        prev = elem;

        // if elem is hinge-type then
        if (roll.type == RollType::HINGE) { // Hinge edge or face

            if (debug) std::cout << "  HINGE ROLL" << std::endl;

            // elem <- NextFace(elem)
            elem = nextFace(roll.next, n); // Face that elem hinge-rolls onto
            // n <- Normal(elem)
            n = _geom.faceNormals[elem.face]; // A face normal
            // N = N union {n}
            N.push_back({elem, n});

        } else { // elem is a vertex, or a cartwheel-type edge/face

            if (debug) std::cout << "  WHEEL ROLL" << std::endl;

            // elem <- NextVertex(elem)
            elem = roll.next;
            // for elemAdj in AdjEdges(elem) do
            for (const Edge& adjEdge : elem.vertex.adjacentEdges()) { // Neighboring edges
                // f1, f2 <- adjFaces(elemAdj)
                const Face& f1 = adjEdge.halfedge().face();
                const Face& f2 = adjEdge.halfedge().twin().face();
                // nf1, nf2 <- Normal(f1), Normal(f2)
                const Vector3& nf1 = _geom.faceNormals[f1];
                const Vector3& nf2 = _geom.faceNormals[f2];
                // Move along the gradient arc until elem's Gauss image boundary
                // nNext <- RayArcInt(n*Elem, n, nf1, nf2)
                Vector3 nNext = rayArcInt(_geom.vertexNormals[elem.vertex], n, nf1, nf2);
                if (nNext != Vector3::zero()) { // Intersection; next normal found
                    // n <- nNext
                    n = nNext;
                    // elem = ElementWithNormal(n);
                    elem = elementWithNormal(n);
                    // N = N union {n}
                    N.push_back({elem, n});
                    break; // don't check other neighbor edges
                }
            }
        }

        if (debug) {
            std::cout << "     curr normal: " << n << std::endl;
            std::cout << "    curr element: " << elem << std::endl << std::endl;
        }

        // Pop last element and exit loop if visited
        if (visited()) {
            if (debug) {
                std::cout << "    revisited: " << elem << std::endl;
                std::cout << "    EXIT" << std::endl;
            }
            N.pop_back();
            break;
        }

    }

    if (debug) {
        std::cout << "End trace" << std::endl << std::endl;

        std::cout << "Start path output" << std::endl;
        for (const TraceStep& s : N) {
            std::cout << "    normal: " << s.n << std::endl;
            std::cout << "   element: " << s.elem << std::endl << std::endl;
        }
        std::cout << "End path output" << std::endl << std::endl;
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

    // return arbitrary face if faces are coplanar
    if (isCoplanar(nf1, nf2)) return { f1, Vector3::constant(RECIP_3) };

    // fetch vertex endpoint coords
    const Vector3& x1 = _geom.vertexPositions[e.firstVertex()];
    const Vector3& x2 = _geom.vertexPositions[e.secondVertex()];

    // compute gradient (negate partial derivative of U wrt n)
    Vector3 grad = 0.5 * (x1 - _c) + 0.5 * (x2 - _c);

    // extract tangential component of gradient wrt current normal
    grad = grad - dot(grad, n) * n;

    // return face based on boolean check
    return SurfacePoint{(dot(nf1 - n, grad) >= 0.0) ? f1 : f2,
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
    Vector3 dn2 = cross(d, n2), dn1 = cross(d, n1);
    if (dot(dn2, n1n2) >= 0.0 && dot(dn1, n2n1) >= 0.0) {
        nInt =  d; //  d inside the arc
    }
    if (dot(-dn2, n1n2) >= 0.0 && dot(-dn1, n2n1) >= 0.0) {
        nInt = -d; // -d inside the arc
    }
    return nInt;
}


// // BOUNDARY CHECKING

bool GaussMap::onGaussEdge(const Edge& e, const Vector3& n)
{
    // fetch adjacent face normals
    const Face& f1 = e.halfedge().face();
    const Face& f2 = e.halfedge().twin().face();
    const Vector3& nf1 = _geom.faceNormals[f1];
    const Vector3& nf2 = _geom.faceNormals[f2];
    // compute great circle normal and possible normal with n
    Vector3 nc = unit(cross(nf1, nf2)), ns = unit(cross(nf1, n));
    // return false if faces are coplanar
    if (!isfinite(nc)) return false;
    // return false if at first face's normal, i.e. Gauss edge endpoint
    if (!isfinite(ns)) {
        std::cout << "Edge n " << n << " == " << " face n " << nf1 << std::endl;
        return false;
    }
    // return false if point is not on great circle
    if (1.0 - std::abs(dot(nc, ns)) > EPS) return false;
    // compute slerp weight
    // double t = angle(nf1, n) / _geom.edgeDihedralAngles[e];
    double t = angle(nf1, n) / angle(nf1, nf2);
    // return true if slerp weight is between 0 and 1
    return 0.0 < t && t < 1.0;
}

bool GaussMap::onGaussPatch(const Vertex& v, const Vector3& n)
{
    // fetch arc normals
    const std::vector<Vector3>& na = _arcNormals[v];
    // return false if no patch exists
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


// // MORSE-SMALE COMPLEX

std::vector<Separatrix> GaussMap::buildSeparatrix(bool debug)
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

    // SE <- SaddleEdges(H)
    std::unordered_map<Edge, Vector3>& SE = _saddles;

    // for e0 in SE do
    for (const auto& [e0, n0] : SE) {
        // (f1, f2) <- AdjFaces(e0)
        const Face& f1 = e0.halfedge().face(); // Two faces neighboring e0
        const Face& f2 = e0.halfedge().twin().face();

        // f*1 <- DestinedFace(f1)
        // f*2 <- DestinedFace(f2)
        Face fs1 = f1, fs2 = f2; // Resting faces starting from f1, f2
        if (!destinedFace(fs1)) continue;
        if (!destinedFace(fs2)) continue;

        // if f*1 == f*2 then
        //     break
        if (fs1 == fs2 || _toGaussFace[fs1] == _toGaussFace[fs2]) { // Not dividing two different basins
            continue;
        }

        // n <- n*e0
        Vector3 n = n0;
        // nNext <- 0
        Vector3 nNext = Vector3::zero();

        // for pi in AdjVertices(e0) do
        for (const Vertex& pi : e0.adjacentVertices()) { // Two vertices adjacent to e
            // p <- pi
            Vertex p = pi;

            // // init visited set of vertices
            // std::unordered_set<Vertex> visited;
            // // init trace list
            // std::vector<Vertex> trace;

            // while True do
            while (true) {
                // if p is maximum vertex then
                if (_maxima.contains(p)) {
                    // nNext <- n*p
                    nNext = _maxima[p];
                    // BN <- BN union (n, nNext, f*1, f*2)
                    BN.push_back({n, nNext, fs1, fs2});

                    // if (debug) {
                    //     trace.push_back(p);
                    //     int i = 0;
                    //     std::cout << "With maxima" << std::endl;
                    //     for (const auto& v : trace) {
                    //         std::cout << "Vertex " << i++ << ": " << v << std::endl;
                    //     }
                    //     std::cout << std::endl << std::endl;
                    // }

                    // break
                    break;
                }

                // create found bool
                // bool found = false;
                // for e in AdjEdges(p) do
                for (const Edge& e : p.adjacentEdges()) {
                    // // break if visited
                    // if (visited.contains(p)) break;
                    // // mark vertex as visited
                    // visited.insert(p);
                    // if (debug) trace.push_back(p);
                    // f, f' <- AdjFaces(e)
                    const Face& f = e.halfedge().face();
                    const Face& fp = e.halfedge().twin().face();
                    // nf, nf' <- Normal(f), Normal(f')
                    const Vector3& nf  = _geom.faceNormals[f];
                    const Vector3& nfp = _geom.faceNormals[fp];
                    // nNext <- ArcsInt(n, nVertex, nf, nf')
                    nNext = rayArcInt(n, _geom.vertexNormals[p], nf, nfp);
                    // if nNext != 0 then
                    if (nNext != Vector3::zero()) { // Intersection happens
                        // BN <- BN union (n, nNext, f*1, f*2)
                        BN.push_back({n, nNext, fs1, fs2});
                        // n <- nNext
                        n = nNext;
                        // p = OtherVertex(e, p)
                        p = e.otherVertex(p);
                        // mark as found
                        // found = true;
                        // break
                        break;
                    }
                }
                // break if no separatrix found
                // if (!found) {

                //     // if (debug) {
                //     //     int i = 0;
                //     //     std::cout << "Without maxima" << std::endl;
                //     //     for (const auto& v : trace) {
                //     //         std::cout << "Vertex " << i++ << ": " << v << std::endl;
                //     //     }
                //     //     std::cout << std::endl << std::endl;
                //     // }

                //     break;
                // }
                // if (!found) break;
                // break;
            }
        }
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
    // fetch last element from path
    const auto& [elem, _] = path.back();
    // return false if element is not face
    if (elem.type != SurfacePointType::Face) return false;
    // return false if face is not stable
    if (_faceRoll[elem.face].type != RollType::STABLE) return false;
    // update ref with stable face and return true
    f = elem.face;
    return true;
}


// // PRE-COMPUTING

void GaussMap::computeMappings()
{
    // link hull-to-Gauss face map to hull mesh
    _toGaussFace = FaceData<Face>(_hull);

    // link Gauss-to-hull face map to hull mesh
    _toHullFace = FaceData<std::vector<Face>>(_hull);

    // init union find data structure
    UnionFind partition(_hull.nFaces());

    for (const Face& fi : _hull.faces()) {
        // map face to self in hull-to-Gauss face map
        _toGaussFace[fi] = fi;
        // fetch face index and normal
        size_t i = _geom.faceIndices[fi];
        const Vector3& ni = _geom.faceNormals[fi];
        // for each adjacent face
        for (const Face& fj : fi.adjacentFaces()) {
            // fetch adjacent face index and normal
            size_t j = _geom.faceIndices[fj];
            const Vector3& nj = _geom.faceNormals[fj];
            // join faces i and j if coplanar
            if (isCoplanar(ni, nj)) partition.join(i, j);
        }
    }

    for (const Face& f : _hull.faces()) {
        // fetch face index
        size_t i = _geom.faceIndices[f];
        // if face is singleton, i.e. not parent
        if (partition.size(i) == 1) {
            // find parent face index
            size_t p = partition.find(i);
            // fetch parent face
            Face fp(&_hull, p);
            // map curr face to parent face
            _toGaussFace[f] = fp;
            // add curr face as child of parent face
            _toHullFace[fp].push_back(f);
        }
    }
}

void GaussMap::computeArcNormals()
{
    // link list of great circle normals to hull
    _arcNormals = VertexData<std::vector<Vector3>>(_hull);

    for (const Vertex& v : _hull.vertices()) {
        // init list of face normals
        std::vector<Vector3> nf;
        nf.reserve(v.faceDegree());

        // std::cout << "Current vertex " << v << std::endl;
        for (const Face& f : v.adjacentFaces()) {
            // std::cout << "    Original face " << f << std::endl;
            // std::cout << "    Original normal " << _geom.faceNormals[f] << std::endl;
            // std::cout << "    Gauss face " << _toGaussFace[f] << std::endl;
            // std::cout << "    Gauss normal " << _geom.faceNormals[_toGaussFace[f]] << std::endl << std::endl;
            nf.push_back(_geom.faceNormals[_toGaussFace[f]]);
        }

        // fetch ref to list of great circle normals
        std::vector<Vector3>& np = _arcNormals[v];
        np.reserve(v.faceDegree());

        if (nf.size() >= 3) {
            Vector3 na;
            for (size_t i = 0; i < nf.size(); i++) {
                const Vector3& ni = nf[i];
                const Vector3& nj = nf[(i + 1) % nf.size()];
                na = unit(cross(ni, nj));
                if (isfinite(na)) np.push_back(na);
            }
        }

        if (np.size() < 3) np.clear();

        // // init face normal vars
        // Vector3 na, nf0, nf1, nf2;
        // nf0 = Vector3::undefined();

        // // get cross prods of adjacent face normals
        // for (const Face& f : v.adjacentFaces()) {
        //     // init first face normal if undefined
        //     if (nf0 == Vector3::undefined()) {
        //         nf0 = nf1 = _geom.faceNormals[f];
        //         continue;
        //     }
        //     // get current face normal
        //     nf2 = _geom.faceNormals[f];
        //     // add great circle normal if no coplanarity
        //     na = unit(cross(nf1, nf2));
        //     if (isfinite(na)) np.push_back(na);
        //     // update latest normals
        //     nf1 = nf2;
        // }
        // // add last great circle normal if no coplanarity
        // na = unit(cross(nf2, nf0));
        // if (isfinite(na)) np.push_back(na);

        // std::cout << "Arc normals of " << v << " at " << _geom.vertexPositions[v] << std::endl;
        // std::cout << "Num of arc normals: " << np.size() << std::endl;
        // for (const Vector3& a : np) {
        //     std::cout << "    " << a << std::endl;
        // }
        // std::cout << std::endl;
    }
}

void GaussMap::computeMinima()
{
    // init all stable face areas to zero
    for (const Face& f : _hull.faces()) {
        if (_faceRoll[f].type == RollType::STABLE) _minima[f] = 0.0;
    }
}

void GaussMap::computeMaxima()
{
    for (const Vertex& v : _hull.vertices()) {
        // compute direction vector from center of mass to hull vertex
        Vector3 nj = unit(_geom.vertexPositions[v] - _c);
        if (!isfinite(nj)) {
            std::cout << "Local maximum is NaN" << std::endl;
            continue;
        }
        // add maxima if on Gauss patch of vertex
        if (onGaussPatch(v, nj)) _maxima[v] = nj;
    }

    std::cout << "No of maxima: " << _maxima.size() << std::endl;
}

void GaussMap::computeSaddles()
{
    for (const Edge& e : _hull.edges()) {
        // if edge is a hinge type
        if (_edgeRoll[e].type == RollType::HINGE) {
            // fetch adjacent face normals
            const Face& fi = e.halfedge().face();
            const Face& fj = e.halfedge().twin().face();
            // skip if faces are coplanar
            if (_toGaussFace[fi] == _toGaussFace[fj]) continue;
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

    std::cout << "No of saddle points: " << _saddles.size() << std::endl;
}

void GaussMap::computeProb()
{
    std::vector<Separatrix> separatrices = buildSeparatrix();
    // std::vector<Separatrix> separatrices = buildSeparatrix(true);

    int i = 0;
    // accumulate signed spherical triangle areas for stable faces
    for (const Separatrix& s : separatrices) {
        std::cout << "Separatrix " << i++ << ": " << std::endl;
        std::cout << "    Face f1: " << s.f1 << std::endl;
        std::cout << "    Face f2: " << s.f2 << std::endl;
        std::cout << "    Normal n1: " << s.n1 << std::endl;
        std::cout << "    Normal n2: " << s.n2 << std::endl;
        const Vector3& nf1 = _geom.faceNormals[s.f1];
        const Vector3& nf2 = _geom.faceNormals[s.f2];
        _minima[s.f1] += spheTriArea(nf1, s.n1, s.n2);
        _minima[s.f2] += spheTriArea(nf2, s.n1, s.n2);
        // std::cout << "Area for " << s.f1 << ": " << _minima[s.f1] << std::endl;
        // std::cout << "Area for " << s.f2 << ": " << _minima[s.f2] << std::endl;
    }

    std::cout << "Odds of stability for each hull face: " << std::endl;

    double sum = 0.0;
    // normalize area sums to get probabilities
    for (const auto& [f, a] : _minima) {
        double area = a;
        area /= SPHERE_AREA;
        sum += area;
        std::cout << "    Prob of face " << f << ": " << area << std::endl;
    }

    std::cout << "Sum of probs: " << sum << std::endl << std::endl;

    // reset after probability computation
    computeMinima();
}
