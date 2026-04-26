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
    return { factor * std::cos(phi), factor * std::sin(phi), 1.0 - 2.0 * r2 };
}

std::vector<Vector3> GaussMap::traceGradient(const Vector3& n0)
{
    // INPUT: A unit vector n0 as the initial orientation, a convex shape H
    //          uniquely determined by a point set {p1, p2, ..., pn} in R3,
    //          a point c in R3 inside H determining the center of mass.
    // OUTPUT: A sequence of unit vectors N = {n0, n2, n4, ..., nk} where
    //          ni, nj, where j = i + 1, determines a great arc segment that
    //          is along the gradient flow of U.

    std::vector<Vector3> N;

    // Initialize with the first orientation
    N.push_back(n0);

    // Get the unique face, edge, or vertex that has n0 as its normal,
    //      prioritizing faces, then edges, when the normal is shared.
    // elem <- ElementWithNormal(n0)
    SurfacePoint elem = elementWithNormal(n0);

    // n <- n0
    Vector3 n = n0;

    // define rolling lambda
    auto rolling = [&]() {
        return (elem.type == SurfacePointType::Edge) ?
                    _edgeRoll[elem.edge] : (elem.type == SurfacePointType::Face) ?
                        _faceRoll[elem.face] : Roll{ RollType::WHEEL, elem };
    };

    // while elem is not stable face do
    for (Roll roll = rolling(); roll.type != RollType::STABLE; roll = rolling()) {
        // Hinge edge or face
        // if elem is hinge-type then
        if (roll.type == RollType::HINGE) {
            // Face that elem hinge-rolls onto
            // elem <- NextFace(elem)
            elem = nextFace(elem, roll.next, n);
            // A face normal
            // n <- Normal(elem)
            n = _geom.faceNormals[elem.face];
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
                // Move along the gradient arc until elem's Gauss image boundary
                // nNext <- RayArcInt(n*Elem, n, nf1, nf2)
                Vector3 nNext = rayArcInt(_geom.vertexNormals[elem.vertex], n, nf1, nf2);
                // Intersection; next normal found
                if (nNext != Vector3::zero()) {
                    // n <- nNext
                    n = nNext;
                    // N = N union {n}
                    N.push_back(n);
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
        const Vector3& nf = _geom.faceNormals[f];
        if (norm(nf - n) <= EPS * std::min(norm(nf), norm(n))) {
            return SurfacePoint(f, Vector3::constant(1.0 / 3.0));
        }
    }

    // edge normal check (between face normals)
    for (const Edge& e : _hull.edges()) {
        if (onGaussEdge(e, n)) return SurfacePoint(e, 0.5);
    }

    // vertex normal check
    for (const Vertex& v : _hull.vertices()) {
        if (onGaussPatch(v, n)) return SurfacePoint(v);
    }

    return SurfacePoint();
}

SurfacePoint GaussMap::nextFace(const SurfacePoint& elem, 
                                const SurfacePoint& next, 
                                const Vector3& n)
{
    // fetch hinging edge
    Edge e = (elem.type == SurfacePointType::Edge) ? elem.edge : next.edge;
    
    // fetch adjacent faces
    const Face& f1 = e.halfedge().face();
    const Face& f2 = e.halfedge().twin().face();
    
    // fetch adjacent face normals
    const Vector3& nf1 = _geom.faceNormals[f1];
    const Vector3& nf2 = _geom.faceNormals[f2];

    // fetch vertex endpoint coords
    const Vector3& x1 = _geom.vertexPositions[e.firstVertex()];
    const Vector3& x2 = _geom.vertexPositions[e.secondVertex()];

    // compute gradients weighted by delta = 0.5 for boundary arc points
    Vector3 grad1 = 0.5 * (x1 - _c);
    Vector3 grad2 = 0.5 * (x2 - _c);

    // compute net gradient vector
    Vector3 grad = unit(grad1 + grad2);

    // return f1 if net gradient and vector from n to nf2 diverge, else f2
    return SurfacePoint{(dot(grad, unit(nf2 - n)) < 0.0) ? f1 : f2, 
                        Vector3::constant(1.0 / 3.0)};
}

Vector3 GaussMap::rayArcInt(const Vector3& ns,
                            const Vector3& n,
                            const Vector3& n1,
                            const Vector3& n2)
{
    // Intersect an arc-ray that starts at n* and moves towards n with
    //      the arc from n1 to n2.

    Vector3 nInt = Vector3::zero();
    Vector3 n1n2 = cross(n1, n2), n2n1 = cross(n2, n1);
    // d is on the intersection of two great circles containing the input arcs
    Vector3 d = cross(cross(ns, n), n1n2).normalize();
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
    // fetch face normal
    const Face& f1 = e.halfedge().face();
    const Vector3& nf1 = _geom.faceNormals[f1];
    // check angle between nf1 and n lies between nf1 and nf2
    return angle(nf1, n) < _geom.edgeDihedralAngles[e];
}

bool GaussMap::onGaussPatch(const Vertex& v, const Vector3& n)
{
    // fetch arc normals
    const std::vector<Vector3>& na = _arcNormals[v];
    // check dot prod of n with arc normals
    bool orientation = std::signbit(dot(na[0], n));
    for (int i = 1; i < na.size(); i++) {
        if (std::signbit(dot(na[i], n)) != orientation) {
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
        // init list of great circle normals
        std::vector<Vector3> np;
        np.reserve(v.faceDegree());

        // init face normal vars
        Vector3 nf0, nf1, nf2;
        nf0 = Vector3::undefined();

        // get cross prods of adjacent face normals
        for (const Face& f : v.adjacentFaces()) {
            // init starting normals if undefined
            if (nf0 == Vector3::undefined()) {
                nf0 = nf1 = _geom.faceNormals[f];
                continue;
            }
            // get current face normal
            nf2 = _geom.faceNormals[f];
            // add great circle normal to list
            np.push_back(cross(nf1, nf2));
            // update latest normals
            nf1 = nf2;
        }
        // add final great circle normal
        np.push_back(cross(nf2, nf0));

        // cache arc normals
        _arcNormals[v] = np;
    }
}

void GaussMap::computeMinima()
{
    // link list of per-face spherical polygon areas to hull
    _minima = FaceData<double>(_hull, -std::numeric_limits<double>::infinity());

    for (const Face& f : _hull.faces()) {
        if (_faceRoll[f].type == RollType::STABLE) {
            _minima[f] = 0.0;
        }
    }
}

void GaussMap::computeMaxima()
{
    // link list of maxima to hull
    _maxima = VertexData<Vector3>(_hull, Vector3::undefined());

    for (const Vertex& v : _hull.vertices()) {
        // compute normalized gradient
        Vector3 nj = unit(_geom.vertexPositions[v] - _c);
        // add maxima if in Gauss patch of vertex
        if (onGaussPatch(v, nj)) _maxima[v] = nj;
    }
}

void GaussMap::computeSaddles()
{
    // link list of saddle points to hull
    _saddles = EdgeData<Vector3>(_hull, Vector3::undefined());

    for (const Edge& e : _hull.edges()) {
        // if edge is a hinge type
        if (_edgeRoll[e].type == RollType::HINGE) {
            // fetch endpoint coords
            const Vector3& xi = _geom.vertexPositions[e.firstVertex()];
            const Vector3& xj = _geom.vertexPositions[e.secondVertex()];
            // compute edge vector
            Vector3 vij = xj - xi;
            // project center of mass onto edge
            Vector3 cij = xi + dot((_c - xi), vij) / dot(vij, vij) * vij;
            // compute vector from projection to center of mass
            Vector3 nij = _c - cij;
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

    // SE <- SaddleEdges(H)
    const EdgeData<Vector3>& SE = _saddles;

    // for e0 in SE do
    for (const Edge& e0 : _hull.edges()) {
        if (SE[e0] == Vector3::undefined()) continue;

        // Two faces neighboring e
        // (f1, f2) <- AdjFaces(e)
        const Face& f1 = e0.halfedge().face();
        const Face& f2 = e0.halfedge().twin().face();

        // Resting face starting from f1
        // f*1 <- DestinedFace(f1)
        // f*2 <- DestinedFace(f2)
        // WARNING: check this is correct, likely bugs from no type checks
        Face fs1 = elementWithNormal(traceGradient(_geom.faceNormals[f1]).back()).face;
        Face fs2 = elementWithNormal(traceGradient(_geom.faceNormals[f2]).back()).face;

        // Not dividing two different basins
        // if f*1 == f*2 then
        //     break
        if (fs1 == fs2) break;

        // n <- n*e0
        Vector3 n = SE[e0];
        // nNext <- 0
        Vector3 nNext = Vector3::zero();

        // Two vertices adjacent to e
        // for pi in AdjVertices(e0) do
        for (const Vertex& pi : e0.adjacentVertices()) {
            // p <- pi
            Vertex p = pi;
            // while True do
            while (true) {
                // if p is maximum vertex then
                if (_maxima[p] != Vector3::undefined()) {
                    // nNext <- n*p
                    nNext = _maxima[p];
                    // BN <- BN union (n, nNext, f*1, f*2)
                    BN.push_back({n, nNext, fs1, fs2});
                    // break
                    break;
                }
                // for e in AdjEdges(p) do
                for (const Edge& e : p.adjacentEdges()) {
                    // f, f' <- AdjFaces(e)
                    const Face& f = e0.halfedge().face();
                    const Face& fp = e0.halfedge().twin().face();
                    // nf, nf' <- Normal(f), Normal(f')
                    const Vector3& nf  = _geom.faceNormals[f];
                    const Vector3& nfp = _geom.faceNormals[fp];
                    // nNext <- ArcsInt(n, nVertex, nf, nf')
                    // WARNING: what is nVertex? which vertex?
                    nNext = rayArcInt(n, _geom.vertexNormals[p], nf, nfp);
                    // Intersection happens
                    // if nNext != 0 then
                    if (nNext != Vector3::zero()) {
                        // BN <- BN union (n, nNext, f*1, f*2)
                        BN.push_back({n, nNext, fs1, fs2});
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

    // return BN
    return BN;
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
    for (const Face& f : _hull.faces()) {
        if (_faceRoll[f].type == RollType::STABLE) {
            _minima[f] /= SPHERE_AREA;
            std::cout << "  Prob of face " << f << ": " << _minima[f] << std::endl;
        }
    }

    // reset after probability computation
    computeMinima();
}

double GaussMap::spheTriArea(const Vector3& a,
                             const Vector3& b,
                             const Vector3& c)
{
    return 2.0 * atan2(dot(a, cross(b, c)),
                       1.0 + dot(a, b) + dot(a, c) + dot(b, c));
}
