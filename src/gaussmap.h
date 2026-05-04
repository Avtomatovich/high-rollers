#ifndef GAUSSMAP_H
#define GAUSSMAP_H

#include "mesh.h"
#include <unordered_map>
#include <vector>

struct Separatrix {
    Vector3 n1, n2;
    Face f1, f2;
};

class GaussMap {
public:
    GaussMap(Mesh& mesh);

    Vector3 randomGaussNormal();
    std::vector<TraceStep> traceGradient(const Vector3& n0);
    SurfacePoint elementWithNormal(const Vector3& n);
    void computeProb();
    void visualizeGaussMap();

private:
    // ── references into Mesh — no copies, all share the same SurfaceMesh instance ──
    ManifoldSurfaceMesh&          _hull;
    const VertexPositionGeometry& _geom;
    const EdgeData<Roll>&         _edgeRoll;
    const FaceData<Roll>&         _faceRoll;

    // ── owned by GaussMap, but constructed against _hull ──
    Vector3                              _c;
    VertexData<std::vector<Vector3>>     _arcNormals;
    std::unordered_map<Face,   double>   _minima;
    std::unordered_map<Vertex, Vector3>  _maxima;
    std::unordered_map<Edge,   Vector3>  _saddles;

    SurfacePoint nextFace(const SurfacePoint& next, const Vector3& n);
    Vector3      rayArcInt(const Vector3& ns, const Vector3& n,
                      const Vector3& n1, const Vector3& n2);
    bool onGaussEdge(const Edge& e, const Vector3& n);
    bool onGaussPatch(const Vertex& v, const Vector3& n);

    void computeArcNormals();
    void computeMinima();
    void computeMaxima();
    void computeSaddles();

    std::vector<Separatrix> buildSeparatrix();
    bool destinedFace(Face& f);

    double spheTriArea(const Vector3& a, const Vector3& b, const Vector3& c);

    static constexpr double EPS         = 1e-6;
    static constexpr double RECIP_3     = 1.0 / 3.0;
    static constexpr double SPHERE_AREA = 4.0 * M_PI;
};

#endif // GAUSSMAP_H
