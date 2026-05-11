#ifndef GAUSSMAP_H
#define GAUSSMAP_H

#include "mesh.h"


struct Separatrix
{
    Vector3 n1; // saddle point
    Vector3 n2; // local maximum
    Face f1, f2; // local minima
};

class GaussMap
{
public:
    GaussMap(Mesh& mesh);

    void computeProb();

    std::vector<TraceStep> traceGradient(const Vector3& n0, bool debug = false);

    SurfacePoint elementWithNormal(const Vector3& n);

    // Gauss map sampling helper
    Vector3 randomGaussNormal();

    // Gauss map visualizing helper
    void visualizeGaussMap();

private:
    // // vars
    ManifoldSurfaceMesh& _hull;
    const VertexPositionGeometry& _geom;

    const Vector3& _c;

    const EdgeData<Roll>& _edgeRoll;
    const FaceData<Roll>& _faceRoll;

    VertexData<std::vector<Vector3>> _arcNormals;

    std::unordered_map<Face, double> _minima;
    std::unordered_map<Vertex, Vector3> _maxima;
    std::unordered_map<Edge, Vector3> _saddles;

    FaceData<Face> _toGaussFace;
    FaceData<std::vector<Face>> _toHullFace;


    // // funcs
    std::vector<Separatrix> buildSeparatrix(bool debug = false);

    // ray-arc/arc-arc intersection routine
    Vector3 rayArcInt(const Vector3& ns,
                      const Vector3& n,
                      const Vector3& n1,
                      const Vector3& n2);

    // hinge-rolling helper
    SurfacePoint nextFace(const SurfacePoint& next,
                          const Vector3& n);

    // boundary checking helpers
    bool onGaussEdge(const Edge& e, const Vector3& n);
    bool onGaussPatch(const Vertex& v, const Vector3& n);

    // pre-computing helpers
    void computeMappings();
    void computeArcNormals();
    void computeMinima();
    void computeMaxima();
    void computeSaddles();

    // utility helpers
    inline double spheTriArea(const Vector3& a,
                              const Vector3& b,
                              const Vector3& c)
    {
        return 2.0 * atan2(dot(a, cross(b, c)),
                           1.0 + dot(a, b) + dot(a, c) + dot(b, c));
    }
    inline bool isCoplanar(const Vector3& ni,
                           const Vector3& nj)
    {
        // NOTE: assumes that vectors are normalized
        return 1.0 - dot(ni, nj) <= EPS;
    }
    inline Face gaussFace(const Face& f) 
    {
        return _toGaussFace[f];
    }
    inline Vector3 gaussNormal(const Face& f) 
    {
        return _geom.faceNormals[_toGaussFace[f]];
    }

    // destined stable face helper
    bool destinedFace(Face& f);


    // // constants
    static constexpr double EPS = 1e-8;
    static constexpr double SPHERE_AREA = 4.0 * std::numbers::pi;
    static constexpr double RECIP_3 = 1.0 / 3.0;
};

#endif // GAUSSMAP_H
