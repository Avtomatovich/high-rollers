#ifndef GAUSSMAP_H
#define GAUSSMAP_H

#include "mesh.h"


struct Separatrix
{
    Vector3 n1, n2;
    Face f1, f2;
};

class GaussMap
{
public:
    GaussMap(Mesh& mesh);

    std::vector<Vector3> traceGradient(const Vector3& n0);

    // Gauss map sampling helper
    Vector3 randomGaussNormal();

private:
    ManifoldSurfaceMesh& _hull;
    const VertexPositionGeometry& _geom;

    const Vector3& _c;

    const EdgeData<Roll>& _edgeRoll;
    const FaceData<Roll>& _faceRoll;

    VertexData<std::vector<Vector3>> _arcNormals;

    std::vector<Face> _minima;
    VertexData<Vector3> _maxima;
    EdgeData<Vector3> _saddles;

    Vector3 rayArcInt(const Vector3& ns,
                      const Vector3& n,
                      const Vector3& n1,
                      const Vector3& n2);

    // TODO: move MS complex logic to MS class
    std::vector<Separatrix> buildSeparatrix();

    // bounds-checking helpers
    SurfacePoint elementWithNormal(const Vector3& n);
    bool onGaussEdge(const Edge& e, const Vector3& n);
    bool onGaussPatch(const Vertex& v, const Vector3& n);

    // pre-computing helpers
    void computeArcNormals();
    void computeMinima();
    void computeMaxima();
    void computeSaddles();
};

#endif // GAUSSMAP_H
