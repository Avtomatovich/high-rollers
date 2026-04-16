#ifndef GAUSSMAP_H
#define GAUSSMAP_H

#include "mesh.h"

#include <vector>

class GaussMap
{
public:
    GaussMap(const Mesh& mesh);

    std::vector<Vector3> traceGradient(const Vector3& n0);

private:
    const SurfaceMesh& _hull;
    const VertexPositionGeometry& _geom;

    const EdgeData<RollType>& _edgeTypes;
    const FaceData<RollType>& _faceTypes;

    // TODO: is explicit Morse-Smale complex necessary?
    // TODO:    perhaps partially... store min, max, saddle per elem
    // TODO:    explicit connectivity necessary for ascending manifold?

    // TODO: pre-compute all max vertices, saddle edges, min faces
    // TODO:    does this belong in the mesh class instead?

    // TODO: define this func signature better
    void buildSeparatrix();

    // TODO: is explicit dual graph necessary?
    // TODO:    unlikely b/c of good primal mesh traversal...
    // TODO:    funcs alr traverse dual graph...

    Vector3 rayArcInt(const Vector3& ns,
                      const Vector3& n,
                      const Vector3& n1,
                      const Vector3& n2);
};

#endif // GAUSSMAP_H
