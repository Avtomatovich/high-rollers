#ifndef GAUSSMAP_H
#define GAUSSMAP_H

#include "mesh.h"

#include <vector>
#include <Eigen/Dense>

using namespace Eigen;

class GaussMap
{
public:
    GaussMap() {}

    // TODO: optimize by storing as MatrixX3d instead of std::vector?
    std::vector<Vector3d> traceGradient(const Vector3d& n0, const Mesh& mesh);

private:
    // TODO: is explicit Morse-Smale complex necessary?
    // TODO:    perhaps partially... store min, max, saddle per elem
    // TODO:    explicit connectivity necessary for ascending manifold?

    // TODO: pre-compute all max vertices, saddle edges, min faces
    // TODO:    does this belong in the mesh class instead?

    // TODO: define this func signature better
    void buildSeparatrix(const Mesh& mesh);

    // TODO: is explicit dual graph necessary?
    // TODO:    unlikely b/c of good primal mesh traversal...
    // TODO:    funcs alr traverse dual graph...

    Vector3d rayArcInt(const Vector3d& ns,
                       const Vector3d& n,
                       const Vector3d& n1,
                       const Vector3d& n2);
};

#endif // GAUSSMAP_H
