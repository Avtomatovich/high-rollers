#include "gaussmap.h"

std::vector<Vector3d> GaussMap::traceGradient(const Vector3d& n0, const Mesh& mesh)
{
    // TODO: trace gradients over the Gauss map
    const auto& hull = mesh.getHull();
    std::vector<Vector3d> N;
    // TODO: check if supposed to insert at index of corresponding elem?
    N.push_back(n0);
    return N;
}

Vector3d GaussMap::rayArcInt(const Vector3d& ns,
                             const Vector3d& n,
                             const Vector3d& n1,
                             const Vector3d& n2)
{
    // TODO: understand how and why this works
    Vector3d nint = Vector3d::Zero();
    Vector3d n1n2 = n1.cross(n2), n2n1 = n2.cross(n1);
    Vector3d d = ns.cross(n).cross(n1n2);
    Vector3d dn2 = d.cross(n2), dn1 = d.cross(n1);
    if (dn2.dot(n1n2) >= 0.0 && dn1.dot(n2n1) >= 0.0) {
        nint =  d;
    }
    if ((-dn2).dot(n1n2) >= 0.0 && (-dn1).dot(n2n1) >= 0.0) {
        nint = -d;
    }
    return nint;
}
