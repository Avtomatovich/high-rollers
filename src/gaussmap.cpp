#include "gaussmap.h"


GaussMap::GaussMap(const Mesh& mesh) :
    _hull(mesh.getHull()),
    _geom(mesh.getHullGeom()),
    _edgeTypes(mesh.getEdgeTypes()),
    _faceTypes(mesh.getFaceTypes())
{}

std::vector<Vector3> GaussMap::traceGradient(const Vector3& n0)
{
    // TODO: trace gradients over the Gauss map
    std::vector<Vector3> N;
    // TODO: check if supposed to insert at index of corresponding elem?
    N.push_back(n0);
    return N;
}

Vector3 GaussMap::rayArcInt(const Vector3& ns,
                            const Vector3& n,
                            const Vector3& n1,
                            const Vector3& n2)
{
    // TODO: understand how and why this works
    Vector3 n1n2 = cross(n1, n2), n2n1 = cross(n2, n1);
    Vector3 d = cross(cross(ns, n), n1n2);
    Vector3 dn2 = cross(d, n2), dn1 = cross(d, n1);
    if (dot(dn2, n1n2) >= 0.0 && dot(dn1, n2n1) >= 0.0) {
        return d;
    }
    if (dot(-dn2, n1n2) >= 0.0 && dot(-dn1, n2n1) >= 0.0) {
        return -d;
    }
    return Vector3::zero();
}
