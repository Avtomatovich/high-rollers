#include <QCoreApplication>
#include <QCommandLineParser>
#include <QtCore>
#include <iostream>
#include <chrono>
#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/surface_mesh.h"

#include "geometrycentral/surface/surface_mesh_factories.h"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "libqhullcpp/Qhull.h"
#include "libqhullcpp/QhullFacetList.h"
#include "libqhullcpp/QhullVertexSet.h"

using namespace geometrycentral;
using namespace geometrycentral::surface;


int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    QCommandLineParser parser;
    parser.addHelpOption();
    parser.addPositionalArgument("config",  "Path of the config (.ini) file.");
    parser.process(a);
    auto t0 = std::chrono::high_resolution_clock::now();
    // Check for invalid argument count
    const QStringList args = parser.positionalArguments();
    if (args.size() < 1) {
        std::cerr << "Not enough arguments. Please provide a path to a config file (.ini) as a command-line argument." << std::endl;
        a.exit(1);
        return 1;
    }

    // Parse common inputs
    QSettings settings( args[0], QSettings::IniFormat );
    QString infile  = settings.value("IO/infile").toString();
    // std::printf("%s\n", infile.toStdString().c_str());

    std::unique_ptr<SurfaceMesh> mesh;
    std::unique_ptr<VertexPositionGeometry> geometry;
    std::tie(mesh, geometry) = readSurfaceMesh(infile.toStdString());

    for (Vertex v : mesh->vertices()) {
        std::cout << "Vertex " << v << " has degree " << v.degree() << "\n";
        for (Face fn : v.adjacentFaces()) {
            std::cout << "  incident on face " << fn << "\n";
        }
    }

    std::vector<double> ptArray;
    ptArray.reserve(geometry->mesh.nVertices() * 3);

    for (Vertex v : geometry->mesh.vertices()) {
        Vector3 pos = geometry->vertexPositions[v];
        ptArray.push_back(pos.x);
        ptArray.push_back(pos.y);
        ptArray.push_back(pos.z);
    }

    orgQhull::Qhull qhull;
    // Parameters: comment, dimension, numPoints, coords, qhullCommand
    qhull.runQhull("", 3, geometry->mesh.nVertices(), ptArray.data(), "Qt");

    // Collect the raw hull faces from the qhull output
    std::vector<std::vector<size_t>> rawFaces;
    for (orgQhull::QhullFacet facet : qhull.facetList()) {
        if (!facet.isUpperDelaunay()) {
            // Get Qhull's normal for this facet
            auto normalCoords = facet.hyperplane().coordinates();
            Vector3 qhullNormal = {normalCoords[0], normalCoords[1], normalCoords[2]};

            // Read vertices
            std::vector<size_t> faceIndices;
            std::vector<Vector3> facePositions;
            for (orgQhull::QhullVertex vertex : facet.vertices()) {
                size_t oldIdx = vertex.point().id();
                faceIndices.push_back(oldIdx);
                facePositions.push_back({
                    vertex.point().toStdVector()[0],
                    vertex.point().toStdVector()[1],
                    vertex.point().toStdVector()[2]
                });
            }

            // Compute winding normal: cross product of two face edges
            Vector3 e1 = facePositions[1] - facePositions[0];
            Vector3 e2 = facePositions[2] - facePositions[0];
            Vector3 windingNormal = cross(e1, e2);

            // If winding normal opposes Qhull normal, reverse vertex order
            if (dot(windingNormal, qhullNormal) < 0) {
                std::reverse(faceIndices.begin(), faceIndices.end());
            }

            rawFaces.push_back(faceIndices);
        }
    }


    // Collect only the vertex IDs actually used by hull faces - ignore the verts that aren't in the hull
    std::map<size_t, size_t> oldToNew;
    std::vector<Vector3> hullVerts;

    //for each vertex index, we create a mapping of the old to new index for the vertices list
    for (std::vector<size_t>& face : rawFaces) {
        for (size_t oldIdx : face) {
            if (oldToNew.find(oldIdx) == oldToNew.end()) {
                oldToNew[oldIdx] = hullVerts.size();  // give the next index to the map

                // look up position from original geometry
                Vector3 pos = geometry->vertexPositions[geometry->mesh.vertex(oldIdx)];
                hullVerts.push_back(pos);
            }
        }
    }

    // Remap face indices to the new range without gaps in the array
    std::vector<std::vector<size_t>> hullFaces;
    for (std::vector<size_t>& face : rawFaces) {
        std::vector<size_t> remapped;
        for (size_t oldIdx : face) { //for each face, push back the new index
            remapped.push_back(oldToNew[oldIdx]);
        }
        hullFaces.push_back(remapped);
    }

    std::tuple<std::unique_ptr<SurfaceMesh>, std::unique_ptr<VertexPositionGeometry>> convexMesh =
        makeManifoldSurfaceMeshAndGeometry(hullFaces, hullVerts);

    polyscope::init(); // initialize the gui

    polyscope::registerSurfaceMesh("My Convex Hull", std::get<1>(convexMesh)->vertexPositions, std::get<0>(convexMesh)->getFaceVertexList());

    polyscope::show(); // pass control to the gui until the user exits

    // Finish timing
    auto t1 = std::chrono::high_resolution_clock::now();
    auto duration = duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    std::cout << "Execution took " << duration << " milliseconds." << std::endl;



    a.exit();
}
