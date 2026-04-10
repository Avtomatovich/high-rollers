#include <QCoreApplication>
#include <QCommandLineParser>
#include <QtCore>
#include <iostream>
#include <chrono>
#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/surface_mesh.h"

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

    // 2. Run Qhull
    orgQhull::Qhull qhull;
    // Parameters: comment, dimension, numPoints, coords, qhullCommand
    qhull.runQhull("", 3, geometry->mesh.nVertices(), ptArray.data(), "Qt");

    std::vector<std::vector<size_t>> hullFaces;

    for (auto facet : qhull.facetList()) {
        if (!facet.isUpperDelaunay()) { // Standard check for 3D hulls
            std::vector<size_t> faceIndices;
            for (auto vertex : facet.vertices()) {
                // Get the original index of this point
                faceIndices.push_back(vertex.point().id());
            }
            hullFaces.push_back(faceIndices);
        }
    }

    polyscope::init(); // initialize the gui

    // add the mesh to the gui
    // polyscope::registerSurfaceMesh("my mesh",
    //                                geometry->vertexPositions, mesh->getFaceVertexList());

    polyscope::registerSurfaceMesh("My Convex Hull",
                                   geometry->vertexPositions,
                                   hullFaces);

    polyscope::show(); // pass control to the gui until the user exits

    // Finish timing
    auto t1 = std::chrono::high_resolution_clock::now();
    auto duration = duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    std::cout << "Execution took " << duration << " milliseconds." << std::endl;



    a.exit();
}
