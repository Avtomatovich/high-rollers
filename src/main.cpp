#include <QCoreApplication>
#include <QCommandLineParser>
#include <QtCore>

#include <iostream>

#include "mesh.h"
#include "gaussmap.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    QCommandLineParser parser;
    parser.addHelpOption();
    parser.addPositionalArgument("mesh",  "Path of the mesh (.obj|.ply|.off|.stl) file.");
    parser.addPositionalArgument("x",  "X-coordinate of center of mass (optional).");
    parser.addPositionalArgument("y",  "Y-coordinate of center of mass (optional).");
    parser.addPositionalArgument("z",  "Z-coordinate of center of mass (optional).");
    parser.process(a);

    // check for invalid argument count
    const QStringList args = parser.positionalArguments();
    if (args.size() < 1) {
        std::cerr << "Not enough args. Please provide a mesh file path (.obj|.ply|.off|.stl)." << std::endl;
        a.exit(1);
        return 1;
    }
    
    // init bool for valid center of mass param
    bool ok = false;
    
    // parse center of mass coords
    Vector3 com;
    if (args.size() >= 4) {
        for (int i = 1; i < 4; i++) {
            args[i].toFloat(&ok);
            if (!ok) {
                char pos = i == 1 ? 'x' : i == 2 ? 'y' : 'z'; 
                std::cerr << "Invalid " << pos << "-coord param." << std::endl;
                break;
            } 
            com[i] = args[i].toFloat();
        }
    }
    
    // create mesh instance
    Mesh mesh(args[0].toStdString(), com, !ok);

    // create Gauss map instance
    GaussMap gaussMap(mesh);

    // std::vector<Vector3> path = gaussMap.traceGradient(gaussMap.randomGaussNormal());

    // for (const Vector3& n : path) {
    //     SurfacePoint elem = gaussMap.elementWithNormal(n);
    //     std::cout << "Surface point: " << elem << std::endl;
    //     std::cout << "Surface point normal: " << n << std::endl;
    // }

    // auto& hull = mesh.getHull();
    // const auto& geom = mesh.getHullGeom();

    // // size_t randIdx = randomIndex(hull.nEdges());
    // size_t randIdx = 0;
    // double t = unitRand();
    // // double t = 0.5;

    // std::cout << "Random index: " << randIdx << std::endl << std::endl;

    // size_t i = 0;
    // for (Edge e : hull.edges()) {
    //     std::cout << "Current index: " << i << std::endl;
    //     Face f1 = e.halfedge().face();
    //     Face f2 = e.halfedge().twin().face();
    //     Vector3 nf1 = geom.faceNormal(f1);
    //     Vector3 nf2 = geom.faceNormal(f2);
    //     // std::cout << "Face normal 1 of edge " << e << ": " << nf1 << std::endl;
    //     // std::cout << "Face normal 2 of edge " << e << ": " << nf2 << std::endl;
    //     if (i >= randIdx) {
    //         double omega = geom.edgeDihedralAngle(e);
    //         // double omega = angle(nf1, nf2);
    //         if (omega <= 0.0) {
    //             std::cout << "Invalid dihedral angle: " << omega << std::endl;
    //             omega = std::acos(std::fmax(-1., std::fmin(1., dot(unit(nf1), unit(nf2)))));
    //             std::cout << "For comparison,    handmade: " << omega << std::endl;
    //             omega = angle(nf1, nf2);
    //             std::cout << "geometry-central angle func: " << omega << std::endl;
    //             omega = geom.edgeDihedralAngles[e];
    //             std::cout << "pre-computed dihedral angle: " << omega << std::endl << std::endl;
    //             i++;
    //             continue;
    //         }
    //         Vector3 n = 1.0 / std::sin(omega) *
    //                 (std::sin((1.0 - t) * omega) * nf1 +
    //                  std::sin(t * omega) * nf2);
    //         SurfacePoint elem = gaussMap.elementWithNormal(n);
    //         std::cout << "Current edge: " << e << std::endl;
    //         std::cout << "Surface point: " << elem << std::endl;
    //         std::cout << "Surface point normal: " << n << std::endl;
    //         break;
    //     }
    //     i++;
    // }

    // Vector3 n = gaussMap.randomGaussNormal();
    // Vector3 n{0, -1, 0.0005};
    // SurfacePoint elem = gaussMap.elementWithNormal(n);
    // std::cout << "Surface point: " << elem << std::endl;
    // std::cout << "Surface point normal: " << n << std::endl;

    // output stable face probabilties
    // gaussMap.computeProb();

    // display mesh and hull
    // mesh.show();

    a.exit();
}
