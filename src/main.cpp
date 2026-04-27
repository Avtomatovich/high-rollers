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

    // output stable face probabilties
    gaussMap.computeProb();

    // display mesh and hull
    // mesh.show();

    a.exit();
}
