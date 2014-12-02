#include <iostream>
#include <thread>
#include "include/Defs/putslam_defs.h"
#include "PoseGraph/graph_g2o.h"
#include "Utilities/CLParser.h"
#include "3rdParty/tinyXML/tinyxml2.h"
#include <cmath>

using namespace std;

int main(int argc, char * argv[])
{
    try {
        using namespace putslam;
        using namespace std::chrono;

        CLParser cmd_line(argc,argv,true);
        if (cmd_line.get_arg("-h").size())
            std::cout << "To run type: ./graph2D2graph3D -i input2Dgraph.g2o -o output3Dgraph.g2o\n";

        Graph * graph;
        cmd_line.get_arg("-i");
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
