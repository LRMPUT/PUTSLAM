#include <iostream>
#include <thread>
#include "include/Defs/putslam_defs.h"
#include "Grabber/kinect_grabber.h"
#include "PoseGraph/graph_g2o.h"
#include "PoseGraph/global_graph.h"
#include "Tracker/trackerKLT.h"
#include "3rdParty/tinyXML/tinyxml2.h"
#include <cmath>
#include <atomic>

using namespace std;

Graph * graph;

auto startT = std::chrono::high_resolution_clock::now();

// optimization thread
void optimize(){
    // graph optimization
    graph->optimize(0);
}

// optimization and pruning thread
void optimizeAndPrune(){
    // graph pruning and optimization
    graph->optimizeAndPrune2(10, 70);
}

class CLParser
{
public:

    CLParser(int argc_, char * argv_[],bool switches_on_=false);
    ~CLParser(){}

    string get_arg(int i);
    string get_arg(string s);

private:

    int argc;
    vector<string> argv;

    bool switches_on;
    map<string,string> switch_map;
};

CLParser::CLParser(int argc_, char * argv_[],bool switches_on_){
    argc=argc_;
    argv.resize(argc);
    copy(argv_,argv_+argc,argv.begin());
    switches_on=switches_on_;

    //map the switches to the actual
    //arguments if necessary
    if (switches_on)
    {
        vector<string>::iterator it1,it2;
        it1=argv.begin();
        it2=it1+1;

        while (true)
        {
            if (it1==argv.end()) break;
            if (it2==argv.end()) break;

            if ((*it1)[0]=='-')
                switch_map[*it1]=*(it2);

            it1++;
            it2++;
        }
    }
}

string CLParser::get_arg(int i){
    if (i>=0&&i<argc)
        return argv[i];

    return "";
}

string CLParser::get_arg(std::string s){
    if (!switches_on) return "";

    if (switch_map.find(s)!=switch_map.end())
        return switch_map[s];

    return "";
}


int main(int argc, char * argv[])
{
    try {
        using namespace putslam;
        using namespace std::chrono;

        CLParser cmd_line(argc,argv,true);
        if (cmd_line.get_arg("-h").size())
            std::cout << "To run type: ./Demo_pruning -i graphsACCV/office_traj0/MatchingORBBALC/graphFile.g2o -t graphsACCV/office_traj0/MatchingORBBALC/graphLoaded.g2o -o graphsACCV/office_traj0/MatchingORBBALC/graphPruned.g2o\n";

        tinyxml2::XMLDocument config;
        config.LoadFile("../../resources/configGlobal.xml");
        if (config.ErrorID())
            std::cout << "unable to load config file.\n";
        std::string configFile(config.FirstChildElement( "Grabber" )->FirstChildElement( "calibrationFile" )->GetText());
        KinectGrabber::UncertaintyModel sensorModel(configFile);
        graph = createPoseGraphG2O(sensorModel.config.pose);

        //load graph from file
        //((PoseGraphG2O *)graph)->loadG2O("graphMN/TrackingOnFeatures/graphFile.g2o");
        //((PoseGraphG2O *)graph)->loadG2O("graphMN/Matching/graphFile.g2o");
        //((PoseGraphG2O *)graph)->loadG2O("graphMN/TrackingWithBundleAdjustment/graphFile.g2o");
        //((PoseGraphG2O *)graph)->loadG2O("graphMN/MatchingwithBundleAdjustment/graphFile.g2o");
        //((PoseGraphG2O *)graph)->loadG2O("graphMN/room/TrackingWithBundleAdjustment/graphFile.g2o");
        //((PoseGraphG2O *)graph)->loadG2O("graphMN/room/MatchingWithBundleAdjustment/graphFile.g2o");
        std::cout << "load g2o\n";
        ((PoseGraphG2O *)graph)->loadG2O(cmd_line.get_arg("-i"));
std::cout << "end load g2o\n";
        //save init graph
        //graph->save2file("graphMN/TrackingOnFeatures/graphFile_test.g2o");
        //graph->save2file("graphMN/Matching/graphFile_test.g2o");
        //graph->save2file("graphMN/TrackingWithBundleAdjustment/graphFile_test.g2o");
        //graph->save2file("graphMN/MatchingwithBundleAdjustment/graphFile_test.g2o");
        //graph->save2file("graphMN/room/TrackingWithBundleAdjustment/graphFile_test.g2o");
        //graph->save2file("graphMN/room/MatchingWithBundleAdjustment/graphFile_test.g2o");
        graph->save2file(cmd_line.get_arg("-t"));

        //optimize
        //std::thread tOpt(optimize);
        std::thread tOpt(optimizeAndPrune);

        tOpt.join();

        ///checking export/import methods
        // save optimal graph to file
        // to view run ./g2o_viewer optimalGraph.g2o
        //graph->save2file("graphMN/TrackingOnFeatures/graphFilePruned.g2o");
        //graph->save2file("graphMN/Matching/graphFilePruned.g2o");
        //graph->save2file("graphMN/TrackingWithBundleAdjustment/graphFilePruned.g2o");
        //graph->save2file("graphMN/MatchingwithBundleAdjustment/graphFilePruned.g2o");
        //graph->save2file("graphMN/room/TrackingWithBundleAdjustment/graphFilePruned.g2o");
        //graph->save2file("graphMN/room/MatchingWithBundleAdjustment/graphFilePruned.g2o");
        graph->save2file(cmd_line.get_arg("-o"));

        //graph->plot2file("graphOpt.m");
        graph->plot2file("graphPruned.m");

        //clear the graph
        graph->clear();
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
