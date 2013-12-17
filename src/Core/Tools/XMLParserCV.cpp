#include "../include/Core/Tools/XMLParserCV.h"
#include <memory>
#include <stdexcept>
#include <chrono>
#include <thread>

using namespace putslam;

/// A single instance of Kinect grabber
XMLParserCV::Ptr parser;

XMLParserCV::XMLParserCV(void) : Parser("OpenCV XML Parser", "configGlobal1.xml") {
    std::cout << "Default constructor. Did you forgot to specify xml file?" << std::endl;
    file_storage.reset(new FileStorage(filename, FileStorage::READ));
    if (!file_storage->isOpened()) {
        std::cerr << "Failed to open " << filename << std::endl;
    }
}

const std::string& XMLParserCV::getName() const {
    return name;
}

/// Returns attributes
const std::string XMLParserCV::getAttribute(const std::string& group, const std::string& attribute){
    file_node = (*file_storage)[group];
    return file_node[attribute];
}

putslam::Parser* putslam::createXMLParserCV(const std::string& filename) {
    parser.reset(new XMLParserCV("OpenCV XML Parser", filename));
    return parser.get();
}
