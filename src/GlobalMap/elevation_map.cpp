#include "../include/GlobalMap/elevation_map.h"
#include <memory>
#include <stdexcept>

using namespace putslam;

/// A single instance of Elevation Map
ElevationMap::Ptr map;

ElevationMap::ElevationMap(void) : Map("Elevation Map", MAP_ELEVATION) {

}

const std::string& ElevationMap::getName() const {
    return name;
}

putslam::Map* putslam::createElevationMap(void) {
    map.reset(new ElevationMap());
    return map.get();
}
