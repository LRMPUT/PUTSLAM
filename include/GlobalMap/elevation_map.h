/** @file elevation_map.h
 *
 * implementation - Elevation Map
 *
 */

#ifndef ELEVATION_MAP_H_INCLUDED
#define ELEVATION_MAP_H_INCLUDED

#include "map.h"
#include <iostream>
#include <memory>

namespace putslam {
    /// create a single Elevation Map
    Map* createElevationMap(void);
};

using namespace putslam;

/// Map implementation
class ElevationMap : public Map {
    public:
        /// Pointer
        typedef std::unique_ptr<ElevationMap> Ptr;

        /// Construction
        ElevationMap(void);

        /// Name of the map
        const std::string& getName() const;

    private:

};

#endif // ELEVATION_MAP_H_INCLUDED
