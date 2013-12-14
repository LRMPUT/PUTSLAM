/** @file map.h
 *
 * Environment Map interface
 *
 */

#ifndef _MAP_H_
#define _MAP_H_

#include "../Defs/putslam_defs.h"
#include <string>
#include <vector>

namespace putslam {
    /// Map interface
    class Map {
        public:

            /// Map type
            enum Type {
                    /// Elecation Map
                    MAP_ELEVATION,
                    /// Voxel Grid
                    MAP_VOXEL_GRID
            };

            /// overloaded constructor
            Map(const std::string _name, Type _type) : name(_name), type(_type) {};

            /// Name of the map
            virtual const std::string& getName() const = 0;

            /// Virtual descrutor
            virtual ~Map() {}

        protected:
            /// Map type
            Type type;

            /// Map name
            const std::string name;
    };
};

#endif // _MAP_H_
