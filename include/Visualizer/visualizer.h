/** @file visualizer.h
 *
 * Visualizer interface
 *
 */

#ifndef _VISUALIZER_H_
#define _VISUALIZER_H_

#include "../Defs/putslam_defs.h"
#include <string>
#include <vector>

namespace putslam {
/// Visualizer interface
class Visualizer {
public:

    /// Visualizer type
    enum Type {
        /// QGLViewer-based
        VISUALIZER_QGL,
    };

    /// overloaded constructor
    Visualizer(const std::string _name, Type _type) :
            name(_name), type(_type) {
    };

    /// Name of the visualizer
    virtual const std::string& getName() const = 0;

    /// Virtual descrutor
    virtual ~Visualizer() {
    }

    /// visualize
    virtual void visualize(void) = 0;

protected:
    /// Visualizer type
    Type type;

    /// Visualizer name
    const std::string name;
};
};

#endif // _VISUALIZER_H_
