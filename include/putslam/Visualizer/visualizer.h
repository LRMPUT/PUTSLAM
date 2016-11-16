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
    enum VisualizerType {
        /// QGLViewer-based
        VISUALIZER_QGL,
    };

    /// overloaded constructor
    Visualizer(const std::string _name, VisualizerType _type) :
            visualizerName(_name), visualizerType(_type) {
    };

    /// Name of the visualizer
    virtual const std::string& getName() const {return visualizerName;};

    /// Virtual descrutor
    virtual ~Visualizer() {
        std::cout << "destructor Visualizer\n";
    }

protected:
    /// Visualizer type
    VisualizerType visualizerType;

    /// Visualizer name
    const std::string visualizerName;
};
};

#endif // _VISUALIZER_H_
