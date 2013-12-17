/** @file parser.h
 *
 * XML Parser interface
 *
 */

#ifndef _PARSER_H_
#define _PARSER_H_

#include "../Defs/putslam_defs.h"
#include <iostream>
#include <string>
#include <vector>

namespace putslam {
    /// Parser interface
    class Parser {
        public:

            /// overloaded constructor
            Parser(const std::string _name, const std::string _filename) : name(_name), filename(_filename) {};

            /// Name of the parser
            virtual const std::string& getName() const = 0;

            /// Returns attribute
            virtual const std::string getAttribute(const std::string& group, const std::string& attribute) = 0;

            /// Virtual descrutor
            virtual ~Parser() {}

        protected:

            /// Parser name
            const std::string name;

            /// Filename
            const std::string filename;
    };
};

#endif // _GRABBER_H_
