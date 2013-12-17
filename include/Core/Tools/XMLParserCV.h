/** @file XMLparserCV.h
 *
 * implementation - openCV-based XML parser
 *
 */

#ifndef XMLPARSERCV_H_INCLUDED
#define XMLPARSERCV_H_INCLUDED

#include "XMLParser.h"
#include <iostream>
#include <memory>
#include <opencv2/core/core.hpp>
#include <string>

using namespace cv;

namespace putslam {
    /// create a single XML parser (using openCV)
    Parser* createXMLParserCV(const std::string& filename);
};

using namespace putslam;

/// Parser implementation
class XMLParserCV : public Parser {
    public:
        /// Pointer
        typedef std::unique_ptr<XMLParserCV> Ptr;

        /// Construction
        XMLParserCV(void);

        ///Overloaded constructor
        XMLParserCV(const std::string _name, const std::string _filename) : Parser(_name, _filename) {
            file_storage.reset(new FileStorage(filename, FileStorage::READ));
            if (!file_storage->isOpened()) {
                std::cerr << "Failed to open " << filename << std::endl;
            }
        };

        /// Name of the parser
        virtual const std::string& getName() const;

        /// Returns attribute
        const std::string getAttribute(const std::string& group, const std::string& attribute);

    private:
        /// openCV file storage
        std::unique_ptr<FileStorage> file_storage;

        /// File node
        FileNode file_node;
};

#endif // XMLPARSERCV_H_INCLUDED
