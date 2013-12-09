/** @file putslam_defs.h
*
* Handest definitions
*
*/

#ifndef PUTSLAM_DEFS_H_INCLUDED
#define PUTSLAM_DEFS_H_INCLUDED

#include <cstdint>
#include <vector>
#include <memory>
#include <cmath>
#include "opencv2/core/core.hpp"

/// putslam name space
namespace putslam {

    /// putslam default floating point
    typedef double float_type;

	/// 3 Element vector class
    class Vec3 {
    public:
        /// vector components
        union {
            struct {
                float_type v1, v2, v3;
            };
            struct {
                float_type x, y, z;
            };
            float_type v[3];
        };

        /// Default constructor sets the default configuration
        inline Vec3() {
            v1 = 0; v2 = 0; v3 = 0;
        }
    };

	/// Matrix representation of SO(3) group of rotations
    class Mat33 {
        public:
        /// Matrix elements
        union {
            struct {
                float_type m11, m12, m13;
                float_type m21, m22, m23;
                float_type m31, m32, m33;
            };
            float_type m[3][3];
        };
        /// Default constructor
        inline Mat33() {
            m11 = 0; m12 = 0; m13 = 0;
            m21 = 0; m22 = 0; m23 = 0;
            m31 = 0; m32 = 0; m33 = 0;
        }
    };

	/// Homogeneous representation of SE(3) rigid body transformations
    class Mat34 {
        public:
            /// rotation matrix
            Mat33 R;
            /// translation
            Vec3 p;

            /// Default constructor sets the default configuration
            inline Mat34() {
            }
    };

	/// RGBA colour space
    class RGBA {
        public:
            /// Colour representation
            union {
                union {
                    struct {
                        std::uint8_t r, g, b, a;
                    };
                    float rgb;
                    std::uint8_t rgba_color[4];
                };
                uint32_t rgba;
            };
            /// Default constructor
            inline RGBA() {
                r = 255; g = 255; b = 255; a = 255;
            }
    };

	/// 3D point representation
    class Point3D {
        public:
            /// Point cloud
            typedef std::vector<Point3D> Cloud;

            /// Position
            Vec3 position;
            /// Colour
            RGBA colour;

            /// Default constructor
            inline Point3D() {
            }
    };

    /// image representation
    class Image {
        public:
            /// sequence of images
            typedef std::vector<Image> Seq;

            /// 2D image
            cv::Mat data;
            /// XYZRGBA point cloud
            Point3D::Cloud depthData;
            /// timestamp
            double timestamp;

            /// Default constructor
            inline Image() {
                timestamp = 0;
            }
    };
}

#endif // PUTSLAM_DEFS_H_INCLUDED
