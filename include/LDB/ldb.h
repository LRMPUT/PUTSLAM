/*
	libLDB-LDB.h
	
	Created on: May 27, 2014
	Author: xinyang, chonghuang

    LDB - Local Difference Binary 
    Reference implementation of
    [1] Yang, X., and K.-T. Cheng. Local Difference Binary for 
	Ultra-fast and Distinctive Feature Description. IEEE Trans. 
	on PAMI, vol. 36, no. 1, 2014.
    [2] Yang, X., and K.-T. Cheng. Learning Optimized Local 
	Difference Binariesfor Scalable Augmented Reality on Mobile 
	Devices. IEEE Trans. on VCG, 2014.
    [3] Yang, X., and K.-T. Cheng. LDB: An ultra-fast feature 
	for scalable Augmented Reality on mobile devices. 
	In Proc. of ISMAR, 2012.

    Copyright (C) 2012  The Learning-Based Multimedia, University of California, 
	Santa Barbara, Xin Yang, Kwang-Ting(Tim) Cheng.

    libLDB is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    libLDB is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with libLDB.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef LDB_H_
#define LDB_H_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

using namespace cv;
using namespace std;

class LDB
{
public:

	int kBytes;
	
	LDB(int _patchSize = 48);
	~LDB();
    // returns the descriptor size in bytes
        int descriptorSize() const;
    // Compute the LDB features and descriptors on an image
	void compute( const Mat& image, vector<KeyPoint>& keypoints, Mat& descriptors, bool flag) const;

protected:
	
	int nfeatures;
	double scaleFactor;
	int nlevels;
	int firstLevel;
	int patchSize;

};

typedef LDB LdbDescriptorExtractor;

#endif
