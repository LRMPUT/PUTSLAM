#!/usr/bin/python
import numpy as np
import cv2
import sys
from math import sqrt

print 'Number of arguments:', len(sys.argv), 'arguments.'
print 'Argument List:', str(sys.argv)

f = open(sys.argv[1], 'r');

scale = 50.0;
size = 480, 640, 1
img = np.zeros(size, dtype=np.uint16)

i = 0;
j = 0;
for line in f:
	lineValues = line.split();
	for val in lineValues:
		img[int(i / 640), int(i%640)] = int( float(val) * scale * 480.6 / sqrt(480.6 * 480.6 + (i / 640 - 239.5) * (i / 640 - 239.5) + (float(i % 640) - 319.5)* (float(i % 640) - 319.5)) );
		i=i+1;
cv2.imwrite(sys.argv[2], img);

#cv2.namedWindow("draw", cv2.CV_WINDOW_AUTOSIZE)
#while True:
#    cv2.imshow("draw", img)
#    ch = 0xFF & cv2.waitKey(1)
#    if ch == 27:
#        break
#cv2.destroyAllWindows()

