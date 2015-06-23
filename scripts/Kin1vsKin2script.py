#
#	author: Michal Nowicki
#
from subprocess import call
import subprocess
import sys
import glob
import os


for datasetId in range(2,10):
	for kinectNo in range(1,3):
		print("python evaluate_ate.py Kin1vsKin2/dataset" + str(datasetId) + "/Kin1_groundtruth_recomputed.txt Kin1vsKin2/dataset" + str(datasetId) + "/Kin" + str(kinectNo) +"_recomputed.txt --verbose --scale 1 --save_associations Kin1vsKin2/dataset" + str(datasetId) + "/Kin"+ str(kinectNo) + "_associations.res --plot Kin1vsKin2/dataset" + str(datasetId) + "/Kin" + str(kinectNo) +"Ate.png > Kin1vsKin2/dataset" + str(datasetId) + "/Kin" + str(kinectNo) +"Ate.res")
		call("python evaluate_ate.py Kin1vsKin2/dataset" + str(datasetId) + "/Kin1_groundtruth_recomputed.txt Kin1vsKin2/dataset" + str(datasetId) + "/Kin" + str(kinectNo) +"_recomputed.txt --verbose --scale 1 --save_associations Kin1vsKin2/dataset" + str(datasetId) + "/Kin"+ str(kinectNo) + "_associations.res --plot Kin1vsKin2/dataset" + str(datasetId) + "/Kin" + str(kinectNo) +"Ate.png > Kin1vsKin2/dataset" + str(datasetId) + "/Kin" + str(kinectNo) +"Ate.res", shell=True);


