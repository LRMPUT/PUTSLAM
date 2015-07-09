#
#	author: Michal Nowicki
#
from subprocess import call
import subprocess
import sys
import glob
import os
import fileinput

# Path to save main results - create if needed
if not os.path.exists("generatedConfigs"):
	os.makedirs("generatedConfigs");	

allFreiburg = ["freiburg1_360", "freiburg1_desk", "freiburg1_desk2",
	    	"freiburg1_room", "freiburg2_desk", "freiburg3_long_office_household"];

allICL = ["icl_office_room_traj1_loop", "icl_office_room_traj2_loop",
	 	"icl_office_room_traj2_loop_noise", "icl_office_room_traj3_loop"];

allKin1vs2 = ["kin1vs2_dataset2_kinect1", "kin1vs2_dataset2_kinect2", "kin1vs2_dataset3_kinect1",
		"kin1vs2_dataset3_kinect2", "kin1vs2_dataset4_kinect1", "kin1vs2_dataset4_kinect2",
		"kin1vs2_dataset5_kinect1", "kin1vs2_dataset5_kinect2"];

chosenDatasets = allFreiburg + allICL;

for datasetName in chosenDatasets:

	# Create or clear a place to store those results
	if not os.path.exists("generatedConfigs/"+datasetName):
		os.makedirs("generatedConfigs/"+datasetName);

	# Copy to currently used settings
	call('cp -r ../resources/* generatedConfigs/'+datasetName+'/', shell=True);	

	# Replace dataset with correct name
	filename = 'generatedConfigs/'+datasetName+'/fileModel.xml';
	for line in fileinput.input(filename, inplace=1):
		if "datasetFile=" in line:
			line = '<Model verbose="0" datasetFile=' + datasetName + '.xml/>';
		sys.stdout.write(line)





