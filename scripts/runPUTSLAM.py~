#
#	author: Michal Nowicki
#
from subprocess import call
import subprocess
import sys
import glob
import os

# Path to save main results - create if needed
if not os.path.exists("../../results"):
	os.makedirs("../../results");	

# For all provided config files
dirs = [d for d in os.listdir('../../configs/') ]
root = '../../configs/';
for dir in dirs:
	print("Current version: " + dir)
	if "backup" in dir:
		print("Skipping a backup of configs!");
		continue;

	# Create or clear a place to store those results
	if not os.path.exists("../../results/"+dir):
		os.makedirs("../../results/"+dir);
	else:
		print("Results exists - skipping!");
		continue;


	# Copy to currently used settings
	call('cp ' + root + dir+"/* ../../resources/", shell=True);	
		
	# Run software
	call('./demoMatching', shell=True);

	# Evaluate results
	datasetName = subprocess.check_output("cat DatasetName" , shell = True);

	call("python2 ../../scripts/evaluate_ate.py " + datasetName
					+ "groundtruth.txt VO_trajectory.res --verbose --scale 1 --save_associations ate_association.res --plot VOAte.png > VOAte.res", shell=True);
	call("python2 ../../scripts/evaluate_ate.py " + datasetName
					+ "groundtruth.txt graph_trajectory.res --verbose --scale 1 --save_associations ate_association.res --plot g2oAte.png > g2oAte.res", shell=True);
	call("python2 ../../scripts/evaluate_ate.py " + datasetName
					+ "groundtruth.txt MotionModel_trajectory.res --verbose --scale 1 --save_associations ate_association.res --plot mmAte.png > mmAte.res", shell=True);
	call("python2 ../../scripts/evaluate_rpe.py " + datasetName
			+ "groundtruth.txt VO_trajectory.res --verbose --delta_unit 'f' --fixed_delta --plot VORpe.png > VORpe.res", shell=True);
	call("python2 ../../scripts/evaluate_rpe.py " + datasetName
				+ "groundtruth.txt graph_trajectory.res --verbose --delta_unit 'f' --fixed_delta --plot g2oRpe.png > g2oRpe.res", shell=True);
	call("python2 ../../scripts/evaluate_rpe.py " + datasetName
				+ "groundtruth.txt MotionModel_trajectory.res --verbose --delta_unit 'f' --fixed_delta --plot mmRpe.png > mmRpe.res", shell=True);
	call('rm DatasetName', shell=True);	

	# Call ransac and map statistics 
	call('python statistics.py', shell=True);
	call('octave mapData.m > mapResults.res', shell=True);		

	# Copy settings (create directory if needed)
	if not os.path.exists("../../results/"+dir+"/resources"):
		os.makedirs("../../results/"+dir+"/resources");
	else:
		call('rm -rf ../../results/' + dir + '/resources/*', shell=True);
	call('cp ' + root + dir+"/* ../../results/" + dir + '/resources/', shell=True);

	# Copy those results
	call('mv *.png ../../results/' + dir + '/', shell=True);
	call('mv *.jpg ../../results/' + dir + '/', shell=True);
	call('mv *.log ../../results/' + dir + '/', shell=True);
	call('mv *.map ../../results/' + dir + '/', shell=True);
	call('mv *.res ../../results/' + dir + '/', shell=True);
	call('mv *.py ../../results/' + dir + '/', shell=True);
	call('mv *.g2o ../../results/' + dir + '/', shell=True);
	call('mv *.m ../../results/' + dir + '/', shell=True);

	# OCTOMAP
	if len(sys.argv) >= 2 and "OCTOMAP" in sys.argv[1].upper():
		print("Running octomap conversion!");
		call('../../../octomap/bin/log2graph ../../results/' + dir + '/octomap.log  ../../results/' + dir + '/octomap.graph', shell=True);
		call('../../../octomap/bin/graph2tree -i ../../results/' + dir + '/octomap.graph -o ../../results/' + dir + '/octomap.bt -res 0.01', shell=True);
		call('../../../octomap/bin/octovis ../../results/' + dir + '/octomap.bt', shell=True);



call("python ../../scripts/summarizeResults.py",shell=True);
