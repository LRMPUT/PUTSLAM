#
#	author: Michal Nowicki
#
from subprocess import call
import subprocess
import sys
import glob
import os

# Path to save main results - create or clear
if not os.path.exists("../../results"):
	os.makedirs("../../results");
else:
	call('rm -rf ../../results/*', shell=True);	

# For all provided config files
for root, dirs, files in os.walk("../../configs/"):
	for dir in dirs:
		# Copy to currently used settings
		call('cp ' + root + dir+"/* ../../resources/", shell=True);	
		
		# Run software
		call('./demoMatching', shell=True);

		# Call ransac and map statistics 
		call('python statistics.py', shell=True);
		call('octave mapData.m > mapResults.res', shell=True);

		# Create or clear a place to store those results
		if not os.path.exists("../../results/"+dir):
			os.makedirs("../../results/"+dir);
		else:
			call('rm -rf ../../results/' + dir + '/*', shell=True);

		# Copy settings
		call('cp ../../resources/* ../../results/' + dir + '/', shell=True);

		# Copy those results
		call('mv *.png ../../results/' + dir + '/', shell=True);
		call('mv *.jpg ../../results/' + dir + '/', shell=True);
		call('mv *.log ../../results/' + dir + '/', shell=True);
		call('mv *.map ../../results/' + dir + '/', shell=True);
		call('mv *.res ../../results/' + dir + '/', shell=True);
		call('mv *.py ../../results/' + dir + '/', shell=True);
		call('mv *.g2o ../../results/' + dir + '/', shell=True);
		call('mv *.m ../../results/' + dir + '/', shell=True);
