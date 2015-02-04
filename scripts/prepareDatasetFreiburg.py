from subprocess import call
import subprocess
import sys
import glob
import os

# How to run:
# 	python2 ../scripts/prepareDatasetFreiburg.py arg
# where arg is the name of the used set, e.x.:
#	python2 ../scripts/prepareDatasetFreiburg.py rgbd_dataset_freiburg1_desk
dataset=sys.argv[1];


# Check if there is need
if os.path.isfile(dataset + "/matched" ):
	print("\nDataset " + dataset + " is probably already converted!\nRemove file 'matched' from the dataset dir if something is wrong!!!\n");
# The conversion is needed
else:
	# Finding matches
	p1 = subprocess.Popen("python2 associate.py " + dataset+"/rgb.txt "+dataset+"/depth.txt", stdout=subprocess.PIPE, shell=True);
	p2 = subprocess.Popen("cut -d' ' -f 1,3 > " + dataset+"/matched", stdin=p1.stdout, stdout=subprocess.PIPE, shell=True);
	p2.communicate();

	# Reading matches
	p = subprocess.Popen("cat " + dataset + "/matched", stdout=subprocess.PIPE, shell=True);
	x, err = p.communicate();

	# Dataset image counter
	numberOfImages = len(x.split('\n')) - 1;

	# Copy images with new names
	i = 0;
	for z in x.split('\n'):
		nums = z.split(' ')
		if len(nums)>1:
			rgbName = nums[0];
			dName = nums[1];

			# Debug printing 
			print(str(i+1)+"/"+ str(numberOfImages)+"\t" + rgbName + "\t" + dName);
	
			# Copying images
			call("cp " + dataset+"/rgb/"+str(rgbName.rstrip())+".png " + dataset+"/rgb_%0.5d.png" % (i), shell=True);
			call("cp "+dataset+"/depth/"+str(dName.rstrip())+".png " + dataset+"/depth_%0.5d.png" % (i), shell=True);
	
			# Saving initial position from gt
			if i==0:
				p1 = subprocess.Popen("cat " + dataset+"/matched", stdout=subprocess.PIPE, shell=True);
				p2 = subprocess.Popen("head -n 1 > test", stdin=p1.stdout, stdout=subprocess.PIPE, shell=True);
				call("python2 associate.py "+ dataset+"/groundtruth.txt test > init", shell=True);
				p1 = subprocess.Popen("cat init", stdout=subprocess.PIPE, shell=True);
				p2 = subprocess.Popen("cut -d' ' -f 2-8 > " + dataset+"/initialPosition", stdin=p1.stdout, stdout=subprocess.PIPE, shell=True);
				call("rm test init", shell=True);
			i=i+1;

	call("rm -rf "+dataset+"/rgb "+dataset+"/depth "+dataset+"/rgb.txt "+dataset+"/depth.txt "+dataset+"/accelerometer.txt", shell=True);
	


