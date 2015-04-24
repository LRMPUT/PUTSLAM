from subprocess import call
import subprocess
import sys
import glob
import os

# How to run:
# 	python2 ../scripts/prepareDatasetFreiburg.py arg
# where arg is the name of the used set, e.x.:
#	python2 ../scripts/prepareDatasetFreiburg.py rgbd_dataset_freiburg1_desk
# Or provide a full path"
#	#	python2 ../scripts/prepareDatasetFreiburg.py path/rgbd_dataset_freiburg1_desk
dataset=sys.argv[1];


# Check if there is need
if os.path.isfile(dataset + "/matched" ):
	print("\nDataset " + dataset + " is probably already converted!\nRemove file 'matched' from the dataset dir if something is wrong!!!\n");
# The conversion is needed
else:
	# Finding matches
	p1 = subprocess.Popen("ls -la " + dataset + "/scene_*.png", stdout=subprocess.PIPE, shell=True);
	p10 = subprocess.Popen("tr -s ' '" , stdin=p1.stdout, stdout=subprocess.PIPE, shell=True);
	p2 = subprocess.Popen("cut -d ' ' -f 9" , stdin=p10.stdout, stdout=subprocess.PIPE, shell=True);
	p3 = subprocess.Popen("cut -d '_' -f 6" , stdin=p2.stdout, stdout=subprocess.PIPE, shell=True);
	p4 = subprocess.Popen("cut -d '.' -f 1 > "+ dataset+"/matched" , stdin=p3.stdout, stdout=subprocess.PIPE, shell=True);
	p4.communicate();

	#exit();
	
	# Reading matches
	p = subprocess.Popen("cat " + dataset + "/matched", stdout=subprocess.PIPE, shell=True);
	x, err = p.communicate();

	# Dataset image counter
	numberOfImages = len(x.split('\n')) - 1;

	# Copy images with new names
	i = 0;
	for number in x.split('\n'):
		# Debug printing 
		print(str(i+1)+"/"+ str(numberOfImages)+"\t|" + number + "|");
	
		# Copying images
		print("cp " + dataset+"/scene_00_"+str(number.rstrip())+".png " + dataset+"/rgb_%0.5d.png" % (i));		
		call("cp " + dataset+"/scene_00_"+str(number.rstrip())+".png " + dataset+"/rgb_%0.5d.png" % (i), shell=True);
		call("python "+ dataset + "/convertICLdepthToDepthImage.py " + dataset+"/scene_00_"+str(number.rstrip())+".depth " + dataset+"/depth_%0.5d.png" % (i), shell=True);
	
		# Saving initial position from gt
		if i==0:
			call("cp " + dataset + "/*freiburg* " + dataset + "/groundtruth.txt", shell=True);

			call("echo '0 0 -2.5 0 0 0 1' > " + dataset+"/initialPosition", shell=True);
		i=i+1;

	#call("rm -rf "+dataset+"/rgb "+dataset+"/depth "+dataset+"/rgb.txt "+dataset+"/depth.txt "+dataset+"/accelerometer.txt", shell=True);
	

