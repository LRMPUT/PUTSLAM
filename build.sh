YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# g2o compilation
if [ ! -d 3rdParty/g2o/build ]; then
	printf "${YELLOW}Building g2o ...${NC}\n"
	cd 3rdParty
	cd g2o
	mkdir build 
	cd build
	cmake .. -DCMAKE_BUILD_TYPE=Release
	make -j$(nproc)
	sudo make install
	cd ../../..
	printf "${YELLOW}g2o build!${NC}\n"
else
	printf "${YELLOW}g2o already built ...${NC}\n"
fi;

# Visual place recognition
if [ ! -e resources/VisualPlaceRecognition/settings.yml ]; then
	printf "${YELLOW}Downloading VisualPlaceRecognition.tar.gz ...${NC}\n"
	cd resources/VisualPlaceRecognition/
	wget http://lrm.put.poznan.pl/files/OPUS/VisualPlaceRecognition.tar.gz
	printf "${YELLOW}Unpacking VisualPlaceRecognition.tar.gz ...${NC}\n"
	tar -xf VisualPlaceRecognition.tar.gz
	cd ../../
	printf "${YELLOW}VisualPlaceRecognition.tar.gz unpacked!${NC}\n"
else
	printf "${YELLOW}VisualPlaceRecognition.tar.gz already unpacked ...${NC}\n"
fi;

# PUTSLAM
printf "${YELLOW}Building PUTSLAM ...${NC}\n"
cd build
cmake ..
make -j$(nproc)
cd ..
printf "${YELLOW}PUTSLAM built finished!${NC}\n"

