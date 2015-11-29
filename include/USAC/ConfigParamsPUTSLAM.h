#ifndef CONFIGPARAMSPUTSLAM_H
#define CONFIGPARAMSPUTSLAM_H

#include "../include/USAC/ConfigParams.h"

namespace USACConfig
{
	// problem specific/data-related parameters: fundamental matrix
	struct PUTSLAM
	{
		PUTSLAM() : inputFilePath("")			// leave blank if not using config file
		{}

		std::string			inputFilePath;
	};
}

class ConfigParamsPUTSLAM : public ConfigParams
{
public:
	USACConfig::PUTSLAM putslam;
};

#endif
