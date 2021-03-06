/*
 * This file is part of ElasticFusion.
 *
 * Copyright (C) 2015 Imperial College London
 * 
 * The use of the code within this file and all code within files that 
 * make up the software that is ElasticFusion is permitted for 
 * non-commercial purposes only.  The full terms and conditions that 
 * apply to the code within this file are detailed within the LICENSE.txt 
 * file and at <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/elastic-fusion/elastic-fusion-license/> 
 * unless explicitly stated.  By downloading this file you agree to 
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then 
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */

#include "LiveLogReader.h"

#include "OpenNI2Interface.h"
#include "RealSenseInterface.h"

#include <chrono>


LiveLogReader::LiveLogReader(std::string file, bool flipColors, CameraType type, std::string outFile)
: LogReader(file, flipColors),
  lastFrameTime(-1),
  lastGot(-1)
{
	std::cout << "Creating live capture... "; std::cout.flush();

	std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
	creation_time = std::chrono::system_clock::to_time_t(now);


	if(type == CameraType::OpenNI2)
		cam = new OpenNI2Interface(creation_time,Resolution::getInstance().width(),Resolution::getInstance().height());
	else if(type == CameraType::RealSense)
		cam = new RealSenseInterface(Resolution::getInstance().width(), Resolution::getInstance().height(), 30,
				outFile.empty()? 0: outFile.c_str(), file.empty()? 0: file.c_str(), false);
	else if(type == CameraType::RealSenseSlam)
		cam = new RealSenseInterface(Resolution::getInstance().width(), Resolution::getInstance().height(), 30,
				outFile.empty()? 0: outFile.c_str(), file.empty()? 0: file.c_str(), true);
	else
		cam = nullptr;

	decompressionBufferDepth = new Bytef[Resolution::getInstance().numPixels() * 2];

	decompressionBufferImage = new Bytef[Resolution::getInstance().numPixels() * 3];

	if(!cam || !cam->ok())
	{
		std::cout << "failed!" << std::endl;
		std::cout << cam->error();
	}
	else
	{
		std::cout << "success!" << std::endl;

		std::cout << "Waiting for first frame"; std::cout.flush();

		int lastDepth = cam->latestDepthIndex.getValue();

		do
		{
			std::this_thread::sleep_for(std::chrono::microseconds(33333));
			std::cout << "."; std::cout.flush();
			lastDepth = cam->latestDepthIndex.getValue();
		} while(lastDepth == -1);

		std::cout << " got it!" << std::endl;
	}
}

LiveLogReader::~LiveLogReader()
{
	delete [] decompressionBufferDepth;

	delete [] decompressionBufferImage;

	delete cam;
}

void LiveLogReader::getNext()
{
	int lastDepth = cam->latestDepthIndex.getValue();

	assert(lastDepth != -1);

	int bufferIndex = lastDepth % CameraInterface::numBuffers;

	if(bufferIndex == lastGot)
	{
		return;
	}

	if(lastFrameTime == cam->frameBuffers[bufferIndex].second)
	{
		return;
	}

	memcpy(&decompressionBufferDepth[0], cam->frameBuffers[bufferIndex].first.first, Resolution::getInstance().numPixels() * 2);
	memcpy(&decompressionBufferImage[0], cam->frameBuffers[bufferIndex].first.second,Resolution::getInstance().numPixels() * 3);

//	std::cout << "LiveLogReader: ";
//	for(int i = 0; i < 16; i++)
//	{
//		std::cout << cam->poses[bufferIndex].data[i] << " ";
//	}
//	std::cout << std::endl;
	memcpy(cam->poses[bufferIndex].data, pose.data(), 16 * sizeof(float));

	pose.transposeInPlace(); // Eigen defaults to column-major storage


	lastFrameTime = cam->frameBuffers[bufferIndex].second;

	timestamp = lastFrameTime;

	rgb = (unsigned char *)&decompressionBufferImage[0];
	depth = (unsigned short *)&decompressionBufferDepth[0];

	imageReadBuffer = 0;
	depthReadBuffer = 0;

	imageSize = Resolution::getInstance().numPixels() * 3;
	depthSize = Resolution::getInstance().numPixels() * 2;

	if(flipColors)
	{
		for(int i = 0; i < Resolution::getInstance().numPixels() * 3; i += 3)
		{
			std::swap(rgb[i + 0], rgb[i + 2]);
		}
	}
}

const std::string LiveLogReader::getFile()
{
	std::stringstream ss;
	ss << Parse::get().baseDir() << "_";

	char mbstr[100];
	std::strftime(mbstr, sizeof(mbstr), "%Y_%m_%d_%H_%M_%S", std::localtime(&creation_time));

	ss << mbstr;

	return ss.str();
}

int LiveLogReader::getNumFrames()
{
	return std::numeric_limits<int>::max();
}

bool LiveLogReader::hasMore()
{
	return true;
}

void LiveLogReader::setAuto(bool value)
{
	cam->setAutoExposure(value);
	cam->setAutoWhiteBalance(value);
}
