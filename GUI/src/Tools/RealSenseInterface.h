#pragma once

#include <string>
#include <iostream>
#include <algorithm>
#include <map>

#ifdef WITH_REALSENSE
#include "librealsense/rs.hpp"
#endif

#ifdef WITH_REALSENSE_SDK
#include <rs_sdk.h>
#endif

#ifdef WITH_REALSENSE_SLAM
#include <librealsense/slam/slam.h>
#include <librealsense/slam/slam_utils.h>
#include <iomanip>
#endif

#include "ThreadMutexObject.h"
#include "CameraInterface.h"

class RealSenseInterface : public CameraInterface
{
public:
	RealSenseInterface(int width = 640, int height = 480, int fps = 30, const char* outFile = 0, const char* inFile = 0, bool tryUseSlam = false);
	virtual ~RealSenseInterface();

	const int width,height,fps;

	bool getAutoExposure();
	bool getAutoWhiteBalance();
	virtual void setAutoExposure(bool value);
	virtual void setAutoWhiteBalance(bool value);

	virtual bool ok()
	{
		return initSuccessful;
	}

	virtual std::string error()
	{
		return errorText;
	}

#ifdef WITH_REALSENSE
	struct RGBCallback
	{
	public:
		RGBCallback(int64_t & lastRgbTime,
				ThreadMutexObject<int> & latestRgbIndex,
				std::pair<uint8_t *,int64_t> * rgbBuffers,
				rs::device * device)
	: lastRgbTime(lastRgbTime),
	  latestRgbIndex(latestRgbIndex),
	  rgbBuffers(rgbBuffers),
	  dev(device)
	{
	}

		void operator()(rs::frame frame)
		{
//			auto aligned_data = reinterpret_cast<const uint8_t*>(dev->get_frame_data(rs::stream::color_aligned_to_depth));
//
//			std::cout << dev->get_stream_width(rs::stream::color_aligned_to_depth) << " "
//					 << dev->get_stream_height(rs::stream::color_aligned_to_depth) << " "
//					 << dev->get_stream_format(rs::stream::color_aligned_to_depth) << std::endl;
//
//			std::cout << frame.get_width() << " " << frame.get_height() << " " << frame.get_format() << std::endl << std::endl;

			lastRgbTime = std::chrono::duration_cast<std::chrono::milliseconds>(
					std::chrono::system_clock::now().time_since_epoch()).count();

			int bufferIndex = (latestRgbIndex.getValue() + 1) % numBuffers;

			memcpy(rgbBuffers[bufferIndex].first, frame.get_data(), frame.get_width() * frame.get_height() * 3);

			rgbBuffers[bufferIndex].second = lastRgbTime;

			latestRgbIndex++;
		}

	private:
		int64_t & lastRgbTime;
		ThreadMutexObject<int> & latestRgbIndex;
		std::pair<uint8_t *,int64_t> * rgbBuffers;
		rs::device * dev;
	};

	struct DepthCallback
	{
	public:
		DepthCallback(int64_t & lastDepthTime,
				ThreadMutexObject<int> & latestDepthIndex,
				ThreadMutexObject<int> & latestRgbIndex,
				std::pair<uint8_t *,int64_t> * rgbBuffers,
				std::pair<std::pair<uint8_t *,uint8_t *>,int64_t> * frameBuffers)
	: lastDepthTime(lastDepthTime),
	  latestDepthIndex(latestDepthIndex),
	  latestRgbIndex(latestRgbIndex),
	  rgbBuffers(rgbBuffers),
	  frameBuffers(frameBuffers)
	{
	}

		void setImageData(const void* data, int32_t width, int32_t height)
		{
			lastDepthTime = std::chrono::duration_cast<std::chrono::milliseconds>(
					std::chrono::system_clock::now().time_since_epoch()).count();

			int bufferIndex = (latestDepthIndex.getValue() + 1) % numBuffers;

			// The multiplication by 2 is here because the depth is actually uint16_t
//			std::copy((const uint8_t*)data, (const uint8_t*)data + height * width * 2, frameBuffers[bufferIndex].first.first);
			memcpy(frameBuffers[bufferIndex].first.first, data, height * width * 2);

			frameBuffers[bufferIndex].second = lastDepthTime;

			int lastImageVal = latestRgbIndex.getValue();

			if(lastImageVal == -1)
			{
				return;
			}

			lastImageVal %= numBuffers;

//			std::copy(rgbBuffers[lastImageVal].first, rgbBuffers[lastImageVal].first + height * width * 3, frameBuffers[bufferIndex].first.second);
			memcpy(frameBuffers[bufferIndex].first.second, rgbBuffers[lastImageVal].first, height * width * 3);

			latestDepthIndex++;
		}

		void operator()(rs::frame frame)
		{
			setImageData(frame.get_data(), frame.get_width(), frame.get_height());
		}

#ifdef WITH_REALSENSE_SDK
		void setImageInterface(rs::core::image_interface* image)
		{
			setImageData(image->query_data(), image->query_info().width, image->query_info().height);
		}
#endif

	private:
		int64_t & lastDepthTime;
		ThreadMutexObject<int> & latestDepthIndex;
		ThreadMutexObject<int> & latestRgbIndex;

		std::pair<uint8_t *,int64_t> * rgbBuffers;
		std::pair<std::pair<uint8_t *,uint8_t *>,int64_t> * frameBuffers;
	};

	virtual bool hasIntrinsics(){ return initSuccessful; }
	virtual void getIntrinsics(float& cx, float& cy, float& fx, float& fy)
	{
		fx = depth_intr.fx;
		fy = depth_intr.fy;
		cx = depth_intr.ppx;
		cy = depth_intr.ppy;
	}

#endif

#ifdef WITH_REALSENSE_SLAM
	class slam_event_handler : public rs::core::video_module_interface::processing_event_handler
	{
	public:
		void module_output_ready(rs::core::video_module_interface * sender, rs::core::correlated_sample_set * sample);
		void setDepthCallback(DepthCallback * callback){ depthCallback = callback; }
		void setLatestDepthIndex(ThreadMutexObject<int> * index){ latestDepthIndex_slam = index; }
		void setPoseBuffer(Pose* buffer){ poses_slam = buffer; }
	private:
		rs::slam::PoseMatrix4f pose;
		DepthCallback * depthCallback;
		ThreadMutexObject<int> * latestDepthIndex_slam;
		Pose* poses_slam;
	};

	virtual bool hasPose(){ return useSlam; }
#endif

	private:
#ifdef WITH_REALSENSE
	rs::device * dev;
	rs::source active_sources;

#ifdef WITH_REALSENSE_SDK
	rs::core::context_interface * ctx;
#else
	rs::context * ctx;
#endif
	rs::intrinsics depth_intr;

	RGBCallback * rgbCallback;
	DepthCallback * depthCallback;

#ifdef WITH_REALSENSE_SLAM
	std::unique_ptr<rs::slam::slam> slam;
	slam_event_handler slamEventHandler;
	rs::core::video_module_interface::actual_module_config slam_config;
	bool useSlam;
#endif
#endif

	bool initSuccessful;
	std::string errorText;

	ThreadMutexObject<int> latestRgbIndex;
	std::pair<uint8_t *,int64_t> rgbBuffers[numBuffers];

	int64_t lastRgbTime;
	int64_t lastDepthTime;

	std::thread frames_thread;
	bool stream_active;

};
