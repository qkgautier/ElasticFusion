#include "RealSenseInterface.h"
#include <functional>


void frame_grabber_task(
		rs::device* dev,
		std::pair<std::pair<uint8_t *,uint8_t *>,int64_t> * frameBuffers,
		ThreadMutexObject<int> & latestDepthIndex,
		int numBuffers,
		bool& stream_active)
{
	while(stream_active && dev->is_streaming())
	{
		dev->wait_for_frames();

		rs::stream depth_stream = rs::stream::depth_aligned_to_rectified_color;
		rs::stream color_stream = rs::stream::rectified_color;

		const void* depth_data = dev->get_frame_data(depth_stream);
		const void* color_data = dev->get_frame_data(color_stream);

		int depth_width  = dev->get_stream_width(depth_stream);
		int depth_height = dev->get_stream_height(depth_stream);

		int color_width  = dev->get_stream_width(color_stream);
		int color_height = dev->get_stream_height(color_stream);

		int64_t lastDepthTime = std::chrono::duration_cast<std::chrono::milliseconds>(
				std::chrono::system_clock::now().time_since_epoch()).count();

		int bufferIndex = (latestDepthIndex.getValue() + 1) % numBuffers;

		// The multiplication by 2 is here because the depth is actually uint16_t
		memcpy(frameBuffers[bufferIndex].first.first, depth_data, depth_width * depth_height * 2);
		memcpy(frameBuffers[bufferIndex].first.second, color_data, color_width * color_height * 3);

		frameBuffers[bufferIndex].second = lastDepthTime;

		latestDepthIndex++;
	}
}



#ifdef WITH_REALSENSE_SLAM
#include "slam_utils.hpp"


static stream_stats inputStreamStats; // Dummy

void RealSenseInterface::slam_event_handler::module_output_ready(
		rs::core::video_module_interface * sender,
		rs::core::correlated_sample_set * sample)
{
	// Get a handle to the slam module
	auto slam = dynamic_cast<rs::slam::slam*>(sender);

    // Get the tracking accuracy as a string
	const std::string trackingAccuracy = tracking_accuracy_to_string(slam->get_tracking_accuracy());

    // Print the camera pose and number of updated occupancy map tiles to the console
//	std::cout << std::fixed << std::setprecision(2) << "Translation:(X=" << pose.m_data[3] << ", Y=" << pose.m_data[7] << ", Z=" << pose.m_data[11] << ")    Accuracy:" << trackingAccuracy << "\r" << std::flush;

	if(sample->images[(int)rs::core::stream_type::depth])
	{
		depthCallback->setImageInterface(sample->images[(int)rs::core::stream_type::depth]);


		// Get the camera pose;
		rs::core::status status = slam->get_camera_pose(pose);
		if(status != rs::core::status_no_error)
		{
			std::cerr << "Error retrieving the pose." << std::endl;
		}

	    int lastDepth = latestDepthIndex_slam->getValue();
	    int bufferIndex = lastDepth % numBuffers;
//	    std::copy(std::begin(pose.m_data), std::end(pose.m_data), std::begin(poses_slam[bufferIndex].data));
//	    std::copy(pose.m_data, pose.m_data + 16, poses_slam[bufferIndex].data);
	    for(int i = 0; i < 16; i++)
	    {
	    	std::cout << pose.m_data[i] << " ";
	    	poses_slam[bufferIndex].data[i] = pose.m_data[i];
	    }
	    std::cout << endl;

	    std::cout << bufferIndex << ": ";
	    for(int i = 0; i < 16; i++)
	    {
	    	std::cout << poses_slam[bufferIndex].data[i] << " ";
	    }
	    std::cout << endl;
	}
}
#endif


#ifdef WITH_REALSENSE
RealSenseInterface::RealSenseInterface(int inWidth, int inHeight, int inFps, const char* outFile, const char* inFile, bool tryUseSlam)
: width(inWidth),
  height(inHeight),
  fps(inFps),
  dev(nullptr),
  initSuccessful(true),
  stream_active(false)
{
#ifdef WITH_REALSENSE_SDK
	if(outFile)
	{
		ctx = new rs::record::context(outFile);
	}
	else if(inFile)
	{
		ctx = new rs::playback::context(inFile);
	}
	else
	{
		ctx = new rs::core::context();
	}
#else
	ctx = new rs::context();
#endif

	if(ctx->get_device_count() == 0)
	{
		errorText = "No device connected.";
		initSuccessful = false;
		return;
	}

	dev = ctx->get_device(0);



	latestDepthIndex.assign(-1);
	latestRgbIndex.assign(-1);

	for(int i = 0; i < numBuffers; i++)
	{
		uint8_t * newImage = (uint8_t *)calloc(width * height * 3,sizeof(uint8_t));
		rgbBuffers[i] = std::pair<uint8_t *,int64_t>(newImage,0);
	}

	for(int i = 0; i < numBuffers; i++)
	{
		uint8_t * newDepth = (uint8_t *)calloc(width * height * 2,sizeof(uint8_t));
		uint8_t * newImage = (uint8_t *)calloc(width * height * 3,sizeof(uint8_t));
		frameBuffers[i] = std::pair<std::pair<uint8_t *,uint8_t *>,int64_t>(std::pair<uint8_t *,uint8_t *>(newDepth,newImage),0);
	}

	for(int i = 0; i < numBuffers; i++)
	{
		poses[i] = Pose::Identity();
	}

	rgbCallback = new RGBCallback(lastRgbTime,
			latestRgbIndex,
			rgbBuffers,
			dev);

	depthCallback = new DepthCallback(lastDepthTime,
			latestDepthIndex,
			latestRgbIndex,
			rgbBuffers,
			frameBuffers);


	bool useSynchronousStreams = false;


	active_sources = rs::source::video;


#ifdef WITH_REALSENSE_SLAM

	useSlam = tryUseSlam;

	if(useSlam)
	{
		slam.reset(new rs::slam::slam());
		slam->register_event_handler(&slamEventHandler);
		slamEventHandler.setDepthCallback(depthCallback);
		slamEventHandler.setLatestDepthIndex(&latestDepthIndex);
		slamEventHandler.setPoseBuffer(poses);

		rs::core::video_module_interface::supported_module_config supported_slam_config = {};
		if (slam->query_supported_module_config(0, supported_slam_config) < rs::core::status_no_error)
		{
			errorText = "error : failed to query the first supported module configuration";
			initSuccessful = false;
			return;
		}

		auto device_name = dev->get_name();
		auto is_current_device_valid = (strcmp(device_name, supported_slam_config.device_name) == 0);

		active_sources = get_source_type(supported_slam_config);

		if(is_current_device_valid && check_motion_sensor_capability_if_required(dev, active_sources))
		{
			std::cout << "Using RealSense SLAM... ";

			dev->set_option(rs::option::fisheye_strobe, 1); // Needed to align image timestamps to common clock-domain with the motion events. Required for SLAM.
			dev->set_option(rs::option::fisheye_external_trigger, 1); // This option causes the fisheye image to be aquired in-sync with the depth image. Required for SLAM.

			// Enable fisheye stream
			auto &fisheye_stream_config = supported_slam_config[rs::core::stream_type::fisheye];
			dev->enable_stream(rs::stream::fisheye, fisheye_stream_config.size.width, fisheye_stream_config.size.height, rs::format::raw8, fisheye_stream_config.frame_rate);

			// Enable depth stream
			auto &depth_stream_config = supported_slam_config[rs::core::stream_type::depth];
			dev->enable_stream(rs::stream::depth, depth_stream_config.size.width, depth_stream_config.size.height, rs::format::z16, depth_stream_config.frame_rate);

			slam_config = get_slam_config(dev, slam, supported_slam_config);

			// Check that actual resolution and fps match required resolution and fps
			rs::core::video_module_interface::actual_image_stream_config actual_depth_config = slam_config.image_streams_configs[static_cast<uint32_t>(rs::core::stream_type::depth)];
			if(actual_depth_config.size.height != height || actual_depth_config.size.width != width || (int)actual_depth_config.frame_rate != fps)
			{
				errorText = "Required resolution and framerate don't match SLAM requirements!";
				initSuccessful = false;
				return;
			}

			set_callback_for_image_stream(dev, rs::core::stream_type::fisheye, slam, inputStreamStats);
			set_callback_for_image_stream(dev, rs::core::stream_type::depth, slam, inputStreamStats);
			set_callback_for_motion_streams(dev, slam, inputStreamStats);

			if(slam->set_module_config(slam_config) < status_no_error)
			{
				errorText = "error : failed to set the SLAM configuration";
				initSuccessful = false;
				return;
			}
		}
		else{ useSlam = false; }
	}

	if(useSlam){ useSynchronousStreams = false; }

	if(!useSlam)
#endif
	{
		std::cout << "Not using RealSense SLAM... ";

		dev->enable_stream(rs::stream::depth, width, height, rs::format::z16, fps);
		if(!useSynchronousStreams)
		{
			dev->set_frame_callback(rs::stream::depth, *depthCallback);
		}

		// Enable fisheye stream for recording only
		if(outFile && dev->get_stream_mode_count(rs::stream::fisheye) > 0)
		{
			dev->enable_stream(rs::stream::fisheye, width, height, rs::format::raw8, fps);
			if(!useSynchronousStreams){ dev->set_frame_callback(rs::stream::fisheye, [](rs::frame frame){}); }
		}

		// Enable motion events for recording only
		if(outFile && dev->supports(rs::capabilities::motion_events))
		{
			dev->enable_motion_tracking([](rs::motion_data motion_data){});
			active_sources = rs::source::all_sources;
		}
	}


	depth_intr = dev->get_stream_intrinsics(rs::stream::depth);


	setAutoExposure(true);
	setAutoWhiteBalance(true);

	dev->enable_stream(rs::stream::color,width,height,rs::format::rgb8,fps);
	if(!useSynchronousStreams)
	{
		dev->set_frame_callback(rs::stream::color,*rgbCallback);
	}

	stream_active = true;
	dev->start(active_sources);

	if(useSynchronousStreams)
	{
		std::cout << "Using synchronous stream... ";
		frames_thread = std::thread(frame_grabber_task, dev, frameBuffers, std::ref(latestDepthIndex), numBuffers, std::ref(stream_active));
	}


}

RealSenseInterface::~RealSenseInterface()
{
	if(initSuccessful)
	{
		stream_active = false;

		if(frames_thread.joinable())
		{
			frames_thread.join();
		}

		if(slam){ slam->flush_resources(); }
		dev->stop(active_sources);


		for(int i = 0; i < numBuffers; i++)
		{
			free(rgbBuffers[i].first);
		}

		for(int i = 0; i < numBuffers; i++)
		{
			free(frameBuffers[i].first.first);
			free(frameBuffers[i].first.second);
		}

		delete rgbCallback;
		delete depthCallback;
	}
	delete ctx;
}

void RealSenseInterface::setAutoExposure(bool value)
{
	dev->set_option(rs::option::color_enable_auto_exposure,value);
}

void RealSenseInterface::setAutoWhiteBalance(bool value)
{
	dev->set_option(rs::option::color_enable_auto_white_balance,value);
}

bool RealSenseInterface::getAutoExposure()
{
	return dev->get_option(rs::option::color_enable_auto_exposure);
}

bool RealSenseInterface::getAutoWhiteBalance()
{
	return dev->get_option(rs::option::color_enable_auto_white_balance);
}

#else

RealSenseInterface::RealSenseInterface(int inWidth,int inHeight,int inFps)
: width(inWidth),
  height(inHeight),
  fps(inFps),
  initSuccessful(false)
{
	errorText = "Compiled without Intel RealSense library";
}

RealSenseInterface::~RealSenseInterface()
{
}

void RealSenseInterface::setAutoExposure(bool value)
{
}

void RealSenseInterface::setAutoWhiteBalance(bool value)
{
}

bool RealSenseInterface::getAutoExposure()
{
	return false;
}

bool RealSenseInterface::getAutoWhiteBalance()
{
	return false;
}
#endif
