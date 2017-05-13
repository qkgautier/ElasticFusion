#pragma once

#include <string>
#include <iostream>
#include <algorithm>
#include <map>

#include "ThreadMutexObject.h"

class CameraInterface
{
public:
		struct Pose
		{
			float data[16];
			static Pose Zeros(){Pose p; std::fill(std::begin(p.data), std::end(p.data), 0.0f); return p; }
			static Pose Identity(){ Pose p = Pose::Zeros(); p.data[0] = p.data[5] = p.data[10] = p.data[15] = 1.f; return p; }
		};

    public:
      virtual ~CameraInterface() {}

      virtual bool ok() = 0;
      virtual std::string error() = 0;

      static const int numBuffers = 10;
      ThreadMutexObject<int> latestDepthIndex;
      std::pair<std::pair<uint8_t *,uint8_t *>,int64_t> frameBuffers[numBuffers];
      Pose poses[numBuffers];

      virtual void setAutoExposure(bool value) = 0;
      virtual void setAutoWhiteBalance(bool value) = 0;
      virtual bool hasIntrinsics(){ return false; }
      virtual void getIntrinsics(float& cx, float& cy, float& fx, float& fy){}
      virtual bool hasPose(){ return false; }

};
