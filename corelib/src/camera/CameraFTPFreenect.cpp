/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <rtabmap/core/camera/CameraFTPFreenect.h>
#include <rtabmap/utilite/UThread.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/core/util2d.h>
#include <opencv2/imgproc/types_c.h>
#include <experimental/filesystem>
#include <iostream>
#include <unistd.h>
#include <thread>


const int TEST = true;
int iii = 0;
const int MAX = 750;

namespace rtabmap
{

class FreenectFTPDevice : public UThread {
  public:
	FreenectFTPDevice(freenect_context * ctx, int index, bool color = true, bool registered = true) :
		index_(index),
		color_(color),
		registered_(registered),
		ctx_(ctx),
		device_(0),
		depthFocal_(0.0f)
	{
	}

	virtual ~FreenectFTPDevice()
	{
		this->join(true);
	}

	const std::string & getSerial() const {
		return serial_;
	}

	bool init()
	{
		printf("inint1");
		rgbIrBuffer_ = cv::Mat(cv::Size(640,480), color_?CV_8UC3:CV_8UC1);
		depthBuffer_ = cv::Mat(cv::Size(640,480), CV_16UC1);
		float rgb_focal_length_sxga = 1050.0f;
		float width_sxga = 1280.0f;
		float width = 640.0f;
		float scale = width / width_sxga;
		depthFocal_ =  rgb_focal_length_sxga * scale;

		UINFO("FreenectFTPDevice: Depth focal = %f", depthFocal_);
		printf("init1.1\n");
		return true;
	}

	float getDepthFocal() const {return depthFocal_;}

	void getData(cv::Mat & rgb, cv::Mat & depth)
	{

	}

	void getAccelerometerValues(double & x, double & y, double & z)
	{

	}

private:
	// Do not call directly even in child
	void VideoCallback(void* rgb){}

	// Do not call directly even in child
	void DepthCallback(void* depth){}

	void startVideo() {}
	void stopVideo() {}
	void startDepth() {}
	void stopDepth() {}

	virtual void mainLoopBegin(){}

	virtual void mainLoop(){}

	virtual void mainLoopEnd(){}

	static void freenect_depth_callback(freenect_device *dev, void *depth, uint32_t timestamp) {}
	static void freenect_video_callback(freenect_device *dev, void *video, uint32_t timestamp) {}

	//noncopyable
	FreenectFTPDevice( const FreenectFTPDevice& );
	const FreenectFTPDevice& operator=( const FreenectFTPDevice& );

  private:
	int index_;
	bool color_;
	bool registered_;
	std::string serial_;
	freenect_context * ctx_;
	freenect_device * device_;
	cv::Mat depthBuffer_;
	cv::Mat rgbIrBuffer_;
	UMutex dataMutex_;
	cv::Mat depthLastFrame_;
	cv::Mat rgbIrLastFrame_;
	float depthFocal_;
	USemaphore dataReady_;
};

//
// CameraFTPFreenect
//
bool CameraFTPFreenect::available()
{
	return true;
}

CameraFTPFreenect::CameraFTPFreenect(int deviceId, Type type, float imageRate, const Transform & localTransform) :
		Camera(imageRate, localTransform)
        ,
		deviceId_(deviceId),
		type_(type),
		ctx_(0),
		FreenectFTPDevice_(0)
{
}

CameraFTPFreenect::~CameraFTPFreenect()
{
	printf("thing\n");
	if(FreenectFTPDevice_)
	{
		printf("if thing\n");
		FreenectFTPDevice_->join(true);
		delete FreenectFTPDevice_;
		FreenectFTPDevice_ = 0;
	}
	printf("done thing\n");
}

bool CameraFTPFreenect::init(const std::string & calibrationFolder, const std::string & cameraName)
{
	printf(">>init2\n");
	bool hardwareRegistration = true;
	FreenectFTPDevice_ = new FreenectFTPDevice(ctx_, deviceId_, type_==kTypeColorDepth, hardwareRegistration);
	FreenectFTPDevice_->init();
	FreenectFTPDevice_->start();
	printf(">>init2.1\n");
	return true;
}

bool CameraFTPFreenect::isCalibrated() const
{
	return true;
}

std::string CameraFTPFreenect::getSerial() const
{
	return "";
}

SensorData CameraFTPFreenect::captureImage(CameraInfo * info)
{
	printf(">>captureImage\n");
	SensorData data;
	if( FreenectFTPDevice_) //ctx_ &&
	{
		printf(">>2\n");
		if(FreenectFTPDevice_->isRunning())
		{
			printf(">>3\n");
			cv::Mat depth,rgb;
			if(TEST){
				iii += 1;
				usleep(200000);
				rgb = cv::imread(("/root/rgbd_mapping/RGBDMapping/rgb_data/" + std::to_string((iii % MAX)) + ".png").c_str(), cv::IMREAD_COLOR).clone();
				depth = cv::imread( ("/root/rgbd_mapping/RGBDMapping/depth_data/" + std::to_string((iii % MAX)) + ".png").c_str(), cv::IMREAD_UNCHANGED).clone();
				for ( int ww = 0; ww < 640; ww++){
					for (int hh = 0; hh < 480; hh++){
						depth.at<ushort>(hh,ww) = depth.at<ushort>(hh,ww) >> 3;
					}
				}
			} else {
				FreenectFTPDevice_->getData(rgb, depth);
			}
			
			if(!rgb.empty() && !depth.empty())
			{
				UASSERT(FreenectFTPDevice_->getDepthFocal() != 0.0f);

				// default calibration
				CameraModel model(
						FreenectFTPDevice_->getDepthFocal(), //fx
						FreenectFTPDevice_->getDepthFocal(), //fy
						float(rgb.cols/2) - 0.5f,  //cx
						float(rgb.rows/2) - 0.5f,  //cy
						this->getLocalTransform(),
						0,
						rgb.size());

				if(type_==kTypeIRDepth)
				{
					if(stereoModel_.left().isValidForRectification())
					{
						rgb = stereoModel_.left().rectifyImage(rgb);
						depth = stereoModel_.left().rectifyImage(depth, 0);
						model = stereoModel_.left();
					}
				}
				else
				{
					if(stereoModel_.right().isValidForRectification())
					{
						rgb = stereoModel_.right().rectifyImage(rgb);
						model = stereoModel_.right();

						if(stereoModel_.left().isValidForRectification() && !stereoModel_.stereoTransform().isNull())
						{
							depth = stereoModel_.left().rectifyImage(depth, 0);
							depth = util2d::registerDepth(depth, stereoModel_.left().K(), rgb.size(), stereoModel_.right().K(), stereoModel_.stereoTransform());
						}
					}
				}
				model.setLocalTransform(this->getLocalTransform());

				data = SensorData(rgb, depth, model, this->getNextSeqID(), UTimer::now());

				double x=0,y=0,z=0;
				FreenectFTPDevice_->getAccelerometerValues(x,y,z);
				if(x != 0.0 && y != 0.0 && z != 0.0)
				{
					 Transform opticalTransform(0,-1,0,0, 0,0,-1,0, 1,0,0,0);
					 Transform base = this->getLocalTransform()*opticalTransform;
					// frame of imu on kinect is x->left, y->up, z->forward
					data.setIMU(IMU(cv::Vec3d(0,0,0), cv::Mat(), cv::Vec3d(x, y, z), cv::Mat(), base*Transform(0,0,1,0, 1,0,0,0, 0,1,0,0)));
				}
			}
		}
		else
		{
			UERROR("CameraFTPFreenect: Re-initialization needed!");
			delete FreenectFTPDevice_;
			FreenectFTPDevice_ = 0;
		}
	}
	//UERROR("CameraFTPFreenect: RTAB-Map is not built with Freenect support!");
	return data;
}


} // namespace rtabmap
