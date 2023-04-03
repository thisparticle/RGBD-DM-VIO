/**
* ROS driver for DM-VIO written by Lukas von Stumberg (http://vision.in.tum.de/dm-vio).
* This file is in part based on the file main_dso_pangolin.cpp of the project DSO as well as the ROS driver for DSO,
* both written by Jakob Engel.
*
* Copyright 2022 Lukas von Stumberg <lukas dot stumberg at tum dot de>
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DM-VIO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DM-VIO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DM-VIO. If not, see <http://www.gnu.org/licenses/>.
*/

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include "cv_bridge/cv_bridge.h"

#include <thread>
#include <locale.h>
#include <stdlib.h>
#include <stdio.h>

#include "IOWrapper/Output3DWrapper.h"

#include "util/Undistort.h"


#include <boost/thread.hpp>
#include "dso/util/settings.h"
#include "dso/util/globalCalib.h"
#include "util/TimeMeasurement.h"

#include "FullSystem/FullSystem.h"

#include <util/SettingsUtil.h>

#include "IOWrapper/Pangolin/PangolinDSOViewer.h"

#include "util/MainSettings.h"
#include "live/FrameSkippingStrategy.h"
#include "live/IMUInterpolator.h"
#include "ROSOutputWrapper.h"

#include <live/FrameContainer.h>

//2022.09.21 czq
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include <map>
//2022.09.21 czq

using namespace dso;
// using namespace std;

dmvio::FrameContainer frameContainer;
dmvio::IMUInterpolator imuInt(frameContainer, nullptr);
dmvio::MainSettings mainSettings;
dmvio::IMUCalibration imuCalibration;
dmvio::IMUSettings imuSettings;
dmvio::FrameSkippingSettings frameSkippingSettings;
std::unique_ptr<Undistort> undistorter;
bool stopSystem = false;
int start = 2;

//2022.09.21 czq
std::map<double, ImageAndExposure*> mapDepth;
//2022.09.21 czq



void run(IOWrap::PangolinDSOViewer* viewer)
{
    bool linearizeOperation = false;
    auto fullSystem = std::make_unique<FullSystem>(linearizeOperation, imuCalibration, imuSettings);

    if(setting_photometricCalibration > 0 && undistorter->photometricUndist == nullptr)
    {
        printf("ERROR: dont't have photometric calibation. Need to use commandline options mode=1 or mode=2 ");
        exit(1);
    }

    if(undistorter->photometricUndist != nullptr)
    {
        fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
    }

    if(viewer)
    {
        fullSystem->outputWrapper.push_back(viewer);
    }

    dmvio::FrameSkippingStrategy frameSkipping(frameSkippingSettings);
    // frameSkipping registers as an outputWrapper to get notified of changes of the system status.
    fullSystem->outputWrapper.push_back(&frameSkipping);

    // This will handle publishing to ROS topics.
    dmvio::ROSOutputWrapper rosOutput;
    fullSystem->outputWrapper.push_back(&rosOutput);

    int ii = 0;
    int lastResetIndex = 0;

    while(!stopSystem)
    {
        //2022.09.21 czq
        // Skip the first few frames if the start variable is set.
        if(start > 0 && ii < start)
        {
            auto pair = frameContainer.getImageAndIMUData();

            ++ii;
            continue;
        }

        auto pair = frameContainer.getImageAndIMUData(frameSkipping.getMaxSkipFrames(frameContainer.getQueueSize()));

        if(!pair.first) continue;

        ImageAndExposure* Img;
        ImageAndExposure* depthImg;
        double image_time = pair.first.get()->timestamp;
        // depthImg = mapDepth.find(image_time);
        depthImg = mapDepth.at(image_time);
        Img = pair.first.get();

        fullSystem->addActiveFrame(pair.first.get(), depthImg, ii, &(pair.second), nullptr);

        // cv::Mat m1(Img->h, Img->w, CV_16UC1);
        // memcpy(m1.data, Img->image, sizeof(float) * Img->h*Img->w);
        // m1.convertTo(m2, CV_16UC1);
        // cv::imshow("image_t", m1);

        // cv::Mat m2(depthImg->h, depthImg->w, CV_32FC1);
        // memcpy(m2.data, depthImg->image, sizeof(float) * depthImg->h*depthImg->w);
        // m2.convertTo(m2, CV_16UC1);
        // cv::imshow("depthimage_t", m2);
        // cv::waitKey(0);

        mapDepth.erase(image_time);
        // delete Img;
        // delete depthImg;
        // if(start > 0 && ii < start)
        // {
        //     auto tupledata = frameContainer.get2ImageAndIMUData();

        //     ++ii;
        //     continue;
        // }


        // auto tupledata = frameContainer.get2ImageAndIMUData(frameSkipping.getMaxSkipFrames(frameContainer.getQueueSize()));

        // if(!std::get<0>(tupledata)) continue;

        // fullSystem->addActiveFrame(std::get<0>(tupledata).get(), std::get<1>(tupledata).get(), ii, &(std::get<2>(tupledata)), nullptr);
        //2022.09.21 czq

        if(fullSystem->initFailed || setting_fullResetRequested)
        {
            if(ii - lastResetIndex < 250 || setting_fullResetRequested)
            {
                printf("RESETTING!\n");
                std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
                fullSystem.reset();
                for(IOWrap::Output3DWrapper* ow : wraps) ow->reset();

                fullSystem = std::make_unique<FullSystem>(linearizeOperation, imuCalibration, imuSettings);
                if(undistorter->photometricUndist != nullptr)
                {
                    fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
                }
                fullSystem->outputWrapper = wraps;

                setting_fullResetRequested = false;
                lastResetIndex = ii;
            }
        }

        if(viewer != nullptr && viewer->shouldQuit())
        {
            std::cout << "User closed window -> Quit!" << std::endl;
            break;
        }

        if(fullSystem->isLost)
        {
            printf("LOST!!\n");
            break;
        }

        ++ii;

    }

    fullSystem->blockUntilMappingIsFinished();

    fullSystem->printResult(imuSettings.resultsPrefix + "result.txt", false, false, true);

    dmvio::TimeMeasurement::saveResults(imuSettings.resultsPrefix + "timings.txt");

    for(IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper)
    {
        ow->join();
    }

    printf("DELETE FULLSYSTEM!\n");
    fullSystem.reset();

    ros::shutdown();

    printf("EXIT NOW!\n");
}

double convertStamp(const ros::Time& time)
{
    // We need the timstamp in seconds as double
    return time.sec * 1.0 + time.nsec / 1000000000.0;
}

void img_callback(const sensor_msgs::ImageConstPtr &color_msg, const sensor_msgs::ImageConstPtr &depth_msg)
{
    double stamp = convertStamp(color_msg->header.stamp);
    cv_bridge::CvImageConstPtr cv_ptr;
    if (color_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = color_msg->header;
        img.height = color_msg->height;
        img.width = color_msg->width;
        img.is_bigendian = color_msg->is_bigendian;
        img.step = color_msg->step;
        img.data = color_msg->data;
        img.encoding = "mono8";
        cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        cv_ptr = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::MONO8);

    cv_bridge::CvImageConstPtr depth_ptr;
	if (depth_msg->encoding == "16UC1"||depth_msg->encoding == "mono16")
    {
		depth_ptr = cv_bridge::toCvCopy(depth_msg);
    }
	else if (depth_msg->encoding == "32FC1")
	{
		// depth_ptr = cv_bridge::toCvCopy(depth_msg);
		cv::Mat depth_img;
		cv::Mat depth_32fc1 = cv_bridge::toCvCopy(depth_msg)->image;
		depth_32fc1.convertTo(depth_img, CV_16UC1, 10000);
		sensor_msgs::Image img;
        img.header = depth_msg->header;
        img.height = depth_msg->height;
        img.width = depth_msg->width;
        img.is_bigendian = depth_msg->is_bigendian;
        img.step = depth_msg->step;
        img.data = depth_msg->data;
        img.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        depth_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_16UC1);
		memcpy(depth_ptr->image.data,  depth_img.data,  2*depth_ptr->image.cols*depth_ptr->image.rows);
	}

    MinimalImageB minImg((int) cv_ptr->image.cols, (int) cv_ptr->image.rows, (unsigned char*) cv_ptr->image.data);
    // Unfortunately the image message does not contain exposure. This means that you cannot use photometric
    // mode 1. But mode 0 will entirely disable the vignette which is far from optimal for fisheye cameras.
    // You can use the new mode 3 however which uses vignette, but does not assume that a full photometric
    // calibration is available.
    // Alternatively, if exposure is published on a different topic you can synchronize them an pass the exposure to
    // undistorter->undistort in the next line.
    std::unique_ptr<ImageAndExposure> undistImg(undistorter->undistort<unsigned char>(&minImg, 1.0, stamp, 1.0f));

    MinimalImage<unsigned short> minDepthImg((int) depth_ptr->image.cols ,(int) depth_ptr->image.rows);
	memcpy(minDepthImg.data,  depth_ptr->image.data,  2*depth_ptr->image.cols*depth_ptr->image.rows);
    ImageAndExposure* undistDepth_Img(undistorter->transformDepthImage<unsigned short>(&minDepthImg, 1, 0));

    undistImg->timestamp = stamp; // relay the timestamp to dso
    // undistDepth_Img->timestamp = color_msg->header.stamp.toSec(); // relay the timestamp to dso

    imuInt.addImage(std::move(undistImg), stamp);
    // mapDepth.insert(make_pair(stamp, undistDepth_Img));
    mapDepth[stamp] = undistDepth_Img;
    // imuInt.add2Image(std::move(undistImg), std::move(undistDepth_Img), stamp);
    // std::cout << "imuInt.addAccData.size(): " << imuInt.gyrData << std::endl;
    // std::cout << "imuInt.addAccData.size(): " << imuInt.A << std::endl;
}

// void vidCb(const sensor_msgs::ImageConstPtr img)
// {
//     double stamp = convertStamp(img->header.stamp);

//     cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
//     assert(cv_ptr->image.type() == CV_8U);
//     assert(cv_ptr->image.channels() == 1);

//     MinimalImageB minImg((int) cv_ptr->image.cols, (int) cv_ptr->image.rows, (unsigned char*) cv_ptr->image.data);
//     // Unfortunately the image message does not contain exposure. This means that you cannot use photometric
//     // mode 1. But mode 0 will entirely disable the vignette which is far from optimal for fisheye cameras.
//     // You can use the new mode 3 however which uses vignette, but does not assume that a full photometric
//     // calibration is available.
//     // Alternatively, if exposure is published on a different topic you can synchronize them an pass the exposure to
//     // undistorter->undistort in the next line.
//     std::unique_ptr<ImageAndExposure> undistImg(undistorter->undistort<unsigned char>(&minImg, 1.0, stamp, 1.0f));

//     imuInt.addImage(std::move(undistImg), stamp);
// }

void imuCb(const sensor_msgs::ImuConstPtr imu)
{
    std::vector<float> accData;
    accData.push_back(imu->linear_acceleration.x);
    accData.push_back(imu->linear_acceleration.y);
    accData.push_back(imu->linear_acceleration.z);

    std::vector<float> gyrData;
    gyrData.push_back(imu->angular_velocity.x);
    gyrData.push_back(imu->angular_velocity.y);
    gyrData.push_back(imu->angular_velocity.z);

    // std::cout << "accData.size(): " << accData.size() << "gyrData.size()" << gyrData.size() << std::endl;

    ros::Time time = imu->header.stamp;
    double timestamp = convertStamp(time);
    imuInt.addAccData(accData, timestamp);
    imuInt.addGyrData(gyrData, timestamp);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "d2vio");
    ros::NodeHandle nh;

    setlocale(LC_ALL, "C");

    std::string imu_topic, cam_topic, depth_topic,settingsFile;
    int preset;
    if (!nh.getParam("useimu", setting_useIMU)||
        !nh.getParam("quiet", setting_debugout_runquiet)||
        !nh.getParam("nogui", disableAllDisplay)||
        // !nh.getParam("init_disableVIOUntilFirstInit", setting_init_disableVIOUntilFirstInit)||
        // !nh.getParam("RGBDInitializerDebug", RGBDInitializerDebug)||
        !nh.getParam("imu_topic", imu_topic) ||
        !nh.getParam("cam_topic", cam_topic) ||
        !nh.getParam("settingsFile", settingsFile)||
        !nh.getParam("depth_topic", depth_topic)) {
        ROS_INFO("Fail to get sensor topics/params, exit.!!!!");
        return -1;
    }

#ifdef DEBUG
    std::cout << "DEBUG MODE!" << std::endl;
#endif

    auto settingsUtil = std::make_shared<dmvio::SettingsUtil>();

    // Create Settings files.
    imuSettings.registerArgs(*settingsUtil);
    imuCalibration.registerArgs(*settingsUtil);
    mainSettings.registerArgs(*settingsUtil);
    frameSkippingSettings.registerArgs(*settingsUtil);

    settingsUtil->registerArg("start", start);

    auto normalizeCamSize = std::make_shared<double>(0.0);
    settingsUtil->registerArg("normalizeCamSize", *normalizeCamSize, 0.0, 5.0);

    // This call will parse all commandline arguments and potentially also read a settings yaml file if passed.
    // mainSettings.parseArguments(argc, argv, *settingsUtil);
    //2022.09.21 czq
    mainSettings.parseROSArgument(*settingsUtil, settingsFile);
    // settingsUtil->registerArg("preset", preset);
    //2022.09.21 czq

    // Print settings to commandline and file.
    std::cout << "setting_useIMU: " << setting_useIMU << std::endl;
    std::cout << "Settings:\n";
    settingsUtil->printAllSettings(std::cout);
    {
        std::ofstream settingsStream;
        settingsStream.open(imuSettings.resultsPrefix + "usedSettingsdso.txt");
        settingsUtil->printAllSettings(settingsStream);
    }

    undistorter.reset(
            Undistort::getUndistorterForFile(mainSettings.calib, mainSettings.gammaCalib, mainSettings.vignette));

    setGlobalCalib(
            (int) undistorter->getSize()[0],
            (int) undistorter->getSize()[1],
            undistorter->getK().cast<float>());

    imuCalibration.loadFromFile(mainSettings.imuCalibFile);

    std::unique_ptr<IOWrap::PangolinDSOViewer> viewer;

    if(!disableAllDisplay)
    {
        viewer = std::make_unique<IOWrap::PangolinDSOViewer>(wG[0], hG[0], true, settingsUtil, normalizeCamSize);
    }

    boost::thread runThread = boost::thread(boost::bind(run, viewer.get()));
    // boost::thread runThread = boost::thread(boost::bind(run_test, viewer.get()));

    message_filters::Subscriber<sensor_msgs::Image> sub_image(nh, cam_topic, 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_depth(nh, depth_topic, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(200), sub_image, sub_depth);
    sync.registerCallback(boost::bind(&img_callback, _1, _2));
    // ros::Subscriber imageSub = nh.subscribe("cam0/image_raw", 3, &vidCb);
    ros::Subscriber imuSub = nh.subscribe(imu_topic, 50, &imuCb);

    ros::spin();
    stopSystem = true;
    frameContainer.stop();

    // Make sure that the destructor of FullSystem, etc. finishes, so all log files are properly flushed.
    runThread.join();

    return 0;
}