/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2016, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * @author Alec Gurman
 */

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <stdexcept>
#include <boost/filesystem.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/thread/thread.hpp>
#include <queue>
#include <mutex>
#include <rtsp_ffmpeg/RTSPFFmpegConfig.h>
#include <rtsp_ffmpeg/ffmpegdecoder.h>

namespace fs = boost::filesystem;

namespace rtsp_ffmpeg {

class RTSPFFmpegNodelet: public nodelet::Nodelet {

protected:
  boost::shared_ptr<ros::NodeHandle> nh, pnh;
  image_transport::CameraPublisher pub;
  boost::shared_ptr<dynamic_reconfigure::Server<RTSPFFmpegConfig> > dyn_srv;
  RTSPFFmpegConfig config;
  std::mutex s_mutex, c_mutex, p_mutex;
  boost::shared_ptr<FFmpegDecoder> decoder;
  std::string rtsp_url;
  int subscriber_num;
  bool capture_thread_running;
  boost::thread capture_thread;
  boost::thread decode_thread;
  sensor_msgs::CameraInfo cam_info_msg;

// Based on the ros tutorial on transforming opencv images to Image messages

virtual sensor_msgs::CameraInfo get_default_camera_info_from_image(sensor_msgs::ImagePtr img){
  sensor_msgs::CameraInfo cam_info_msg;
  cam_info_msg.header.frame_id = img->header.frame_id;
  // Fill image size
  cam_info_msg.height = img->height;
  cam_info_msg.width = img->width;

  // Add the most common distortion model as sensor_msgs/CameraInfo says
  cam_info_msg.distortion_model = "plumb_bob";
  // Don't let distorsion matrix be empty
  cam_info_msg.D.resize(5, 0.0);
  // Give a reasonable default intrinsic camera matrix
  cam_info_msg.K = boost::assign::list_of(img->width/2.0) (0.0) (img->width/2.0)
          (0.0) (img->height/2.0) (img->height/2.0)
          (0.0) (0.0) (1.0);
  // Give a reasonable default rectification matrix
  cam_info_msg.R = boost::assign::list_of (1.0) (0.0) (0.0)
          (0.0) (1.0) (0.0)
          (0.0) (0.0) (1.0);
  // Give a reasonable default projection matrix
  cam_info_msg.P = boost::assign::list_of (img->width/2.0) (0.0) (img->width/2.0) (0.0)
          (0.0) (img->height/2.0) (img->height/2.0) (0.0)
          (0.0) (0.0) (1.0) (0.0);
  return cam_info_msg;
}

virtual void do_capture() 
{
  NODELET_DEBUG("Capture thread started");
  cv::Mat frame;
  RTSPFFmpegConfig latest_config = config;

  capture_thread_running = true;
  while (nh->ok() && capture_thread_running && subscriber_num > 0 && decoder->isConnected()) {
    {
      std::lock_guard<std::mutex> lock(c_mutex);
      latest_config = config;
    } 

    decoder->mtx.lock();
    if(!decoder->decodedImgBuf.empty())
    {
      frame = decoder->decodedImgBuf.front().clone();
      decoder->decodedImgBuf.pop_front();
    }
    decoder->mtx.unlock();

    if(!frame.empty())
    {
      // Publish   
      do_publish(frame);

      frame.release();
    }
  }
}

// TODO: Move this to a thread to avoid any computational lag with converting to ROS message
// always use the latest frame available.
virtual void do_publish(cv::Mat frame)
{
  sensor_msgs::ImagePtr msg;
  std_msgs::Header header;
  RTSPFFmpegConfig latest_config = config;

  {
    std::lock_guard<std::mutex> lock(p_mutex);
    latest_config = config;
  }

  header.frame_id = latest_config.frame_id;

  if(!frame.empty()) {
    // From http://docs.opencv.org/modules/core/doc/operations_on_arrays.html#void flip(InputArray src, OutputArray dst, int flipCode)
    // FLIP_HORIZONTAL == 1, FLIP_VERTICAL == 0 or FLIP_BOTH == -1
    // Flip the image if necessary
    if (latest_config.flip_horizontal && latest_config.flip_vertical)
      cv::flip(frame, frame, -1);
    else if (latest_config.flip_horizontal)
      cv::flip(frame, frame, 1);
    else if (latest_config.flip_vertical)
      cv::flip(frame, frame, 0);

    cv_bridge::CvImagePtr cv_image =
      boost::make_shared<cv_bridge::CvImage>(header, "bgr8", frame);
    if (latest_config.output_encoding != "bgr8")
    {
      try {
        // https://github.com/ros-perception/vision_opencv/blob/melodic/cv_bridge/include/cv_bridge/cv_bridge.h#L247
        cv_image = cv_bridge::cvtColor(cv_image, latest_config.output_encoding);
      } catch (std::runtime_error &ex) {
        NODELET_ERROR_STREAM("cannot change encoding to " << latest_config.output_encoding
                              << ": " << ex.what());
      }
    }
    msg = cv_image->toImageMsg();
    // Create a default camera info if we didn't get a stored one on initialization
    if (cam_info_msg.distortion_model == ""){
        NODELET_WARN_STREAM("No calibration file given, publishing a reasonable default camera info.");
        cam_info_msg = get_default_camera_info_from_image(msg);
    }

    pub.publish(*msg, cam_info_msg, ros::Time::now());
  }
}

virtual void subscribe() {
  ROS_DEBUG("Subscribe");
  RTSPFFmpegConfig& latest_config = config;
  // initialize camera info publisher
  camera_info_manager::CameraInfoManager cam_info_manager(
      *nh, latest_config.camera_name, latest_config.camera_info_url);
  // Get the saved camera info if any
  cam_info_msg = cam_info_manager.getCameraInfo();
  cam_info_msg.header.frame_id = latest_config.frame_id;

  // Initialize decoder
  decoder.reset(new FFmpegDecoder(rtsp_url, *nh));
  NODELET_INFO_STREAM("Connecting to decoder with url: " << rtsp_url);

  decoder->connect();
  if(decoder->isConnected())
  {
    NODELET_INFO_STREAM("Decoder has connected, starting threads!");
    try {
      decode_thread = boost::thread(
        boost::bind(&FFmpegDecoder::decode, decoder));
      capture_thread = boost::thread(
        boost::bind(&RTSPFFmpegNodelet::do_capture, this));
    } catch (std::exception& e) {
      NODELET_ERROR_STREAM("Failed to start capture thread: " << e.what());
    }
  } else {
    NODELET_ERROR_STREAM("Decoder failed to connect!");
  }
}

virtual void unsubscribe() {
  ROS_DEBUG("Unsubscribe");

  capture_thread_running = false;
  capture_thread.join();

  decoder->stop();
  decode_thread.join();
  decoder.reset();
}

virtual void connectionCallbackImpl() {
  std::lock_guard<std::mutex> lock(s_mutex);
  subscriber_num++;
  NODELET_DEBUG_STREAM("Got connection callback, current subscribers: " << subscriber_num);
  if (subscriber_num == 1) {
    subscribe();
  }
}

virtual void disconnectionCallbackImpl() {
  std::lock_guard<std::mutex> lock(s_mutex);
  bool always_subscribe = false;
  pnh->getParamCached("always_subscribe", always_subscribe);
  if (always_subscribe) {
    return;
  }

  subscriber_num--;
  NODELET_DEBUG_STREAM("Got disconnection callback, current subscribers: " << subscriber_num);
  if (subscriber_num == 0) {
    unsubscribe();
  }
}

virtual void connectionCallback(const image_transport::SingleSubscriberPublisher&) {
  connectionCallbackImpl();
}

virtual void infoConnectionCallback(const ros::SingleSubscriberPublisher&) {
  connectionCallbackImpl();
}

virtual void disconnectionCallback(const image_transport::SingleSubscriberPublisher&) {
  disconnectionCallbackImpl();
}

virtual void infoDisconnectionCallback(const ros::SingleSubscriberPublisher&) {
  disconnectionCallbackImpl();
}

virtual void configCallback(RTSPFFmpegConfig& new_config, uint32_t level) {
  NODELET_DEBUG("configCallback");

  if (new_config.fps > new_config.set_camera_fps) {
    NODELET_WARN_STREAM(
        "Asked to publish at 'fps' (" << new_config.fps
        << ") which is higher than the 'set_camera_fps' (" << new_config.set_camera_fps <<
        "), we can't publish faster than the camera provides images.");
    new_config.fps = new_config.set_camera_fps;
  }

  {
    std::lock_guard<std::mutex> c_lock(c_mutex);
    std::lock_guard<std::mutex> p_lock(p_mutex);
    config = new_config;
  }

  // show current configuration
  NODELET_INFO_STREAM("Camera name: " << new_config.camera_name);
  NODELET_INFO_STREAM("Provided camera_info_url: '" << new_config.camera_info_url << "'");
  NODELET_INFO_STREAM("Publishing with frame_id: " << new_config.frame_id);
  NODELET_INFO_STREAM("Flip horizontal image is: " << ((new_config.flip_horizontal)?"true":"false"));
  NODELET_INFO_STREAM("Flip vertical image is: " << ((new_config.flip_vertical)?"true":"false"));

  NODELET_DEBUG_STREAM("subscriber_num: " << subscriber_num << " and level: " << level);
  if (subscriber_num > 0 && (level & 0x1))
  {
    NODELET_DEBUG("New dynamic_reconfigure config received on a parameter with configure level 1, unsubscribing and subscribing");
    unsubscribe();
    subscribe();
  }
}

virtual void onInit() {
  nh.reset(new ros::NodeHandle(getNodeHandle()));
  pnh.reset(new ros::NodeHandle(getPrivateNodeHandle()));
  subscriber_num = 0;

  // provider can be an url (e.g.: rtsp://10.0.0.1:554) or a number of device, (e.g.: 0 would be /dev/video0)
  pnh->param<std::string>("rtsp_url", rtsp_url, "rtsp://");

  // set parameters from dynamic reconfigure server
  dyn_srv = boost::make_shared<dynamic_reconfigure::Server<RTSPFFmpegConfig> >(*pnh);
  auto f = boost::bind(&RTSPFFmpegNodelet::configCallback, this, _1, _2);
  dyn_srv->setCallback(f);

  subscriber_num = 0;
  image_transport::SubscriberStatusCallback connect_cb =
    boost::bind(&RTSPFFmpegNodelet::connectionCallback, this, _1);
  ros::SubscriberStatusCallback info_connect_cb =
    boost::bind(&RTSPFFmpegNodelet::infoConnectionCallback, this, _1);
  image_transport::SubscriberStatusCallback disconnect_cb =
    boost::bind(&RTSPFFmpegNodelet::disconnectionCallback, this, _1);
  ros::SubscriberStatusCallback info_disconnect_cb =
    boost::bind(&RTSPFFmpegNodelet::infoDisconnectionCallback, this, _1);
  pub = image_transport::ImageTransport(*nh).advertiseCamera(
    "image_raw", 1,
    connect_cb, disconnect_cb,
    info_connect_cb, info_disconnect_cb,
    ros::VoidPtr(), false
  );
}

virtual ~RTSPFFmpegNodelet() {
  if (subscriber_num > 0)
    subscriber_num = 0;
    unsubscribe();
}
};
} // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rtsp_ffmpeg::RTSPFFmpegNodelet, nodelet::Nodelet)
