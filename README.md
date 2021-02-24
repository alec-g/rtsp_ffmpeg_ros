# rtsp_ffmpeg_ros
A package to publish RTSP video streams on a ROS image topic (including camera info). This package is an extension of video_stream_opencv https://github.com/ros-drivers/video_stream_opencv. Although the implmentation in video_stream_opencv for RTSP video streams works, the opencv VideoCapture implementation can introduce lag / delay in the image output due to the decoding process. This package implements FFMPEG for decoding the RTSP stream which results in a quicker pipeline and reduced output video lag.

## Setup
Install libraries to use ffmpeg
```
sudo apt-get install libavcodec-dev
sudo apt-get install libavformat-dev
sudo apt-get install libavutil-dev
sudo apt-get install libswscale-dev
sudo apt-get install libswresample-dev
sudo apt-get install libavdevice-dev
sudo apt-get install libavfilter-dev
```

## Usage
Example usages in launch folder (**only the argument `rtsp_url` is mandatory**):

```xml
<launch>
  <arg name="camera_name" default="front_camera" />
  <arg name="rtsp_url" default="rtsp://username:password@hostname:port/channel" />
  <arg name="frame_id" default="$(arg camera_name)" />
  <arg name="camera_info_url" default="file:///$(find rtsp_ffmpeg)/config/front_camera_calibration.yaml" />
  <arg name="flip_horizontal" default="false" />
  <arg name="flip_vertical" default="false" />

  <group ns="$(arg camera_name)">
    <node pkg="rtsp_ffmpeg" type="rtsp_ffmpeg" name="$(arg camera_name)_stream" output="screen"> 
      <remap from="camera" to="image_raw" />
      <param name="camera_name" type="string" value="$(arg camera_name)" />
      <param name="rtsp_url" type="string" value="$(arg rtsp_url)" />
      <param name="frame_id" type="string" value="$(arg frame_id)" />
      <param name="camera_info_url" type="string" value="$(arg camera_info_url)" />
      <param name="flip_horizontal" type="bool" value="$(arg flip_horizontal)" />
      <param name="flip_vertical" type="bool" value="$(arg flip_vertical)" />
    </node>
  </group>
</launch>

```
