/*********************************************************************
 *
 *  Copyright (c) 2014, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

 *  Author: Julius Kammerl (jkammerl@willowgarage.com)
 *
 */

#include "depthcloud_encoder/depthcloud_encoder.h"

#include <boost/bind.hpp>
#include <boost/cstdint.hpp>

#include <cv_bridge/cv_bridge.h>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf_conversions/tf_eigen.h>

#include <iostream>

#include <ros/console.h>

namespace depthcloud
{

using namespace message_filters::sync_policies;
namespace enc = sensor_msgs::image_encodings;

DepthCloudEncoder::DepthCloudEncoder(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
    nh_(nh),
    pnh_(pnh),
    depth_sub_(),
    color_sub_(),
    pub_it_(nh_),
    connectivityExceptionFlag(false),
    lookupExceptionFlag(false)
{
  // ROS parameters
  ros::NodeHandle priv_nh_("~");

  // read source from param server
  priv_nh_.param<std::string>("depth_source", depth_source_, "depthmap");

  // read point cloud topic from param server
  priv_nh_.param<std::string>("cloud", cloud_topic_, "");

  // read camera info topic from param server.
  priv_nh_.param<std::string>("camera_info_topic", camera_info_topic_, "");

  // The original frame id of the camera that captured this cloud
  priv_nh_.param<std::string>("camera_frame_id", camera_frame_id_, "/camera_rgb_optical_frame");

  // read focal length value from param server in case a cloud topic is used.
  priv_nh_.param<double>("f", f_, 525.0);

  // read focal length multiplication factor value from param server in case a cloud topic is used.
  priv_nh_.param<double>("f_mult_factor", f_mult_factor_, 1.0);

  // read max depth per tile from param server
  priv_nh_.param<float>("max_depth_per_tile", max_depth_per_tile_, 1.0);

  // read target resolution from param server.
  priv_nh_.param<int>("target_resolution", crop_size_, 512);

  // read depth map topic from param server
  priv_nh_.param<std::string>("depth", depthmap_topic_, "/camera/depth/image");

  // read rgb topic from param server
  priv_nh_.param<std::string>("rgb", rgb_image_topic_, "/camera/rgb/image_color");

  // Whether the encoded topic should be latched.
  priv_nh_.param<bool>("latch", latch_, false);

  // Monitor whether anyone is subscribed to the output
  image_transport::SubscriberStatusCallback connect_cb = boost::bind(&DepthCloudEncoder::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to pub_point_cloud_
  boost::lock_guard<boost::mutex> lock(connect_mutex_);

  pub_ = pub_it_.advertise("depthcloud_encoded", 1, connect_cb, connect_cb, ros::VoidPtr(), latch_);
}
DepthCloudEncoder::~DepthCloudEncoder()
{
}

void DepthCloudEncoder::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);

  if (pub_.getNumSubscribers())
  {
    if ( depth_source_ == "depthmap" && !depthmap_topic_.empty() )
      subscribe(depthmap_topic_, rgb_image_topic_);
    else if ( depth_source_ == "cloud" && !cloud_topic_.empty() )
      subscribeCloud(cloud_topic_);
    else {
      if ( depth_source_ != "depthmap" && depth_source_ != "cloud" ) {
        ROS_ERROR("Invalid depth_source given to DepthCloudEncoder: use 'depthmap' or 'cloud'.");
        return;
      }
      ROS_ERROR_STREAM("Empty topic provided for DepthCloudEncoder depth_source " << depth_source_ << ". Check your arguments.");
    }
  }
  else
  {
    unsubscribe();
  }
}

void DepthCloudEncoder::cameraInfoCb(const sensor_msgs::CameraInfoConstPtr& cam_info_msg)
{
  if (cam_info_msg) {
    // Update focal length value.
    const double fx = f_mult_factor_ * cam_info_msg->K[0];
    const double fy = f_mult_factor_ * cam_info_msg->K[4];
    f_ = (fx + fy) / 2;
  }
}

void DepthCloudEncoder::dynReconfCb(depthcloud_encoder::paramsConfig& config, uint32_t level)
{
  max_depth_per_tile_ = config.max_depth_per_tile;
  f_mult_factor_ = config.f_mult_factor;
}

void DepthCloudEncoder::subscribeCloud(std::string& cloud_topic)
{
  unsubscribe();

  ROS_DEBUG_STREAM("Subscribing to "<< cloud_topic);

  // subscribe to depth cloud topic
  cloud_sub_ = nh_.subscribe(cloud_topic, 1, &DepthCloudEncoder::cloudCB, this );
  if (!camera_info_topic_.empty()) {
    camera_info_sub_ = nh_.subscribe(camera_info_topic_, 2, &DepthCloudEncoder::cameraInfoCb, this);
  }
}

void DepthCloudEncoder::subscribe(std::string& depth_topic, std::string& color_topic)
{
  unsubscribe();

  ROS_DEBUG_STREAM("Subscribing to "<< color_topic);
  ROS_DEBUG_STREAM("Subscribing to "<< depth_topic);

  if (!depth_topic.empty())
  {
    try
    {
      image_transport::ImageTransport depth_it(pnh_);
      image_transport::ImageTransport color_it(pnh_);

      // subscribe to depth map topic
      depth_sub_->subscribe(depth_it, depth_topic, 1, image_transport::TransportHints("raw"));

      if (!color_topic.empty())
      {
        // subscribe to color image topic
        color_sub_->subscribe(color_it, color_topic, 1, image_transport::TransportHints("raw"));

        // connect message filters to synchronizer
        sync_depth_color_->connectInput(*depth_sub_, *color_sub_);
        sync_depth_color_->setInterMessageLowerBound(0, ros::Duration(1.5));
        sync_depth_color_->setInterMessageLowerBound(1, ros::Duration(1.5));
        sync_depth_color_->registerCallback(boost::bind(&DepthCloudEncoder::depthColorCB, this, _1, _2));
      }
      else
      {
        depth_sub_->registerCallback(boost::bind(&DepthCloudEncoder::depthCB, this, _1));
      }
    }
    catch (ros::Exception& e)
    {
      ROS_ERROR("Error subscribing: %s", e.what());
    }
  }
}


void DepthCloudEncoder::unsubscribe()
{
  try
  {
    // reset all message filters
    cloud_sub_.shutdown();
    camera_info_sub_.shutdown();
    sync_depth_color_.reset(new SynchronizerDepthColor(SyncPolicyDepthColor(10)));
    depth_sub_.reset(new image_transport::SubscriberFilter());
    color_sub_.reset(new image_transport::SubscriberFilter());
  }
  catch (ros::Exception& e)
  {
    ROS_ERROR("Error unsubscribing: %s", e.what());
  }
}

void DepthCloudEncoder::cloudCB(const sensor_msgs::PointCloud2& cloud_msg)
{
  sensor_msgs::ImagePtr depth_msg( new sensor_msgs::Image() );
  sensor_msgs::ImagePtr color_msg( new sensor_msgs::Image() );
  /* For depth:
    height: 480
     width: 640
     encoding: 32FC1
     is_bigendian: 0
     step: 2560
  */
  /* For color:
     height: 480
     width: 640
     encoding: rgb8
     is_bigendian: 0
     step: 1920
  */

  color_msg->height = depth_msg->height = cloud_msg.height;
  color_msg->width  = depth_msg->width  = cloud_msg.width;
  depth_msg->encoding = "32FC1";
  color_msg->encoding = "rgb8";
  color_msg->is_bigendian = depth_msg->is_bigendian = 0;
  depth_msg->step = depth_msg->width * 4;
  color_msg->step = color_msg->width * 3;
  // 480 (default) rows of 2560 bytes.
  depth_msg->data.resize(depth_msg->height * depth_msg->step);
  // 480 (default) rows of 1920 bytes.
  color_msg->data.resize(color_msg->height * color_msg->step, 0);
  for (int j=0; j < depth_msg->height; ++j) {
    for (int i =0; i < depth_msg->width; ++i) {
      *(float*)&depth_msg->data[ j * cloud_msg.width * 4 + i * 4 ] =
          std::numeric_limits<float>::quiet_NaN();
    }
  }

  cloudToDepth(cloud_msg, depth_msg, color_msg);

  process(depth_msg, color_msg, crop_size_);
}

void DepthCloudEncoder::depthCB(const sensor_msgs::ImageConstPtr& depth_msg)
{
  process(depth_msg, sensor_msgs::ImageConstPtr(), crop_size_);
}

void DepthCloudEncoder::depthColorCB(const sensor_msgs::ImageConstPtr& depth_msg,
                                     const sensor_msgs::ImageConstPtr& color_msg)
{
  process(depth_msg, color_msg, crop_size_);
}

void DepthCloudEncoder::cloudToDepth(const sensor_msgs::PointCloud2& cloud_msg, sensor_msgs::ImagePtr depth_msg, sensor_msgs::ImagePtr color_msg)
{
  // projected coordinates + z depth value + Cx,y offset of image plane to origin :
  double u, v, zd, cx, cy;

  cx = depth_msg->width / 2;
  cy = depth_msg->height / 2;

  // we assume that all the coordinates are in meters...
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud( new pcl::PointCloud<pcl::PointXYZRGB> );
  pcl::fromROSMsg(cloud_msg,*scene_cloud);

  // first convert to camera frame.
  if ( cloud_msg.header.frame_id != this->camera_frame_id_ ) {

    tf::StampedTransform transform;
    try {
      ros::Duration timeout(0.05);
      // ros::Time(0) can cause unexpected frames?
      tf_listener_.waitForTransform( this->camera_frame_id_, cloud_msg.header.frame_id,
				    ros::Time(0), timeout);
      tf_listener_.lookupTransform(this->camera_frame_id_, cloud_msg.header.frame_id,
				   ros::Time(0), transform);
    }
    catch (tf::ExtrapolationException& ex) {
      ROS_WARN("[run_viewer] TF ExtrapolationException:\n%s", ex.what());
    }
    catch (tf::ConnectivityException& ex) {
      if (!connectivityExceptionFlag) {
	ROS_WARN("[run_viewer] TF ConnectivityException:\n%s", ex.what());
	ROS_INFO("[run_viewer] Pick-it-App might not run correctly");
	connectivityExceptionFlag = true;
      }
    }
    catch (tf::LookupException& ex) {
      if (!lookupExceptionFlag){
	ROS_WARN("[run_viewer] TF LookupException:\n%s", ex.what());
	ROS_INFO("[run_viewer] Pick-it-App might not be running yet");
	lookupExceptionFlag = true;
	return;
      }
    }
    catch (tf::TransformException& ex) {
      ROS_WARN("[run_viewer] TF exception:\n%s", ex.what());
      return;
    }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr camera_cloud( new pcl::PointCloud<pcl::PointXYZRGB> );
	// Transform to the Camera reference frame
	Eigen::Affine3d eigen_transform3d;
	tf::transformTFToEigen(transform, eigen_transform3d );
	Eigen::Affine3f eigen_transform3f( eigen_transform3d );
	pcl::transformPointCloud(*scene_cloud,*camera_cloud,eigen_transform3f);

	// pass on new point cloud expressed in camera frame:
	scene_cloud = camera_cloud;
  }

  int number_of_points =  scene_cloud->size();

  using namespace std;

  for(int i = 0 ; i < number_of_points ; i++)
    {
      if(isFinite(scene_cloud->points[i]))
	{
	  // calculate in 'image plane' :
	  u = (f_ / scene_cloud->points[i].z ) * scene_cloud->points[i].x + cx;
	  v = (f_ / scene_cloud->points[i].z ) * scene_cloud->points[i].y + cy;
	  // only write out pixels that fit into our depth image
	  int dlocation = int(u)*4 + int(v)*depth_msg->step;
	  if ( dlocation >=0 && dlocation < depth_msg->data.size() ) {
	    *(float*)&depth_msg->data[ dlocation ] = scene_cloud->points[i].z;
	  }
	  int clocation = int(u)*3 + int(v)*color_msg->step;
	  if ( clocation >=0 && clocation < color_msg->data.size() ) {
	    color_msg->data[ clocation   ] = scene_cloud->points[i].r;
	    color_msg->data[ clocation+1 ] = scene_cloud->points[i].g;
	    color_msg->data[ clocation+2 ] = scene_cloud->points[i].b;
	  }
	}
    }
}

void DepthCloudEncoder::process(const sensor_msgs::ImageConstPtr& depth_msg,
                                const sensor_msgs::ImageConstPtr& color_msg,
                                const std::size_t crop_size)
{
  // Bit depth of image encoding
  int depth_bits = enc::bitDepth(depth_msg->encoding);
  int depth_channels = enc::numChannels(depth_msg->encoding);

  int color_bits = 0;
  int color_channels = 0;

  if ((depth_bits != 32) || (depth_channels != 1))
  {
    ROS_DEBUG_STREAM( "Invalid color depth image format ("<<depth_msg->encoding <<")");
    return;
  }

  // OpenCV-ros bridge
  cv_bridge::CvImagePtr color_cv_ptr;

  // check for color message
  if (color_msg)
  {
    try
    {
      color_cv_ptr = cv_bridge::toCvCopy(color_msg, "bgr8");

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR_STREAM("OpenCV-ROS bridge error: " << e.what());
      return;
    }

    if (depth_msg->width != color_msg->width || depth_msg->height != color_msg->height)
    {
      ROS_DEBUG_STREAM( "Depth image resolution (" << (int)depth_msg->width << "x" << (int)depth_msg->height << ") "
      "does not match color image resolution (" << (int)color_msg->width << "x" << (int)color_msg->height << ")");
      return;
    }
  }

  // preprocessing => close NaN hole via interpolation and generate valid_point_mask
  sensor_msgs::ImagePtr depth_int_msg(new sensor_msgs::Image());
  sensor_msgs::ImagePtr valid_mask_msg(new sensor_msgs::Image());

  depthInterpolation(depth_msg, depth_int_msg, valid_mask_msg);

  unsigned int pix_size = enc::bitDepth(enc::BGR8) / 8 * 3;

  // generate output image message
  sensor_msgs::ImagePtr output_msg(new sensor_msgs::Image);
  output_msg->header = depth_int_msg->header;
  output_msg->encoding = enc::BGR8;
  output_msg->width = crop_size * 2;
  output_msg->height = crop_size * 2;
  output_msg->step = output_msg->width * pix_size;
  output_msg->is_bigendian = depth_int_msg->is_bigendian;

  // allocate memory
  output_msg->data.resize(output_msg->width * output_msg->height * pix_size, 0xFF);

  std::size_t input_width = depth_int_msg->width;
  std::size_t input_height = depth_int_msg->height;

  // copy depth & color data to output image
  {
    std::size_t y, x, left_x, top_y, width_x, width_y;

    // calculate borders to crop input image to crop_size X crop_size
    int top_bottom_corner = (static_cast<int>(input_height) - static_cast<int>(crop_size)) / 2;
    int left_right_corner = (static_cast<int>(input_width) - static_cast<int>(crop_size)) / 2;

    if (top_bottom_corner < 0)
    {
      top_y = 0;
      width_y = input_height;
    }
    else
    {
      top_y = top_bottom_corner;
      width_y = input_height - top_bottom_corner;
    }

    if (left_right_corner < 0)
    {
      left_x = 0;
      width_x = input_width;
    }
    else
    {
      left_x = left_right_corner;
      width_x = input_width - left_right_corner;
    }

    // pointer to output image
    uint8_t* dest_ptr = &output_msg->data[((top_y - top_bottom_corner) * output_msg->width + left_x - left_right_corner)
        * pix_size];
    const std::size_t dest_y_step = output_msg->step;

    // pointer to interpolated depth data
    const float* source_depth_ptr = (const float*)&depth_int_msg->data[(top_y * input_width + left_x) * sizeof(float)];

    // pointer to valid-pixel-mask
    const uint8_t* source_mask_ptr = &valid_mask_msg->data[(top_y * input_width + left_x) * sizeof(uint8_t)];

    // pointer to color data
    const uint8_t* source_color_ptr = 0;
    std::size_t source_color_y_step = 0;
    if (color_msg)
    {
      source_color_y_step = input_width * pix_size;
      source_color_ptr = static_cast<const uint8_t*>(&color_cv_ptr->image.data[(top_y * input_width + left_x) * pix_size]);
    }

    // helpers
    const std::size_t right_frame_shift = crop_size * pix_size;
    ;
    const std::size_t down_frame_shift = dest_y_step * crop_size;

    // generate output image
    for (y = top_y; y < width_y;
        ++y, source_color_ptr += source_color_y_step, source_depth_ptr += input_width, source_mask_ptr += input_width, dest_ptr +=
            dest_y_step)
    {
      const float *depth_ptr = source_depth_ptr;

      // reset iterator pointers for each column
      const uint8_t *color_ptr = source_color_ptr;
      const uint8_t *mask_ptr = source_mask_ptr;

      uint8_t *out_depth_low_ptr = dest_ptr;
      uint8_t *out_depth_high_ptr = dest_ptr + right_frame_shift;
      uint8_t *out_color_ptr = dest_ptr + right_frame_shift + down_frame_shift;

      for (x = left_x; x < width_x; ++x)
      {
        int depth_pix_low;
        int depth_pix_high;

        if (*depth_ptr == *depth_ptr) // valid point
        {
          depth_pix_low = std::min(std::max(0.0f, (*depth_ptr / max_depth_per_tile_) * (float)(0xFF * 3)), (float)(0xFF * 3));
          depth_pix_high = std::min(std::max(0.0f, ((*depth_ptr - max_depth_per_tile_) / max_depth_per_tile_) * (float)(0xFF) * 3), (float)(0xFF * 3));
        }
        else
        {
          depth_pix_low = 0;
          depth_pix_high = 0;

        }

        uint8_t *mask_pix_ptr = out_depth_low_ptr + down_frame_shift;
        if (*mask_ptr == 0)
        {
          // black pixel for valid point
          memset(mask_pix_ptr, 0x00, pix_size);
        }
        else
        {
          // white pixel for invalid point
          memset(mask_pix_ptr, 0xFF, pix_size);
        }

        // divide into color channels + saturate for each channel:
        uint8_t depth_pix_low_r = std::min(std::max(0, depth_pix_low), (0xFF));
        uint8_t depth_pix_low_g = std::min(std::max(0, depth_pix_low-(0xFF)), (0xFF));
        uint8_t depth_pix_low_b = std::min(std::max(0, depth_pix_low-(0xFF*2)), (0xFF));

        *out_depth_low_ptr = depth_pix_low_r;  ++out_depth_low_ptr;
        *out_depth_low_ptr = depth_pix_low_g;  ++out_depth_low_ptr;
        *out_depth_low_ptr = depth_pix_low_b;  ++out_depth_low_ptr;

        uint8_t depth_pix_high_r = std::min(std::max(0, depth_pix_high), (0xFF));
        uint8_t depth_pix_high_g = std::min(std::max(0, depth_pix_high-(0xFF)), (0xFF));
        uint8_t depth_pix_high_b = std::min(std::max(0, depth_pix_high-(0xFF*2)), (0xFF));

        *out_depth_high_ptr = depth_pix_high_r; ++out_depth_high_ptr;
        *out_depth_high_ptr = depth_pix_high_g; ++out_depth_high_ptr;
        *out_depth_high_ptr = depth_pix_high_b; ++out_depth_high_ptr;

        if (color_ptr)
        {
          // copy three color channels
          *out_color_ptr = *color_ptr; ++out_color_ptr; ++color_ptr;

          *out_color_ptr = *color_ptr; ++out_color_ptr; ++color_ptr;

          *out_color_ptr = *color_ptr; ++out_color_ptr; ++color_ptr;
        }

        // increase input iterator pointers
        ++mask_ptr;
        ++depth_ptr;

      }
    }
  }

  pub_.publish(output_msg);
}

void DepthCloudEncoder::depthInterpolation(sensor_msgs::ImageConstPtr depth_msg, sensor_msgs::ImagePtr depth_int_msg,
                                           sensor_msgs::ImagePtr mask_msg)
{
  const std::size_t input_width = depth_msg->width;
  const std::size_t input_height = depth_msg->height;

  // prepare output image - depth image with interpolated NaN holes
  depth_int_msg->header = depth_msg->header;
  depth_int_msg->encoding = depth_msg->encoding;
  depth_int_msg->width = input_width;
  depth_int_msg->height = input_height;
  depth_int_msg->step = depth_msg->step;
  depth_int_msg->is_bigendian = depth_msg->is_bigendian;
  depth_int_msg->data.resize(depth_int_msg->step * depth_int_msg->height, 0);

  // prepare output image - valid sample mask
  mask_msg->header = depth_msg->header;
  mask_msg->encoding = enc::TYPE_8UC1;
  mask_msg->width = input_width;
  mask_msg->height = input_height;
  mask_msg->step = depth_msg->step;
  mask_msg->is_bigendian = depth_msg->is_bigendian;
  mask_msg->data.resize(mask_msg->step * mask_msg->height, 0xFF);

  const float* depth_ptr = (const float*)&depth_msg->data[0];
  float* depth_int_ptr = (float*)&depth_int_msg->data[0];
  uint8_t* mask_ptr = (uint8_t*)&mask_msg->data[0];

  float leftVal = -1.0f;

  unsigned int y, len;
  for (y = 0; y < input_height; ++y, depth_ptr += input_width, depth_int_ptr += input_width, mask_ptr += input_width)
  {
    const float* in_depth_ptr = depth_ptr;
    float* out_depth_int_ptr = depth_int_ptr;
    uint8_t* out_mask_ptr = mask_ptr;

    const float* in_depth_end_ptr = depth_ptr + input_width;

    while (in_depth_ptr < in_depth_end_ptr)
    {
      len = 0;
      const float* last_valid_pix_ptr = in_depth_ptr;
      while ((isnan(*in_depth_ptr) || (*in_depth_ptr == 0)) && (in_depth_ptr < in_depth_end_ptr))
      {
        ++in_depth_ptr;
        ++len;
      }
      if (len > 0)
      {
        // we found a NaN hole

        // find valid pixel on right side of hole
        float rightVal;
        if (in_depth_ptr < in_depth_end_ptr)
        {
          rightVal = *in_depth_ptr;
        }
        else
        {
          rightVal = leftVal;
        }

        // find valid pixel on left side of hole
        if (isnan(leftVal) || (leftVal < 0.0f))
          leftVal = rightVal;

        float incVal = (rightVal - leftVal) / (float)len;
        float fillVal = leftVal;
        const float* fill_ptr;

        for (fill_ptr = last_valid_pix_ptr; fill_ptr < in_depth_ptr; ++fill_ptr)
        {
          // fill hole with interpolated pixel data
          *out_depth_int_ptr = fillVal;
          ++out_depth_int_ptr;

          // write unknown sample to valid-pixel-mask
          *out_mask_ptr = 0xFF; ++out_mask_ptr;

          fillVal += incVal;
        }

        leftVal = rightVal;
      }
      else
      {
        // valid sample found - no hole to patch

        leftVal = *in_depth_ptr;

        *out_depth_int_ptr = *in_depth_ptr;

        // write known sample to valid-pixel-mask
        *out_mask_ptr = 0; ++out_mask_ptr;

        ++in_depth_ptr;
        ++out_depth_int_ptr;
      }

    }

  }

}


}
