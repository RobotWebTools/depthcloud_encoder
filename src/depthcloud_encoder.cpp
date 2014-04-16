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

namespace depthcloud
{

using namespace message_filters::sync_policies;
namespace enc = sensor_msgs::image_encodings;

static int queue_size_ = 10;
static int target_resolution_ = 512;
static int max_depth_per_tile = 2.0;

DepthCloudEncoder::DepthCloudEncoder(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
    nh_(nh),
    pnh_(pnh),
    depth_sub_(),
    color_sub_(),
    pub_it_(nh_),
    crop_size_(target_resolution_)
{
  // ROS parameters
  ros::NodeHandle priv_nh_("~");

  // read depth map topic from param server
  priv_nh_.param<std::string>("depth", depthmap_topic_, "/camera/depth/image");

  // read rgb topic from param server
  priv_nh_.param<std::string>("rgb", rgb_image_topic_, "/camera/rgb/image_color");

  // Monitor whether anyone is subscribed to the output
  image_transport::SubscriberStatusCallback connect_cb = boost::bind(&DepthCloudEncoder::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to pub_point_cloud_
  boost::lock_guard<boost::mutex> lock(connect_mutex_);

  pub_ = pub_it_.advertise("depthcloud_encoded", 1, connect_cb, connect_cb);
}
DepthCloudEncoder::~DepthCloudEncoder()
{
}

void DepthCloudEncoder::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);

  if (pub_.getNumSubscribers())
  {
    subscribe(depthmap_topic_, rgb_image_topic_);
  }
  else
  {
    unsubscribe();
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
    sync_depth_color_.reset(new SynchronizerDepthColor(SyncPolicyDepthColor(10)));
    depth_sub_.reset(new image_transport::SubscriberFilter());
    color_sub_.reset(new image_transport::SubscriberFilter());
  }
  catch (ros::Exception& e)
  {
    ROS_ERROR("Error unsubscribing: %s", e.what());
  }
}

void DepthCloudEncoder::depthCB(const sensor_msgs::ImageConstPtr& depth_msg)
{
  process(depth_msg, sensor_msgs::ImageConstPtr(), crop_size_);
}

void DepthCloudEncoder::depthColorCB(const sensor_msgs::ImageConstPtr& depth_msg,
                                     const sensor_msgs::ImageConstPtr& color_msg)
{
  ROS_DEBUG("Image depth/color pair received");
  process(depth_msg, color_msg, crop_size_);
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
    int top_bottom_corner = (input_height - crop_size) / 2;
    int left_right_corner = (input_width - crop_size) / 2;

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
        uint16_t depth_pix_low;
        uint16_t depth_pix_high;

        if (*depth_ptr == *depth_ptr) // valid point
        {
          depth_pix_low = std::min(std::max(0.0f, (*depth_ptr / 2.0f) * (float)(0xFF * 3)), (float)(0xFF * 3));
          depth_pix_high = std::min(std::max(0.0f, ((*depth_ptr - max_depth_per_tile) / 2.0f) * (float)(0xFF) * 3), (float)(0xFF * 3));
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

        uint8_t depth_pix_low_r = depth_pix_low / 3;
        uint8_t depth_pix_low_g = depth_pix_low / 3;
        uint8_t depth_pix_low_b = depth_pix_low / 3;

        if (depth_pix_low % 3 == 1)  ++depth_pix_low_r;
        if (depth_pix_low % 3 == 2)  ++depth_pix_low_g;

        *out_depth_low_ptr = depth_pix_low_r;  ++out_depth_low_ptr;
        *out_depth_low_ptr = depth_pix_low_g;  ++out_depth_low_ptr;
        *out_depth_low_ptr = depth_pix_low_b;  ++out_depth_low_ptr;

        uint8_t depth_pix_high_r = depth_pix_high / 3;
        uint8_t depth_pix_high_g = depth_pix_high / 3;
        uint8_t depth_pix_high_b = depth_pix_high / 3;

        if ((depth_pix_high % 3) == 1)
          ++depth_pix_high_r;
        if ((depth_pix_high % 3) == 2)
          ++depth_pix_high_g;

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

