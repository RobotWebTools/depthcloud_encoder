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

#ifndef DEPTHCLOUD_ENCODER_H
#define DEPTHCLOUD_ENCODER_H

#include <iostream>
#include <string>
#include <boost/thread.hpp>

#include <dynamic_reconfigure/server.h>
#include <depthcloud_encoder/paramsConfig.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <sensor_msgs/image_encodings.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>

#include "ros/ros.h"

namespace depthcloud
{

class DepthCloudEncoder
{
public:
  DepthCloudEncoder(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  virtual ~DepthCloudEncoder();

  void dynReconfCb(depthcloud_encoder::paramsConfig& config, uint32_t level);

protected:

  void connectCb();

  void subscribe(std::string& depth_topic, std::string& color_topic);
  void subscribeCloud(std::string& cloud_topic);
  void unsubscribe();

  void cameraInfoCb(const sensor_msgs::CameraInfoConstPtr& cam_info_msg);

  void cloudCB(const sensor_msgs::PointCloud2& cloud_msg);

  void depthCB(const sensor_msgs::ImageConstPtr& depth_msg);

  void depthColorCB(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::ImageConstPtr& color_msg);

  void depthInterpolation(sensor_msgs::ImageConstPtr depth_msg,
                          sensor_msgs::ImagePtr depth_int_msg,
                          sensor_msgs::ImagePtr mask_msg);

  void cloudToDepth(const sensor_msgs::PointCloud2& cloud_msg, sensor_msgs::ImagePtr depth_msg, sensor_msgs::ImagePtr color_msg);
  void process(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::ImageConstPtr& color_msg, const std::size_t crop_size);


  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicyDepthColor;
  typedef message_filters::Synchronizer<SyncPolicyDepthColor> SynchronizerDepthColor;

  ros::NodeHandle& nh_;
  ros::NodeHandle& pnh_;

  // ROS stuff
  boost::shared_ptr<image_transport::SubscriberFilter > depth_sub_;
  boost::shared_ptr<image_transport::SubscriberFilter > color_sub_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber camera_info_sub_;

  boost::shared_ptr<SynchronizerDepthColor> sync_depth_color_;

  boost::mutex connect_mutex_;

  image_transport::ImageTransport pub_it_;
  image_transport::Publisher pub_;

  int crop_size_;

  std::string depthmap_topic_;
  std::string rgb_image_topic_;
  std::string cloud_topic_;
  std::string camera_info_topic_;
  std::string camera_frame_id_;
  std::string depth_source_;

  tf::TransformListener tf_listener_;

  double f_;
  double f_mult_factor_;
  float max_depth_per_tile_;
  bool latch_;

  bool connectivityExceptionFlag, lookupExceptionFlag;
};

}

#endif
