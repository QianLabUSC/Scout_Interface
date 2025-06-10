// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef IMAGE_PIPELINE__CAMERA_NODE_HPP_
#define IMAGE_PIPELINE__CAMERA_NODE_HPP_

#include <chrono>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <atomic>
#include <vector>
#include <memory>

using namespace std::chrono_literals;
#include "opencv2/highgui/highgui.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "common.hpp"

#include "webcam.hpp"
/// Node which captures images from a camera using OpenCV and publishes them.
/// Images are annotated with this process's id as well as the message's ptr.
class CameraNode final : public rclcpp::Node
{
public:
  /// \brief Construct a new CameraNode object for capturing video
  /// \param output The output topic name to use
  /// \param node_name The node name to use
  /// \param watermark Whether to add a watermark to the image before publishing
  /// \param device Which camera device to use
  /// \param width What video width to capture at
  /// \param height What video height to capture at
  explicit CameraNode(
    const std::string & output,
    const std::string & node_name = "camera_node",
    // bool watermark = true, 
    // int device = 0, int width = 320, int height = 240
    const std::vector<int> & serial_number = {5, 3, 7},
    int port = 7567
  )
  : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true)),
    canceled_(false)
  {
    // // Initialize OpenCV
    // cap_.open(device);
    // cap_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width));
    // cap_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height));
    //
    // if (!cap_.isOpened()) {
    //   throw std::runtime_error("Could not open video stream!");
    // }
    
    // Instead of directly opening a cv::VideoCapture device, create our gopro::GoProStream wrapper.
    stream_ = std::make_unique<gopro::GoProStream>(serial_number, port);

    // Create a publisher on the output topic.
    pub_ = this->create_publisher<sensor_msgs::msg::Image>(output, rclcpp::SensorDataQoS());
    timer_ = this->create_wall_timer(15ms, std::bind(&CameraNode::loop, this));

    tt_.tic();
    dupe_timer_.tic();
    // // Create the camera reading loop.
    // thread_ = std::thread(std::bind(&CameraNode::loop, this));
  }

  ~CameraNode()
  {
    // // Make sure to join the thread on shutdown.
    // canceled_.store(true);
    // if (thread_.joinable()) {
    //   thread_.join();
    // }
  }

  /// \brief Capture and publish data until the program is closed
  void loop()
  {
    // While running...
    count++;
    // while (rclcpp::ok() && !canceled_.load()) {
  // if(count%3 == 0){
  //   std::cout << tt_.toc();
  // }
      // tt_.tic();
      // // Capture a frame from OpenCV.
      // cap_ >> frame_;
      // if (frame_.empty()) {
      //   continue;
      // }
      
      // Use the webcam wrapper to capture a frame.
  // bool ret = stream_->image_capture(frame_);
      auto frame = stream_->get_frame();
      if (frame.empty()) {
        std::cout<<"No frame"<<std::endl;
        // continue;
        return;
      }
      cv::Mat gray,diff;
      cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
      if(frame_.empty()){
        //Do nothing
      }else{
        cv::absdiff(frame_gray_,gray,diff);
        float norm = cv::norm(diff);
        if (norm < 0.001) {
          if (!first_dupe_flag_){
            std::cout<<dupe_timer_.toc()<<std::endl;
            first_dupe_flag_=true;
          }
          std::cout<<"dupe:"<<norm<<std::endl;
          return;
        }
      }
      frame_ = std::move(frame);
      frame_gray_ = std::move(gray);
       
  // if(count%3 != 0){
  //   tt_.tic();
  //   return;
  // }
      // Create a new unique_ptr to an Image message for storage.
      sensor_msgs::msg::Image::UniquePtr msg(new sensor_msgs::msg::Image());

      // if (watermark_) {
      //   std::stringstream ss;
      //   // Put this process's id and the msg's pointer address on the image.
      //   ss << "pid: " << GETPID() << ", ptr: " << msg.get();
      //   draw_on_image(frame_, ss.str(), 20);
      // }
      std::cout<<"\t:\t"<<tt_.toc();
      // Pack the OpenCV image into the ROS image.
      set_now(msg->header.stamp);
      msg->header.frame_id = "camera_frame";
      msg->height = frame_.rows;
      msg->width = frame_.cols;
      msg->encoding = mat_type2encoding(frame_.type());
      msg->is_bigendian = false;
      msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame_.step);
      msg->data.assign(frame_.datastart, frame_.dataend);
      std::cout<<"\t:\t"<<msg->height<<":"<<msg->width;
      // Publish the message.
      pub_->publish(std::move(msg));  // Publish.

      std::cout<<"\t:\t"<<tt_.toc()<<std::endl;
      tt_.tic();
    // }
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  std::thread thread_;
  std::atomic<bool> canceled_;

  TicToc tt_;
  TicToc dupe_timer_;
  // /// whether or not to add a watermark to the image showing process id and
  // /// pointer location
  // bool watermark_;

  // cv::VideoCapture cap_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::Mat frame_;
  cv::Mat frame_gray_;
  uint32_t count = 0;
  bool first_dupe_flag_ = false;
  // Instead of a cv::VideoCapture, use the wrapper for the GoPro webcam.
  std::unique_ptr<gopro::GoProStream> stream_;
  
};

#endif  // IMAGE_PIPELINE__CAMERA_NODE_HPP_
