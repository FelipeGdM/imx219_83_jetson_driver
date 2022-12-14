#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"
#include "camera_info_manager/camera_info_manager.hpp"

#include "rclcpp/logging.hpp"

inline std::string generate_pipeline(
  int16_t device_id, int16_t width, int16_t height,
  std::string & framerate, int16_t flip_method)
{

  std::ostringstream capture_args;

  capture_args << "nvarguscamerasrc sensor-id=" << device_id <<
    " ! video/x-raw(memory:NVMM), width=(int)" << width << ", height=(int)" << height <<
    ", framerate=(fraction)" << framerate << ", format=NV12 ! nvvidconv flip-method=" << flip_method <<
    " ! video/x-raw, width=(int)" << width << ", height=(int)" << height <<
    ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! queue ! appsink";

  return capture_args.str();
}

class Imx219_83_Publisher : public rclcpp::Node
{
public:
  Imx219_83_Publisher()
  : Node("Imx219_83_Publisher"), frame_count_(0)
  {

    this->declare_parameter<int16_t>("left_camera_id", 1);
    this->declare_parameter<int16_t>("right_camera_id", 0);
    this->declare_parameter<int16_t>("width", 1280);
    this->declare_parameter<int16_t>("height", 720);
    this->declare_parameter<std::string>("framerate", "60/1");
    this->declare_parameter<int16_t>("flip_method", 2);

    this->declare_parameter<std::string>("frame_id", "camera_frame");
    this->declare_parameter<std::string>("left_camera_name", "left_camera");
    this->declare_parameter<std::string>(
      "left_camera_info_url",
      "file:///${ROS_HOME}/camera_info/stereo/left.ini"
    );

    this->declare_parameter<std::string>("right_camera_name", "right_camera");
    this->declare_parameter<std::string>(
      "right_camera_info_url",
      "file:///${ROS_HOME}/camera_info/stereo/right.ini"
    );

    this->get_parameter("left_camera_id", this->left_camera_id_);
    this->get_parameter("right_camera_id", this->right_camera_id_);
    this->get_parameter("width", this->width_);
    this->get_parameter("height", this->height_);
    this->get_parameter("framerate", this->framerate_);
    this->get_parameter("flip_method", this->flip_method_);

    this->get_parameter("frame_id", this->frame_id_);
    this->get_parameter("left_camera_name", this->left_camera_name_);
    this->get_parameter("left_camera_info_url", this->left_camera_info_url_);
    this->get_parameter("right_camera_name", this->right_camera_name_);
    this->get_parameter("right_camera_info_url", this->right_camera_info_url_);

    this->left_camera_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
      this,
      this->left_camera_name_,
      this->left_camera_info_url_
    );

    this->right_camera_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
      this,
      this->right_camera_name_,
      this->right_camera_info_url_
    );

    this->left_camera_info_msg_ = std::make_shared<sensor_msgs::msg::CameraInfo>(
      this->left_camera_manager_->getCameraInfo());

    this->right_camera_info_msg_ = std::make_shared<sensor_msgs::msg::CameraInfo>(
      this->right_camera_manager_->getCameraInfo());

    rmw_qos_profile_t qos_profile = {
      .history = RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT,
      .depth = 5,
      // .reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
      .reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
      .durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
      .liveliness = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC,
    };

    this->left_publisher_ = std::make_shared<image_transport::CameraPublisher>(
      this,
      "left/image_raw",
      qos_profile
    );

    this->right_publisher_ = std::make_shared<image_transport::CameraPublisher>(
      this,
      "right/image_raw",
      qos_profile
    );

    RCLCPP_INFO_STREAM(
      this->get_logger(), "Pipeline : " << generate_pipeline(
        this->left_camera_id_, this->width_, this->height_, this->framerate_, this->flip_method_
        )
    );

    this->left_camera_ = std::make_unique<cv::VideoCapture>(
      generate_pipeline(
        this->left_camera_id_, this->width_, this->height_, this->framerate_, this->flip_method_
      ), cv::CAP_GSTREAMER
    );

    this->right_camera_ = std::make_unique<cv::VideoCapture>(
      generate_pipeline(
        this->right_camera_id_, this->width_, this->height_, this->framerate_, this->flip_method_
      ), cv::CAP_GSTREAMER
    );

    rclcpp::on_shutdown(std::bind(&Imx219_83_Publisher::shutdown, this));
    RCLCPP_INFO_STREAM(this->get_logger(), "Camera streams created");
  }

  void spin()
  {
    while (rclcpp::ok() && !this->request_shutdown_) {
      cv::Mat left_frame, right_frame;

      if (!this->left_camera_->grab() || !this->right_camera_->grab()) {
        RCLCPP_ERROR_THROTTLE(
          this->get_logger(),
          (*this->get_clock()), 5000, "Got empty frame from camera");
        continue;
      }

      auto left_call = std::async(
        std::launch::async,
        [this, &left_frame] {
          return this->right_camera_->retrieve(left_frame);
        });

      auto right_call = std::async(
        std::launch::async,
        [this, &right_frame] {
          return this->right_camera_->retrieve(right_frame);
        });

      if (!left_call.get() || !right_call.get()) {
        RCLCPP_ERROR_THROTTLE(
          this->get_logger(),
          (*this->get_clock()), 5000, "Error while retrieving images");
        continue;
      }

      RCLCPP_INFO_STREAM_THROTTLE(
        this->get_logger(),
        (*this->get_clock()), (60 * 1000), "Publishing image! " << frame_count_++);

      std_msgs::msg::Header header = std_msgs::msg::Header();
      header.frame_id = this->frame_id_;

      this->left_image_msg_ = cv_bridge::CvImage(header, "bgr8", left_frame).toImageMsg();
      this->right_image_msg_ = cv_bridge::CvImage(header, "bgr8", right_frame).toImageMsg();

      this->left_publisher_->publish(*this->left_image_msg_, *this->left_camera_info_msg_);
      this->right_publisher_->publish(*this->right_image_msg_, *this->right_camera_info_msg_);

      // rclcpp::spin_some(this->shared_from_this());
    }
  }

  void shutdown()
  {
    this->request_shutdown_ = 1;
    this->left_camera_->release();
    this->right_camera_->release();
  }

private:
  // Signal-safe flag for whether shutdown is requested
  sig_atomic_t volatile request_shutdown_ = 0;
  int16_t left_camera_id_;
  int16_t right_camera_id_;
  int16_t width_;
  int16_t height_;
  std::string framerate_;
  std::string frame_id_;
  std::string left_camera_info_url_;
  std::string right_camera_info_url_;
  int16_t flip_method_;

  std::string left_camera_name_;
  std::string right_camera_name_;

  std::shared_ptr<camera_info_manager::CameraInfoManager> left_camera_manager_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> right_camera_manager_;

  sensor_msgs::msg::Image::SharedPtr left_image_msg_;
  sensor_msgs::msg::Image::SharedPtr right_image_msg_;
  sensor_msgs::msg::CameraInfo::SharedPtr left_camera_info_msg_;
  sensor_msgs::msg::CameraInfo::SharedPtr right_camera_info_msg_;

  std::shared_ptr<cv::VideoCapture> left_camera_;
  std::shared_ptr<cv::VideoCapture> right_camera_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<image_transport::CameraPublisher> left_publisher_;
  std::shared_ptr<image_transport::CameraPublisher> right_publisher_;
  size_t frame_count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<Imx219_83_Publisher> node = std::make_shared<Imx219_83_Publisher>();

  node->spin();

  rclcpp::shutdown();
  return 0;
}
