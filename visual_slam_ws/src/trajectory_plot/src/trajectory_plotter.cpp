#include <chrono>
#include <functional>
#include <memory>
#include <fstream>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"


using namespace std::chrono_literals;

class FixedFrameBroadcaster : public rclcpp::Node
{
public:
  explicit FixedFrameBroadcaster()
  : Node("fixed_frame_tf2_broadcaster"), file_read_(false)
  {

    cameraname_ = this->declare_parameter<std::string>("camerapose", "camera");

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&FixedFrameBroadcaster::broadcast_timer_callback, this));

    trajectory_filename_ = "../share/trajectory_plot/resources/trajectory.txt";
    
    std::ifstream fin(trajectory_filename_);

    if (!fin)
    {
      // std::cout << "Cannot find trajectory file at " << trajectory_filename_ << std::endl;
      RCLCPP_WARN(this->get_logger(), "Cannot find trajectory file at %s", trajectory_filename_.c_str());
      rclcpp::shutdown();
    } 
    else
    {
      file_read_ = true;
    }

    while (!fin.eof())
    {
      double time, tx, ty, tz, qx, qy, qz, qw;
      fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
      Eigen::Isometry3d transformation(Eigen::Quaterniond(qw, qx, qy, qz));
      transformation.pretranslate(Eigen::Vector3d(tx, ty, tz));
      poses.push_back(transformation);
    }
    RCLCPP_INFO(this->get_logger(), "Total poses read is %d", poses.size());
    // initiate the current pose
    current_transform_ = Eigen::Isometry3d::Identity();
  }

private:

  void broadcast_timer_callback()
  {
    if ((!file_read_) || (poses_count_ >= poses.size()))
    {
      RCLCPP_WARN(this->get_logger(), "Shutting down the node");
      rclcpp::shutdown();
    }
    geometry_msgs::msg::TransformStamped t;
    
    current_transform_ = poses[poses_count_] * current_transform_;

    Eigen::Quaterniond q(current_transform_.rotation());
    Eigen::Vector3d p = current_transform_.translation();
    // count the poses
    poses_count_++;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "turtle1";
    t.child_frame_id = cameraname_.c_str();

    t.transform.translation.x = p[0];
    t.transform.translation.y = p[1];
    t.transform.translation.z = p[2];
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
  }

rclcpp::TimerBase::SharedPtr timer_;
std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
std::string cameraname_;
std::string trajectory_filename_;
std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;
bool file_read_;
Eigen::Isometry3d current_transform_;
size_t poses_count_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FixedFrameBroadcaster>());
  rclcpp::shutdown();
  return 0;
}