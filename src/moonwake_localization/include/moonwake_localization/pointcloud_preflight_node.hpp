#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace moonwake_localization
{

class PointCloudPreflightNode final : public rclcpp::Node //final 表示不打算继续派生这个类，明确它是一个完整节点
{
public:
  explicit PointCloudPreflightNode( //explicit 防止 NodeOptions 被隐式转换成节点对象
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using PointCloud2 = sensor_msgs::msg::PointCloud2; //缩短类型名称
  using PointField = sensor_msgs::msg::PointField;

  struct ScanInspection
  {
    bool valid{false}; //有效性

    std::size_t point_count{0};
    std::size_t non_finite_time_count{0}; //存在多少个 NaN 或无穷大的时间值
    std::size_t reversed_time_count{0}; //每点时间出现多少次倒退
    std::size_t invalid_ring_count{0}; //多少点的线号不在 VLP-16 的 [0, 15]

    float minimum_point_time{0.0F};
    float maximum_point_time{0.0F};

    std::uint16_t minimum_ring{0};
    std::uint16_t maximum_ring{0};
  };

  void pointcloud_callback(const PointCloud2::ConstSharedPtr message);

  bool validate_layout(const PointCloud2 & message) const; //消息结构检查

  bool validate_field(
    const PointCloud2 & message,
    const std::string & field_name,
    std::uint8_t expected_datatype) const;

  bool validate_frame_id(const PointCloud2 & message) const; //坐标系检查

  bool update_frame_timing(const PointCloud2 & message); //帧时间检查

  ScanInspection inspect_scan(const PointCloud2 & message) const; //每点数据检查

  void report_status( //报告检查结果
    const PointCloud2 & message,
    const ScanInspection & inspection) const;

  static const PointField * find_field(
    const PointCloud2 & message,
    const std::string & field_name);

  static std::size_t datatype_size(std::uint8_t datatype);

  rclcpp::Subscription<PointCloud2>::SharedPtr pointcloud_subscription_;

  std::string points_topic_;
  std::string expected_frame_id_;

  double expected_rate_hz_{10.0};
  double minimum_rate_hz_{9.0};
  double maximum_rate_hz_{11.0};
  double maximum_frame_gap_sec_{0.2};
  double maximum_message_age_sec_{0.5};

  std::size_t timing_window_size_{100};
  std::size_t report_interval_frames_{50};

  std::optional<rclcpp::Time> previous_frame_stamp_; //运行状态
  std::deque<double> frame_periods_sec_;

  std::uint64_t received_frame_count_{0};
  std::uint64_t invalid_frame_count_{0};
  std::uint64_t timestamp_rewind_count_{0};
  std::uint64_t large_gap_count_{0};
};

}  // namespace moonwake_localization
