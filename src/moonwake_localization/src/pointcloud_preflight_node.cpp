#include "moonwake_localization/pointcloud_preflight_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace moonwake_localization
{

PointCloudPreflightNode::PointCloudPreflightNode(const rclcpp::NodeOptions &options) : rclcpp::Node("pointcloud_preflight_node", options)
{
    points_topic_ = this->declare_parameter<std::string>("points_topic", "/velodyne_points");
    expected_frame_id_ = this->declare_parameter<std::string>("expected_frame_id", "velodyne"); //默认坐标系为 velodyne
    expected_rate_hz_ = this->declare_parameter<double>("expected_rate_hz",10.0); //默认点云频率为10Hz
    minimum_rate_hz_ = this->declare_parameter<double>("minimum_rate_hz",9.0); //平均帧率阈值
    maximum_rate_hz_ = this->declare_parameter<double>("maximum_rate_hz",11.0); //平均帧率阈值
    maximum_frame_gap_sec_ = this->declare_parameter<double>("maximum_frame_gap_sec",0.2); //帧间隔阈值
    maximum_message_age_sec_ = this->declare_parameter<double>("maximum_message_age_sec",0.5); //数据延迟阈值
    const auto timing_window_size = this->declare_parameter<std::int64_t>("timing_window_size",100); //平均帧率计算窗口大小
    const auto report_interval_frames = this->declare_parameter<std::int64_t>("report_interval_frames",50); //报告间隔帧数

    if (points_topic_.empty()){
        throw std::invalid_argument("points_topic must not be empty!");
    }

    if (expected_frame_id_.empty()){
        throw std::invalid_argument("expected_frame_id must not be empty!");
    }

    if (!std::isfinite(expected_rate_hz_) || expected_rate_hz_ <= 0.0){
        throw std::invalid_argument("expected_rate_hz must be positive!");
    }

    if (!std::isfinite(minimum_rate_hz_) || !std::isfinite(maximum_rate_hz_) ||
        minimum_rate_hz_ <= 0.0 || maximum_rate_hz_ <= minimum_rate_hz_){
        throw std::invalid_argument("invalid point cloud rate limits!");
    }

    if (!std::isfinite(maximum_frame_gap_sec_) || !std::isfinite(maximum_message_age_sec_) ||
        maximum_frame_gap_sec_ <= 0.0 || maximum_message_age_sec_ <= 0.0){
        throw std::invalid_argument("timing thresholds must be positive!");
    }

    if (timing_window_size <= 0 || report_interval_frames <= 0){
        throw std::invalid_argument("timing window and report interval must be positive!");
    }
    
    timing_window_size_ = static_cast<std::size_t>(timing_window_size);
    report_interval_frames_ = static_cast<std::size_t>(report_interval_frames);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.reliable();
    qos.durability_volatile();
    pointcloud_subscription_ = this->create_subscription<PointCloud2>(
        points_topic_, qos, std::bind(&PointCloudPreflightNode::pointcloud_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(
        get_logger(), "Monitoring %s, expected frame=%s, expected rate=%.2f Hz",
        points_topic_.c_str(), expected_frame_id_.c_str(), expected_rate_hz_);
}

void PointCloudPreflightNode::pointcloud_callback(const PointCloud2::ConstSharedPtr message){
    ++received_frame_count_;

    bool frame_valid = true;

    const bool layout_valid = validate_layout(*message); //消息布局检查
    frame_valid = frame_valid && layout_valid;

    frame_valid = validate_frame_id(*message) && frame_valid;
    frame_valid = update_frame_timing(*message) && frame_valid;

    ScanInspection inspection;

    if (layout_valid){
        inspection = inspect_scan(*message);
        frame_valid = inspection.valid && frame_valid;
    }

    if (!frame_valid){
        ++invalid_frame_count_;
    }

    if (received_frame_count_ == 1 || received_frame_count_ % report_interval_frames_ == 0){
        report_status(*message, inspection);
    }
}

bool PointCloudPreflightNode::validate_layout(const PointCloud2 & message) const
{
    rclcpp::Clock throttle_clock(RCL_STEADY_TIME);
    bool valid = true;

    if (message.width == 0 || message.height == 0){
        RCLCPP_ERROR_THROTTLE(get_logger(), throttle_clock, 5000, "Point cloud has zero width or height!");
        valid = false;
    }

    if (message.point_step == 0 || message.row_step == 0){
        RCLCPP_ERROR_THROTTLE(get_logger(), throttle_clock, 5000, "Point cloud has an invalid point_step or row_step!");
        valid = false;
    }

    if (!valid){
        return false;
    }

    // 使用 64 位计算，避免 32 位宽度和步长相乘时溢出
    const auto minimum_row_bytes =
        static_cast<std::uint64_t>(message.width) * message.point_step;
    const auto required_bytes =
        static_cast<std::uint64_t>(message.row_step) * message.height;
    if (message.row_step < minimum_row_bytes || message.data.size() < required_bytes){
        RCLCPP_ERROR_THROTTLE(get_logger(), throttle_clock, 5000, "Point cloud data buffer is too small");
        return false;
    }

    const std::uint16_t byte_order_probe = 1;
    std::uint8_t first_byte = 0;
    std::memcpy(&first_byte, &byte_order_probe, sizeof(first_byte));
    const bool host_is_big_endian = first_byte == 0;
    if (message.is_bigendian != host_is_big_endian){
        RCLCPP_ERROR_THROTTLE(get_logger(), throttle_clock, 5000, "Point cloud byte order differs from host");
        return false;
    }

    valid = validate_field(message, "x", PointField::FLOAT32) && valid;
    valid = validate_field(message, "y", PointField::FLOAT32) && valid;
    valid = validate_field(message, "z", PointField::FLOAT32) && valid;
    valid = validate_field(message, "intensity", PointField::FLOAT32) && valid;
    valid = validate_field(message, "ring", PointField::UINT16) && valid;
    valid = validate_field(message, "time", PointField::FLOAT32) && valid;
    return valid;
}

bool PointCloudPreflightNode::validate_field(
    const PointCloud2 & message, const std::string & field_name,
    std::uint8_t expected_datatype) const
{
    rclcpp::Clock throttle_clock(RCL_STEADY_TIME);
    const auto * field = find_field(message, field_name);
    if (field == nullptr){
        RCLCPP_ERROR_THROTTLE(
            get_logger(), throttle_clock, 5000, "Missing point field: %s", field_name.c_str());
        return false;
    }
    const auto size = datatype_size(field->datatype);
    if (field->datatype != expected_datatype || field->count != 1 || size == 0 ||
        static_cast<std::uint64_t>(field->offset) + size > message.point_step)
    {
        RCLCPP_ERROR_THROTTLE(
            get_logger(), throttle_clock, 5000, "Invalid datatype, count or offset for field: %s",
            field_name.c_str());
        return false;
    }
    return true;
}

bool PointCloudPreflightNode::validate_frame_id(const PointCloud2 & message) const
{
    rclcpp::Clock throttle_clock(RCL_STEADY_TIME);
    if (message.header.frame_id != expected_frame_id_){
        RCLCPP_WARN_THROTTLE(
            get_logger(), throttle_clock, 5000, "Unexpected frame_id '%s'; expected '%s'",
            message.header.frame_id.c_str(), expected_frame_id_.c_str());
        return false;
    }
    return true;
}

bool PointCloudPreflightNode::update_frame_timing(const PointCloud2 & message)
{
    if (message.header.stamp.nanosec >= 1000000000U || message.header.stamp.sec < 0){
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Invalid frame timestamp");
        return false;
    }

    const rclcpp::Time stamp(message.header.stamp, get_clock()->get_clock_type());
    const double age_sec = (now() - stamp).seconds();
    bool timing_valid = true;
    if (age_sec > maximum_message_age_sec_ || age_sec < -maximum_message_age_sec_){
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000, "Point cloud timestamp differs from node clock by %.3f s",
            age_sec);
        timing_valid = false;
    }

    if (previous_frame_stamp_){
        const double period_sec = (stamp - *previous_frame_stamp_).seconds();
        if (period_sec <= 0.0){
            ++timestamp_rewind_count_;
            frame_periods_sec_.clear();
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Frame timestamp did not increase");
            timing_valid = false;
        } else {
            if (period_sec > maximum_frame_gap_sec_){
                ++large_gap_count_;
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 5000, "Large gap between frames: %.3f s", period_sec);
                timing_valid = false;
            }
            frame_periods_sec_.push_back(period_sec);
            if (frame_periods_sec_.size() > timing_window_size_){
                frame_periods_sec_.pop_front();
            }
            const double total_sec = std::accumulate(
                frame_periods_sec_.begin(), frame_periods_sec_.end(), 0.0);
            const double rate_hz = static_cast<double>(frame_periods_sec_.size()) / total_sec;
            if (rate_hz < minimum_rate_hz_ || rate_hz > maximum_rate_hz_){
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 5000, "Average point cloud rate is %.2f Hz", rate_hz);
                timing_valid = false;
            }
        }
    }
    previous_frame_stamp_ = stamp;
    return timing_valid;
}

PointCloudPreflightNode::ScanInspection PointCloudPreflightNode::inspect_scan(
    const PointCloud2 & message) const
{
    rclcpp::Clock throttle_clock(RCL_STEADY_TIME);
    ScanInspection result;
    const auto * time_field = find_field(message, "time");
    const auto * ring_field = find_field(message, "ring");
    if (time_field == nullptr || ring_field == nullptr){
        return result;
    }

    result.minimum_point_time = std::numeric_limits<float>::infinity();
    result.maximum_point_time = -std::numeric_limits<float>::infinity();
    result.minimum_ring = std::numeric_limits<std::uint16_t>::max();
    result.maximum_ring = std::numeric_limits<std::uint16_t>::min();
    std::optional<float> previous_point_time;

    for (std::size_t row = 0; row < message.height; ++row){
        for (std::size_t column = 0; column < message.width; ++column){
            const std::size_t offset = row * static_cast<std::size_t>(message.row_step) +
                column * static_cast<std::size_t>(message.point_step);
            float point_time = 0.0F;
            std::uint16_t ring = 0;
            // memcpy 在 Jetson 等平台也适用
            std::memcpy(&point_time, message.data.data() + offset + time_field->offset, sizeof(point_time));
            std::memcpy(&ring, message.data.data() + offset + ring_field->offset, sizeof(ring));
            ++result.point_count;

            result.minimum_ring = std::min(result.minimum_ring, ring);
            result.maximum_ring = std::max(result.maximum_ring, ring);
            if (ring > 15){
                ++result.invalid_ring_count;
            }
            if (!std::isfinite(point_time)){
                ++result.non_finite_time_count;
                continue;
            }
            result.minimum_point_time = std::min(result.minimum_point_time, point_time);
            result.maximum_point_time = std::max(result.maximum_point_time, point_time);
            if (previous_point_time && point_time < *previous_point_time){
                ++result.reversed_time_count;
            }
            previous_point_time = point_time;
        }
    }

    if (!std::isfinite(result.minimum_point_time)){
        result.minimum_point_time = 0.0F;
        result.maximum_point_time = 0.0F;
    }
    //时间错序报告
    result.valid = result.point_count > 0 && result.non_finite_time_count == 0 &&
        result.invalid_ring_count == 0;
    if (!result.valid || result.reversed_time_count > 0){
        RCLCPP_WARN_THROTTLE(
            get_logger(), throttle_clock, 5000,
            "Point checks: non-finite time=%zu, reversed time=%zu, invalid ring=%zu",
            result.non_finite_time_count, result.reversed_time_count, result.invalid_ring_count);
    }
    return result;
}

void PointCloudPreflightNode::report_status(
    const PointCloud2 & message, const ScanInspection & inspection) const
{
    if (frame_periods_sec_.empty()){
        RCLCPP_INFO(
            get_logger(), "Frames=%llu invalid=%llu frame_id=%s points=%zu (rate not available yet)",
            static_cast<unsigned long long>(received_frame_count_),
            static_cast<unsigned long long>(invalid_frame_count_),
            message.header.frame_id.c_str(), inspection.point_count);
    } else {
        const double total_sec = std::accumulate(
            frame_periods_sec_.begin(), frame_periods_sec_.end(), 0.0);
        const double rate_hz = static_cast<double>(frame_periods_sec_.size()) / total_sec;
        RCLCPP_INFO(
            get_logger(),
            "Frames=%llu invalid=%llu frame_id=%s points=%zu rate=%.2f Hz gaps=%llu rewinds=%llu",
            static_cast<unsigned long long>(received_frame_count_),
            static_cast<unsigned long long>(invalid_frame_count_), message.header.frame_id.c_str(),
            inspection.point_count, rate_hz, static_cast<unsigned long long>(large_gap_count_),
            static_cast<unsigned long long>(timestamp_rewind_count_));
    }
    if (inspection.point_count > 0){
        RCLCPP_INFO(
            get_logger(),
            "Point time=[%.6f, %.6f] s ring=[%u, %u] non-finite=%zu reversed=%zu invalid-ring=%zu",
            static_cast<double>(inspection.minimum_point_time),
            static_cast<double>(inspection.maximum_point_time),
            static_cast<unsigned int>(inspection.minimum_ring),
            static_cast<unsigned int>(inspection.maximum_ring), inspection.non_finite_time_count,
            inspection.reversed_time_count, inspection.invalid_ring_count);
    }
}

const PointCloudPreflightNode::PointField * PointCloudPreflightNode::find_field(
    const PointCloud2 & message, const std::string & field_name)
{
    for (const auto & field : message.fields){
        if (field.name == field_name){
            return &field;
        }
    }
    return nullptr;
}

std::size_t PointCloudPreflightNode::datatype_size(std::uint8_t datatype)
{
    switch (datatype){
        case PointField::INT8:
        case PointField::UINT8:
            return 1;
        case PointField::INT16:
        case PointField::UINT16:
            return 2;
        case PointField::INT32:
        case PointField::UINT32:
        case PointField::FLOAT32:
            return 4;
        case PointField::FLOAT64:
            return 8;
        default:
            return 0;
    }
}

}  // namespace moonwake_localization

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<moonwake_localization::PointCloudPreflightNode>());
    rclcpp::shutdown();
    return 0;
}
