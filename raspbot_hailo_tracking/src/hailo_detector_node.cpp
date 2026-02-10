#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <hailo/vdevice.hpp>
#include <hailo/infer_model.hpp>
#include <hailo/buffer.hpp>
#include <hailo/expected.hpp>

extern "C" {
#include <hailo/hailort.h>
}

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

namespace {

std::string json_escape(const std::string &s)
{
    std::string out;
    out.reserve(s.size() + 8);
    for (const char c : s) {
        switch (c) {
        case '\\': out += "\\\\"; break;
        case '"': out += "\\\""; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default:
            if (static_cast<unsigned char>(c) < 0x20) {
                // Control characters -> skip
                out += ' ';
            } else {
                out += c;
            }
            break;
        }
    }
    return out;
}

float clampf(float v, float lo, float hi)
{
    return std::max(lo, std::min(hi, v));
}

int norm_sign(int v)
{
    return (v < 0) ? -1 : 1;
}

} // namespace

class HailoDetectorNode final : public rclcpp::Node
{
public:
    HailoDetectorNode() : rclcpp::Node("hailo_detector")
    {
        declare_parameter<std::string>("hef_path", "");
        declare_parameter<std::string>("model_name", "");
        declare_parameter<std::string>("labels_path", "");

        declare_parameter<std::string>("input_topic", "image_raw/compressed");
        declare_parameter<std::string>("detections_topic", "detections/json");
        declare_parameter<std::string>("tracking_enable_topic", "tracking/enable");
        declare_parameter<std::string>("tracking_config_topic", "tracking/config");
        declare_parameter<std::string>("gimbal_topic", "camera_gimbal/command_deg");

        declare_parameter<double>("inference_fps", 10.0);
        declare_parameter<double>("score_threshold", 0.45);
        declare_parameter<double>("iou_threshold", 0.45);
        declare_parameter<int>("max_proposals_total", 40);

        // Tracking
        declare_parameter<bool>("tracking_enabled", false);
        declare_parameter<int>("track_class_id", -1);
        declare_parameter<std::string>("track_label", "face");  // auto-pick: prefer face, fallback to person
        declare_parameter<double>("kp_pan_deg", 28.0);
        declare_parameter<double>("kp_tilt_deg", 20.0);
        declare_parameter<double>("max_step_deg", 6.0);
        declare_parameter<double>("deadband_norm", 0.05);
        declare_parameter<int>("pan_sign", 1);
        declare_parameter<int>("tilt_sign", 1);

        declare_parameter<double>("pan_min_deg", 0.0);
        declare_parameter<double>("pan_max_deg", 180.0);
        declare_parameter<double>("tilt_min_deg", 0.0);
        declare_parameter<double>("tilt_max_deg", 110.0);
        declare_parameter<double>("pan_neutral_deg", 90.0);
        declare_parameter<double>("tilt_neutral_deg", 45.0);

        // Auto-follow (robot base movement to follow a subject)
        declare_parameter<bool>("follow_enabled", false);
        declare_parameter<std::string>("follow_cmd_vel_topic", "cmd_vel");
        declare_parameter<std::string>("follow_enable_topic", "follow/enable");
        declare_parameter<std::string>("follow_target_area_topic", "follow/target_area");
        declare_parameter<std::string>("follow_max_linear_topic", "follow/max_linear");
        declare_parameter<double>("follow_target_bbox_area", 0.04);
        declare_parameter<double>("follow_kp_linear", 0.8);
        declare_parameter<double>("follow_ki_linear", 0.05);
        declare_parameter<double>("follow_kd_linear", 0.1);
        declare_parameter<double>("follow_kp_angular", 1.2);
        declare_parameter<double>("follow_ki_angular", 0.05);
        declare_parameter<double>("follow_kd_angular", 0.1);
        declare_parameter<double>("follow_max_linear", 0.3);
        declare_parameter<double>("follow_max_angular", 0.8);
        declare_parameter<double>("follow_linear_deadband", 0.02);
        declare_parameter<double>("follow_angular_deadband", 0.05);
        declare_parameter<double>("follow_lost_timeout_sec", 1.0);
        declare_parameter<bool>("follow_use_gimbal_feedback", true);
        declare_parameter<double>("follow_scan_speed_deg", 30.0);
        declare_parameter<double>("follow_scan_pause_sec", 0.4);
        declare_parameter<int>("follow_scan_max_sweeps", 3);
        declare_parameter<double>("follow_gimbal_recenter_kp", 0.10);
        declare_parameter<double>("follow_angular_blend", 0.5);

        // Ultrasonic obstacle avoidance
        declare_parameter<std::string>("follow_ultrasonic_topic", "ultrasonic/range");
        declare_parameter<double>("follow_obstacle_stop_m", 0.20);
        declare_parameter<double>("follow_obstacle_slow_m", 0.50);

        // Depth estimation (optional second model on same VDevice)
        declare_parameter<std::string>("depth_hef_path", "");
        declare_parameter<std::string>("depth_model_name", "");
        declare_parameter<std::string>("depth_input_topic", "front_camera/compressed");
        declare_parameter<std::string>("depth_image_topic", "depth/image");
        declare_parameter<std::string>("depth_colorized_topic", "depth/colorized/compressed");
        declare_parameter<std::string>("depth_enable_topic", "depth/enable");
        declare_parameter<double>("depth_inference_fps", 5.0);
        declare_parameter<int>("depth_colormap", 20);       // COLORMAP_TURBO
        declare_parameter<int>("depth_jpeg_quality", 75);
        declare_parameter<double>("depth_max_depth_m", 10.0);

        const auto input_topic = get_parameter("input_topic").as_string();
        const auto detections_topic = get_parameter("detections_topic").as_string();
        const auto tracking_enable_topic = get_parameter("tracking_enable_topic").as_string();
        const auto tracking_config_topic = get_parameter("tracking_config_topic").as_string();
        const auto gimbal_topic = get_parameter("gimbal_topic").as_string();
        const auto follow_cmd_vel_topic = get_parameter("follow_cmd_vel_topic").as_string();
        const auto follow_enable_topic = get_parameter("follow_enable_topic").as_string();
        const auto follow_target_area_topic = get_parameter("follow_target_area_topic").as_string();
        const auto follow_max_linear_topic = get_parameter("follow_max_linear_topic").as_string();

        detections_pub_ = create_publisher<std_msgs::msg::String>(detections_topic, 10);
        gimbal_pub_ = create_publisher<geometry_msgs::msg::Vector3>(gimbal_topic, 10);
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(follow_cmd_vel_topic, 10);

        tracking_enabled_.store(get_parameter("tracking_enabled").as_bool());
        follow_enabled_.store(get_parameter("follow_enabled").as_bool());
        follow_target_bbox_area_.store(get_parameter("follow_target_bbox_area").as_double());
        follow_max_linear_.store(get_parameter("follow_max_linear").as_double());
        pan_sign_live_.store(norm_sign(get_parameter("pan_sign").as_int()));
        tilt_sign_live_.store(norm_sign(get_parameter("tilt_sign").as_int()));

        tracking_sub_ = create_subscription<std_msgs::msg::Bool>(
            tracking_enable_topic, 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                tracking_enabled_.store(static_cast<bool>(msg->data));
            });

        tracking_cfg_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
            tracking_config_topic, 10,
            [this](const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
                if (!msg) {
                    return;
                }
                if (msg->data.size() >= 1) {
                    pan_sign_live_.store(norm_sign(static_cast<int>(msg->data[0])));
                }
                if (msg->data.size() >= 2) {
                    tilt_sign_live_.store(norm_sign(static_cast<int>(msg->data[1])));
                }
            });

        follow_enable_sub_ = create_subscription<std_msgs::msg::Bool>(
            follow_enable_topic, 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                const bool was_enabled = follow_enabled_.exchange(static_cast<bool>(msg->data));
                if (was_enabled && !msg->data) {
                    publish_stop_cmd_vel();
                    reset_follow_pid();
                }
                RCLCPP_INFO(get_logger(), "Follow %s", msg->data ? "ENABLED" : "DISABLED");
            });

        follow_target_area_sub_ = create_subscription<std_msgs::msg::Float64>(
            follow_target_area_topic, 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                const double v = std::max(0.01, std::min(1.0, msg->data));
                follow_target_bbox_area_.store(v);
                RCLCPP_INFO(get_logger(), "Follow target bbox area set to %.3f", v);
            });

        follow_max_linear_sub_ = create_subscription<std_msgs::msg::Float64>(
            follow_max_linear_topic, 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                const double v = std::max(0.05, std::min(1.0, msg->data));
                follow_max_linear_.store(v);
                RCLCPP_INFO(get_logger(), "Follow max linear set to %.3f m/s", v);
            });

        // Ultrasonic obstacle avoidance
        const auto ultrasonic_topic = get_parameter("follow_ultrasonic_topic").as_string();
        ultrasonic_sub_ = create_subscription<sensor_msgs::msg::Range>(
            ultrasonic_topic, 10,
            [this](const sensor_msgs::msg::Range::SharedPtr msg) {
                if (msg && !std::isnan(msg->range) && !std::isinf(msg->range)) {
                    ultrasonic_range_m_.store(msg->range);
                }
            });
        RCLCPP_INFO(get_logger(), "Ultrasonic topic: %s", ultrasonic_topic.c_str());

        // Safety timer – sends stop when subject is lost for > timeout
        follow_safety_timer_ = create_wall_timer(100ms,
            std::bind(&HailoDetectorNode::follow_safety_check, this));

        img_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
            input_topic, rclcpp::SensorDataQoS(),
            std::bind(&HailoDetectorNode::on_image, this, std::placeholders::_1));

        // Depth estimation setup
        depth_enabled_.store(true);
        {
            const auto depth_input_topic = get_parameter("depth_input_topic").as_string();
            const auto depth_image_topic = get_parameter("depth_image_topic").as_string();
            const auto depth_colorized_topic = get_parameter("depth_colorized_topic").as_string();
            const auto depth_enable_topic = get_parameter("depth_enable_topic").as_string();

            depth_pub_ = create_publisher<sensor_msgs::msg::Image>(depth_image_topic, 10);
            depth_colorized_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(depth_colorized_topic, 10);

            depth_img_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
                depth_input_topic, rclcpp::SensorDataQoS(),
                std::bind(&HailoDetectorNode::on_depth_image, this, std::placeholders::_1));

            depth_enable_sub_ = create_subscription<std_msgs::msg::Bool>(
                depth_enable_topic, 10,
                [this](const std_msgs::msg::Bool::SharedPtr msg) {
                    depth_enabled_.store(msg->data);
                    RCLCPP_INFO(get_logger(), "Depth estimation %s", msg->data ? "ENABLED" : "DISABLED");
                });

            RCLCPP_INFO(get_logger(), "depth input topic: %s", depth_input_topic.c_str());
            RCLCPP_INFO(get_logger(), "depth image topic: %s", depth_image_topic.c_str());
            RCLCPP_INFO(get_logger(), "depth colorized topic: %s", depth_colorized_topic.c_str());
        }

        load_timer_ = create_wall_timer(1s, std::bind(&HailoDetectorNode::try_load_model, this));

        last_infer_time_ = steady_clock::time_point(steady_clock::duration::zero());

        // Load labels (optional)
        load_labels();

        pan_deg_ = get_parameter("pan_neutral_deg").as_double();
        tilt_deg_ = get_parameter("tilt_neutral_deg").as_double();

        RCLCPP_INFO(get_logger(), "hailo_detector subscribing to %s", input_topic.c_str());
        RCLCPP_INFO(get_logger(), "detections topic: %s", detections_topic.c_str());
        RCLCPP_INFO(get_logger(), "tracking enable topic: %s", tracking_enable_topic.c_str());
        RCLCPP_INFO(get_logger(), "tracking config topic: %s", tracking_config_topic.c_str());
        RCLCPP_INFO(get_logger(), "gimbal topic: %s", gimbal_topic.c_str());
        RCLCPP_INFO(get_logger(), "follow cmd_vel topic: %s", follow_cmd_vel_topic.c_str());
        RCLCPP_INFO(get_logger(), "follow enable topic: %s", follow_enable_topic.c_str());
    }

private:
    using steady_clock = std::chrono::steady_clock;

    struct Det {
        int class_id;
        float score;
        float x1;
        float y1;
        float x2;
        float y2;
    };

    void load_labels()
    {
        const auto path = get_parameter("labels_path").as_string();
        if (path.empty()) {
            return;
        }
        std::ifstream f(path);
        if (!f.good()) {
            RCLCPP_WARN(get_logger(), "labels_path not readable: %s", path.c_str());
            return;
        }
        std::string line;
        labels_.clear();
        while (std::getline(f, line)) {
            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }
            labels_.push_back(line);
        }

        // Auto-pick track_class_id if user didn't set one.
        // Prefer the label specified by track_label (default "face"),
        // falling back to "person" if face is not found.
        if (get_parameter("track_class_id").as_int() < 0) {
            const auto preferred = get_parameter("track_label").as_string();
            int fallback_id = -1;
            for (size_t i = 0; i < labels_.size(); i++) {
                if (labels_[i] == preferred) {
                    track_class_id_cached_ = static_cast<int>(i);
                    RCLCPP_INFO(get_logger(), "Auto-detected '%s' at class_id=%zu",
                        preferred.c_str(), i);
                    break;
                }
                if (labels_[i] == "person" && fallback_id < 0) {
                    fallback_id = static_cast<int>(i);
                }
            }
            if (track_class_id_cached_ < 0 && fallback_id >= 0) {
                track_class_id_cached_ = fallback_id;
                RCLCPP_INFO(get_logger(), "'%s' not found in labels, falling back to 'person' at class_id=%d",
                    preferred.c_str(), fallback_id);
            }
        } else {
            track_class_id_cached_ = static_cast<int>(get_parameter("track_class_id").as_int());
        }
    }

    std::string label_for(int class_id) const
    {
        if (class_id >= 0 && static_cast<size_t>(class_id) < labels_.size()) {
            return labels_[static_cast<size_t>(class_id)];
        }
        return "id:" + std::to_string(class_id);
    }

    void try_load_model()
    {
        if (configured_) {
            return;
        }

        const auto hef_path = get_parameter("hef_path").as_string();
        if (hef_path.empty()) {
            static int warn_count = 0;
            if (warn_count < 3) {
                RCLCPP_WARN(get_logger(), "hef_path is empty; set -p hef_path:=/path/to/model.hef");
                warn_count++;
            }
            return;
        }

        const auto model_name = get_parameter("model_name").as_string();

        auto vdevice_exp = hailort::VDevice::create_shared();
        if (!vdevice_exp) {
            RCLCPP_ERROR(get_logger(), "Failed to create VDevice (status=%d)", static_cast<int>(vdevice_exp.status()));
            return;
        }
        vdevice_ = vdevice_exp.release();

        auto infer_model_exp = vdevice_->create_infer_model(hef_path, model_name);
        if (!infer_model_exp) {
            RCLCPP_ERROR(get_logger(), "Failed to create InferModel from HEF '%s' (status=%d)", hef_path.c_str(), static_cast<int>(infer_model_exp.status()));
            vdevice_.reset();
            return;
        }
        infer_model_ = infer_model_exp.release();

        // Configure input/output formats.
        auto in_exp = infer_model_->input();
        if (!in_exp) {
            RCLCPP_ERROR(get_logger(), "InferModel::input failed (status=%d)", static_cast<int>(in_exp.status()));
            reset_model();
            return;
        }
        auto input_stream = in_exp.release();
        input_stream.set_format_type(HAILO_FORMAT_TYPE_UINT8);
        input_stream.set_format_order(HAILO_FORMAT_ORDER_NHWC);

        auto out_exp = infer_model_->output();
        if (!out_exp) {
            RCLCPP_ERROR(get_logger(), "InferModel::output failed (status=%d)", static_cast<int>(out_exp.status()));
            reset_model();
            return;
        }
        auto output_stream = out_exp.release();

        if (!output_stream.is_nms()) {
            RCLCPP_ERROR(get_logger(), "Model output is not NMS. This node currently expects an NMS output (HAILO_NMS_BY_SCORE).");
            reset_model();
            return;
        }

        output_stream.set_format_type(HAILO_FORMAT_TYPE_FLOAT32);
        output_stream.set_format_order(HAILO_FORMAT_ORDER_HAILO_NMS_BY_SCORE);
        output_stream.set_nms_score_threshold(static_cast<float>(get_parameter("score_threshold").as_double()));
        output_stream.set_nms_iou_threshold(static_cast<float>(get_parameter("iou_threshold").as_double()));
        output_stream.set_nms_max_proposals_total(static_cast<uint32_t>(get_parameter("max_proposals_total").as_int()));

        auto configured_exp = infer_model_->configure();
        if (!configured_exp) {
            RCLCPP_ERROR(get_logger(), "InferModel::configure failed (status=%d)", static_cast<int>(configured_exp.status()));
            reset_model();
            return;
        }
        configured_ = std::make_unique<hailort::ConfiguredInferModel>(configured_exp.release());

        const auto shape = input_stream.shape();
        input_h_ = static_cast<int>(shape.height);
        input_w_ = static_cast<int>(shape.width);

        input_frame_size_ = input_stream.get_frame_size();
        output_frame_size_ = output_stream.get_frame_size();

        auto in_buf_exp = hailort::Buffer::create(input_frame_size_);
        auto out_buf_exp = hailort::Buffer::create(output_frame_size_);
        if (!in_buf_exp || !out_buf_exp) {
            RCLCPP_ERROR(get_logger(), "Failed to allocate IO buffers");
            reset_model();
            return;
        }
        input_buffer_ = std::make_unique<hailort::Buffer>(in_buf_exp.release());
        output_buffer_ = std::make_unique<hailort::Buffer>(out_buf_exp.release());

        input_name_ = input_stream.name();
        output_name_ = output_stream.name();

        RCLCPP_INFO(get_logger(), "Loaded HEF '%s' (input=%dx%d bytes=%zu, output bytes=%zu)",
            hef_path.c_str(), input_w_, input_h_, input_frame_size_, output_frame_size_);

        // Load depth model on the same VDevice (optional)
        try_load_depth_model();
    }

    void try_load_depth_model()
    {
        if (depth_configured_) {
            return;
        }
        if (!vdevice_) {
            return; // VDevice not ready yet
        }

        const auto depth_hef_path = get_parameter("depth_hef_path").as_string();
        if (depth_hef_path.empty()) {
            return; // Depth not configured
        }

        const auto depth_model_name = get_parameter("depth_model_name").as_string();

        auto infer_exp = vdevice_->create_infer_model(depth_hef_path, depth_model_name);
        if (!infer_exp) {
            RCLCPP_ERROR(get_logger(), "Failed to create depth InferModel from '%s' (status=%d)",
                         depth_hef_path.c_str(), static_cast<int>(infer_exp.status()));
            return;
        }
        depth_infer_model_ = infer_exp.release();

        auto in_exp = depth_infer_model_->input();
        if (!in_exp) {
            RCLCPP_ERROR(get_logger(), "Depth InferModel::input failed (status=%d)",
                         static_cast<int>(in_exp.status()));
            depth_infer_model_.reset();
            return;
        }
        auto depth_input = in_exp.release();
        depth_input.set_format_type(HAILO_FORMAT_TYPE_UINT8);
        depth_input.set_format_order(HAILO_FORMAT_ORDER_NHWC);

        auto out_exp = depth_infer_model_->output();
        if (!out_exp) {
            RCLCPP_ERROR(get_logger(), "Depth InferModel::output failed (status=%d)",
                         static_cast<int>(out_exp.status()));
            depth_infer_model_.reset();
            return;
        }
        auto depth_output = out_exp.release();
        depth_output.set_format_type(HAILO_FORMAT_TYPE_FLOAT32);

        auto configured_exp = depth_infer_model_->configure();
        if (!configured_exp) {
            RCLCPP_ERROR(get_logger(), "Depth InferModel::configure failed (status=%d)",
                         static_cast<int>(configured_exp.status()));
            depth_infer_model_.reset();
            return;
        }
        depth_configured_ = std::make_unique<hailort::ConfiguredInferModel>(configured_exp.release());

        const auto shape = depth_input.shape();
        depth_input_h_ = static_cast<int>(shape.height);
        depth_input_w_ = static_cast<int>(shape.width);
        depth_input_frame_size_ = depth_input.get_frame_size();
        depth_output_frame_size_ = depth_output.get_frame_size();

        auto in_buf = hailort::Buffer::create(depth_input_frame_size_);
        auto out_buf = hailort::Buffer::create(depth_output_frame_size_);
        if (!in_buf || !out_buf) {
            RCLCPP_ERROR(get_logger(), "Failed to allocate depth IO buffers");
            depth_configured_.reset();
            depth_infer_model_.reset();
            return;
        }
        depth_input_buffer_ = std::make_unique<hailort::Buffer>(in_buf.release());
        depth_output_buffer_ = std::make_unique<hailort::Buffer>(out_buf.release());

        depth_input_name_ = depth_input.name();
        depth_output_name_ = depth_output.name();

        RCLCPP_INFO(get_logger(),
                     "Loaded depth HEF '%s' (input=%dx%d bytes=%zu, output bytes=%zu)",
                     depth_hef_path.c_str(), depth_input_w_, depth_input_h_,
                     depth_input_frame_size_, depth_output_frame_size_);
    }

    void reset_model()
    {
        // Reset depth first (depends on vdevice)
        depth_configured_.reset();
        depth_infer_model_.reset();
        depth_input_buffer_.reset();
        depth_output_buffer_.reset();
        depth_input_name_.clear();
        depth_output_name_.clear();
        depth_input_w_ = 0;
        depth_input_h_ = 0;
        depth_input_frame_size_ = 0;
        depth_output_frame_size_ = 0;

        configured_.reset();
        infer_model_.reset();
        vdevice_.reset();
        input_buffer_.reset();
        output_buffer_.reset();
        input_name_.clear();
        output_name_.clear();
        input_w_ = 0;
        input_h_ = 0;
        input_frame_size_ = 0;
        output_frame_size_ = 0;
    }

    void on_image(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        if (!configured_) {
            return;
        }

        const double fps = std::max(1.0, get_parameter("inference_fps").as_double());
        const auto min_dt = std::chrono::duration_cast<steady_clock::duration>(
            std::chrono::duration<double>(1.0 / fps));
        const auto now = steady_clock::now();
        if ((now - last_infer_time_) < min_dt) {
            return;
        }
        last_infer_time_ = now;

        // Decode JPEG
        if (msg->data.empty()) {
            return;
        }
        cv::Mat raw(1, static_cast<int>(msg->data.size()), CV_8UC1, const_cast<unsigned char*>(msg->data.data()));
        cv::Mat bgr = cv::imdecode(raw, cv::IMREAD_COLOR);
        if (bgr.empty()) {
            return;
        }

        const int orig_w = bgr.cols;
        const int orig_h = bgr.rows;

        cv::Mat resized;
        cv::resize(bgr, resized, cv::Size(input_w_, input_h_), 0, 0, cv::INTER_LINEAR);
        cv::Mat rgb;
        cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
        if (!rgb.isContinuous()) {
            rgb = rgb.clone();
        }

        if (static_cast<size_t>(rgb.total() * rgb.elemSize()) < input_frame_size_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "RGB buffer too small for input frame size");
            return;
        }

        std::memcpy(input_buffer_->data(), rgb.data, input_frame_size_);

        std::map<std::string, hailort::MemoryView> buffers;
        buffers.emplace(input_name_, hailort::MemoryView(*input_buffer_));
        buffers.emplace(output_name_, hailort::MemoryView(*output_buffer_));

        auto bindings_exp = configured_->create_bindings(buffers);
        if (!bindings_exp) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "create_bindings failed (status=%d)", static_cast<int>(bindings_exp.status()));
            return;
        }
        auto bindings = bindings_exp.release();

        const auto st = configured_->run(bindings, 50ms);
        if (HAILO_SUCCESS != st) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "hailo run failed status=%d", static_cast<int>(st));
            return;
        }

        // Parse NMS_BY_SCORE output
        const uint8_t *out = output_buffer_->data();
        if (output_frame_size_ < sizeof(uint16_t)) {
            return;
        }
        const auto det_count = *reinterpret_cast<const uint16_t*>(out);
        const size_t header_size = sizeof(uint16_t);
        const size_t det_size = sizeof(hailo_detection_t);
        const size_t max_possible = (output_frame_size_ - header_size) / det_size;
        const size_t n = std::min<size_t>(static_cast<size_t>(det_count), max_possible);

        std::vector<Det> dets;
        dets.reserve(n);

        const uint8_t *p = out + header_size;
        for (size_t i = 0; i < n; i++) {
            const auto &d = *reinterpret_cast<const hailo_detection_t*>(p);
            p += det_size;

            const float x1 = clampf(d.x_min, 0.0f, 1.0f);
            const float y1 = clampf(d.y_min, 0.0f, 1.0f);
            const float x2 = clampf(d.x_max, 0.0f, 1.0f);
            const float y2 = clampf(d.y_max, 0.0f, 1.0f);

            dets.push_back(Det{static_cast<int>(d.class_id), d.score, x1, y1, x2, y2});
        }

        publish_detections_json(msg->header.stamp.sec, msg->header.stamp.nanosec, orig_w, orig_h, dets);

        if (tracking_enabled_.load()) {
            maybe_track_person(orig_w, orig_h, dets);
        }

        if (follow_enabled_.load()) {
            follow_subject(dets);
        }
    }

    void publish_detections_json(int32_t stamp_sec, uint32_t stamp_nanosec, int image_w, int image_h, const std::vector<Det> &dets)
    {
        std::ostringstream ss;
        ss.setf(std::ios::fixed);
        ss.precision(3);

          ss << "{\"image_width\":" << image_w
              << ",\"image_height\":" << image_h
              << ",\"stamp\":{\"sec\":" << stamp_sec << ",\"nanosec\":" << stamp_nanosec << "}"
           << ",\"detections\":[";

        bool first = true;
        for (const auto &d : dets) {
            if (std::isnan(d.score)) {
                continue;
            }
            if (d.score < static_cast<float>(get_parameter("score_threshold").as_double())) {
                continue;
            }

            const int x = static_cast<int>(std::round(d.x1 * image_w));
            const int y = static_cast<int>(std::round(d.y1 * image_h));
            const int w = static_cast<int>(std::round((d.x2 - d.x1) * image_w));
            const int h = static_cast<int>(std::round((d.y2 - d.y1) * image_h));

            const std::string label = json_escape(label_for(d.class_id));

            if (!first) ss << ',';
            first = false;

            ss << "{\"class_id\":" << d.class_id
               << ",\"label\":\"" << label << "\""
               << ",\"score\":" << d.score
               << ",\"x\":" << x
               << ",\"y\":" << y
               << ",\"w\":" << w
               << ",\"h\":" << h
               << "}";
        }
        ss << "]}";

        std_msgs::msg::String out;
        out.data = ss.str();
        detections_pub_->publish(out);
    }

    void maybe_track_person(int image_w, int image_h, const std::vector<Det> &dets)
    {
        const int track_id = track_class_id_cached_;

        const float score_thresh = static_cast<float>(get_parameter("score_threshold").as_double());

        const Det *best = nullptr;
        for (const auto &d : dets) {
            if (d.score < score_thresh) continue;
            if (track_id >= 0 && d.class_id != track_id) continue;
            if (best == nullptr || d.score > best->score) {
                best = &d;
            }
        }
        if (best == nullptr) {
            return;
        }

        const float cx = (best->x1 + best->x2) * 0.5f;
        const float cy = (best->y1 + best->y2) * 0.5f;

        const float ex = (cx - 0.5f) / 0.5f; // normalized -1..1
        const float ey = (cy - 0.5f) / 0.5f;

        const double deadband = get_parameter("deadband_norm").as_double();
        if (std::abs(ex) < deadband && std::abs(ey) < deadband) {
            return;
        }

        const double kp_pan = get_parameter("kp_pan_deg").as_double();
        const double kp_tilt = get_parameter("kp_tilt_deg").as_double();
        const double max_step = std::max(0.1, get_parameter("max_step_deg").as_double());

        const int pan_sign = pan_sign_live_.load();
        const int tilt_sign = tilt_sign_live_.load();

        double pan_step = static_cast<double>(pan_sign) * kp_pan * static_cast<double>(ex);
        double tilt_step = static_cast<double>(tilt_sign) * kp_tilt * static_cast<double>(ey);

        pan_step = clampf(static_cast<float>(pan_step), static_cast<float>(-max_step), static_cast<float>(max_step));
        tilt_step = clampf(static_cast<float>(tilt_step), static_cast<float>(-max_step), static_cast<float>(max_step));

        pan_deg_ += pan_step;
        tilt_deg_ += tilt_step;

        // When follow mode is active, add a re-centering spring that biases
        // the gimbal toward neutral.  This forces the robot base to handle
        // the heavy yaw so the gimbal stays near center and available for
        // fast fine corrections.  The spring intentionally leaves a residual
        // camera error so the base angular controller has a signal to react to.
        if (follow_enabled_.load()) {
            const double spring = get_parameter("follow_gimbal_recenter_kp").as_double();
            const double pn = get_parameter("pan_neutral_deg").as_double();
            const double tn = get_parameter("tilt_neutral_deg").as_double();
            pan_deg_  += spring * (pn - pan_deg_);
            tilt_deg_ += spring * (tn - tilt_deg_);
        }

        const double pan_min = get_parameter("pan_min_deg").as_double();
        const double pan_max = get_parameter("pan_max_deg").as_double();
        const double tilt_min = get_parameter("tilt_min_deg").as_double();
        const double tilt_max = get_parameter("tilt_max_deg").as_double();

        pan_deg_ = std::max(pan_min, std::min(pan_max, pan_deg_));
        tilt_deg_ = std::max(tilt_min, std::min(tilt_max, tilt_deg_));

        geometry_msgs::msg::Vector3 cmd;
        cmd.x = pan_deg_;
        cmd.y = tilt_deg_;
        cmd.z = 0.0;
        gimbal_pub_->publish(cmd);
    }

    // ----------------------------------------------------------------
    // Auto-follow: closed-loop PID visual servoing for the robot base
    // ----------------------------------------------------------------

    void follow_subject(const std::vector<Det> &dets)
    {
        const int track_id = track_class_id_cached_;

        const float score_thresh = static_cast<float>(get_parameter("score_threshold").as_double());

        const Det *best = nullptr;
        for (const auto &d : dets) {
            if (d.score < score_thresh) continue;
            if (track_id >= 0 && d.class_id != track_id) continue;
            if (best == nullptr || d.score > best->score) {
                best = &d;
            }
        }

        const auto now = steady_clock::now();

        if (best == nullptr) {
            // No target – the safety timer will send stop after timeout.
            return;
        }

        // Target found – update last-seen timestamp.
        last_follow_detection_time_ = now;

        // ---- Error signals ----

        const float cx     = (best->x1 + best->x2) * 0.5f;
        const float bbox_w = best->x2 - best->x1;
        const float bbox_h = best->y2 - best->y1;
        const float bbox_area = bbox_w * bbox_h;

        // --- Angular error ---
        //
        // We blend two complementary signals for the robot base yaw:
        //   1. Camera error  – raw bbox-centre offset (fast, direct reaction)
        //   2. Gimbal offset – how far the gimbal has moved from neutral
        //      (steady-state, lets the base "unwind" the gimbal back to centre)
        //
        // The gimbal re-centering spring (in maybe_track_person) intentionally
        // prevents the gimbal from perfectly centring the bbox, so the camera
        // error stays non-zero and gives the base a fast reaction signal.
        // The blend factor alpha controls the mix (0 = all camera, 1 = all gimbal).
        double err_angular = 0.0;

        const double cam_err = static_cast<double>((cx - 0.5f) / 0.5f);  // −1 … +1
        const int pan_sign = pan_sign_live_.load();

        const bool use_gimbal_fb = get_parameter("follow_use_gimbal_feedback").as_bool();
        if (use_gimbal_fb && tracking_enabled_.load()) {
            const double pan_neutral = get_parameter("pan_neutral_deg").as_double();
            const double pan_min    = get_parameter("pan_min_deg").as_double();
            const double pan_max    = get_parameter("pan_max_deg").as_double();
            const double half_range = std::max(1.0, (pan_max - pan_min) * 0.5);
            const double gimbal_err = static_cast<double>(pan_sign)
                                    * (pan_deg_ - pan_neutral) / half_range;  // −1 … +1

            const double alpha = get_parameter("follow_angular_blend").as_double();
            err_angular = alpha * gimbal_err + (1.0 - alpha) * cam_err;
        } else {
            // No gimbal tracking – use camera error directly.
            err_angular = cam_err;
        }

        // --- Linear error (distance keeping via bbox area) ---
        const double target_area = follow_target_bbox_area_.load();
        // Positive when subject is too far (bbox smaller than target) → forward.
        const double err_linear = target_area - static_cast<double>(bbox_area);

        // ---- dt ----
        double dt = 0.1;
        if (follow_prev_time_.time_since_epoch().count() > 0) {
            dt = std::chrono::duration<double>(now - follow_prev_time_).count();
            dt = std::max(0.01, std::min(dt, 1.0));
        }
        follow_prev_time_ = now;

        // ---- PID gains ----
        const double kp_lin = get_parameter("follow_kp_linear").as_double();
        const double ki_lin = get_parameter("follow_ki_linear").as_double();
        const double kd_lin = get_parameter("follow_kd_linear").as_double();
        const double kp_ang = get_parameter("follow_kp_angular").as_double();
        const double ki_ang = get_parameter("follow_ki_angular").as_double();
        const double kd_ang = get_parameter("follow_kd_angular").as_double();

        // ---- Deadbands ----
        const double db_lin = get_parameter("follow_linear_deadband").as_double();
        const double db_ang = get_parameter("follow_angular_deadband").as_double();
        const double eff_lin = (std::abs(err_linear)  < db_lin) ? 0.0 : err_linear;
        const double eff_ang = (std::abs(err_angular) < db_ang) ? 0.0 : err_angular;

        // ---- Integral (anti-windup clamp) ----
        follow_integral_linear_  += eff_lin * dt;
        follow_integral_angular_ += eff_ang * dt;
        constexpr double kMaxIntegral = 2.0;
        follow_integral_linear_  = std::max(-kMaxIntegral, std::min(kMaxIntegral, follow_integral_linear_));
        follow_integral_angular_ = std::max(-kMaxIntegral, std::min(kMaxIntegral, follow_integral_angular_));

        // ---- Derivative ----
        const double d_lin = (dt > 1e-3) ? (eff_lin - follow_prev_err_linear_)  / dt : 0.0;
        const double d_ang = (dt > 1e-3) ? (eff_ang - follow_prev_err_angular_) / dt : 0.0;
        follow_prev_err_linear_  = eff_lin;
        follow_prev_err_angular_ = eff_ang;

        // ---- PID output ----
        double cmd_linear  =  (kp_lin * eff_lin + ki_lin * follow_integral_linear_  + kd_lin * d_lin);
        // Negative: +err_angular (subject right) → −angular.z (clockwise in ROS).
        double cmd_angular = -(kp_ang * eff_ang + ki_ang * follow_integral_angular_ + kd_ang * d_ang);

        // ---- Clamp to safe velocity limits ----
        const double max_lin = follow_max_linear_.load();
        const double max_ang = get_parameter("follow_max_angular").as_double();
        cmd_linear  = std::max(-max_lin, std::min(max_lin, cmd_linear));
        cmd_angular = std::max(-max_ang, std::min(max_ang, cmd_angular));

        // ---- Ultrasonic obstacle avoidance (forward motion only) ----
        if (cmd_linear > 0.0) {
            const double range_m = ultrasonic_range_m_.load();
            const double stop_m  = get_parameter("follow_obstacle_stop_m").as_double();
            const double slow_m  = get_parameter("follow_obstacle_slow_m").as_double();

            if (range_m <= stop_m) {
                cmd_linear = 0.0;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "follow: OBSTACLE at %.2fm – forward motion blocked", range_m);
            } else if (range_m < slow_m && slow_m > stop_m) {
                const double scale = (range_m - stop_m) / (slow_m - stop_m);
                cmd_linear *= scale;
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                    "follow: obstacle at %.2fm – speed scaled to %.0f%%",
                    range_m, scale * 100.0);
            }
        }

        // ---- Publish Twist ----
        geometry_msgs::msg::Twist twist;
        twist.linear.x  = cmd_linear;
        twist.angular.z = cmd_angular;
        cmd_vel_pub_->publish(twist);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "follow: area=%.3f err_lin=%.3f err_ang=%.3f(cam=%.2f) -> v=%.2f w=%.2f pan=%.0f us=%.2fm",
            static_cast<double>(bbox_area), err_linear, err_angular, cam_err,
            cmd_linear, cmd_angular, pan_deg_, ultrasonic_range_m_.load());
    }

    /// Called at 10 Hz – manages lost-target state machine:
    ///   TRACKING → (lost for timeout) → SCANNING → (found) → TRACKING
    ///                                            → (sweeps exhausted) → IDLE
    void follow_safety_check()
    {
        if (!follow_enabled_.load()) {
            if (scan_state_ != ScanState::IDLE) {
                end_scan();
            }
            return;
        }

        const auto now = steady_clock::now();

        switch (scan_state_) {
        case ScanState::IDLE:
        {
            // Normal tracking – check if we lost the target.
            if (last_follow_detection_time_.time_since_epoch().count() == 0) {
                return;  // never seen a target yet
            }
            const double elapsed = std::chrono::duration<double>(
                now - last_follow_detection_time_).count();
            const double timeout = get_parameter("follow_lost_timeout_sec").as_double();
            if (elapsed > timeout) {
                publish_stop_cmd_vel();
                reset_follow_pid();
                begin_scan(now);
            }
            break;
        }
        case ScanState::SWEEPING:
        {
            // Check if subject was re-detected (follow_subject updates
            // last_follow_detection_time_ when it finds a target).
            if (last_follow_detection_time_ > scan_started_at_) {
                RCLCPP_INFO(get_logger(), "follow-scan: target RE-ACQUIRED");
                end_scan();
                return;
            }

            // Drive the gimbal toward the current scan edge.
            const double speed = get_parameter("follow_scan_speed_deg").as_double();
            const double dt = 0.1;  // timer period
            double step = speed * dt * (scan_direction_right_ ? 1.0 : -1.0);
            step *= static_cast<double>(pan_sign_live_.load());
            pan_deg_ += step;

            const double pan_min = get_parameter("pan_min_deg").as_double();
            const double pan_max = get_parameter("pan_max_deg").as_double();
            pan_deg_ = std::max(pan_min, std::min(pan_max, pan_deg_));

            geometry_msgs::msg::Vector3 cmd;
            cmd.x = pan_deg_;
            cmd.y = tilt_deg_;
            cmd.z = 0.0;
            gimbal_pub_->publish(cmd);

            // Check if we hit the edge – switch direction.
            if (pan_deg_ <= pan_min + 1.0 || pan_deg_ >= pan_max - 1.0) {
                scan_direction_right_ = !scan_direction_right_;
                scan_sweep_count_++;
                const int max_sweeps = get_parameter("follow_scan_max_sweeps").as_int();
                if (scan_sweep_count_ >= max_sweeps) {
                    RCLCPP_WARN(get_logger(),
                        "follow-scan: %d sweeps done, target NOT found – giving up",
                        scan_sweep_count_);
                    end_scan();
                    // Return gimbal to neutral.
                    pan_deg_ = get_parameter("pan_neutral_deg").as_double();
                    tilt_deg_ = get_parameter("tilt_neutral_deg").as_double();
                    geometry_msgs::msg::Vector3 center;
                    center.x = pan_deg_;
                    center.y = tilt_deg_;
                    center.z = 0.0;
                    gimbal_pub_->publish(center);
                    // Reset follow detection so we don't immediately re-enter scan.
                    last_follow_detection_time_ = steady_clock::time_point(
                        steady_clock::duration::zero());
                }
            }

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "follow-scan: sweeping %s, pan=%.1f, sweep %d",
                scan_direction_right_ ? "RIGHT" : "LEFT",
                pan_deg_, scan_sweep_count_);
            break;
        }
        }  // switch
    }

    void begin_scan(const steady_clock::time_point &now)
    {
        scan_state_ = ScanState::SWEEPING;
        scan_started_at_ = now;
        scan_sweep_count_ = 0;

        // Start scanning in the direction the gimbal was already offset.
        const double pan_neutral = get_parameter("pan_neutral_deg").as_double();
        scan_direction_right_ = (pan_deg_ >= pan_neutral);

        RCLCPP_WARN(get_logger(), "follow-scan: target LOST – starting gimbal scan");
    }

    void end_scan()
    {
        scan_state_ = ScanState::IDLE;
        scan_sweep_count_ = 0;
    }

    void publish_stop_cmd_vel()
    {
        geometry_msgs::msg::Twist twist;
        twist.linear.x  = 0.0;
        twist.angular.z = 0.0;
        cmd_vel_pub_->publish(twist);
    }

    void reset_follow_pid()
    {
        follow_integral_linear_  = 0.0;
        follow_integral_angular_ = 0.0;
        follow_prev_err_linear_  = 0.0;
        follow_prev_err_angular_ = 0.0;
        follow_prev_time_ = steady_clock::time_point(steady_clock::duration::zero());
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detections_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr gimbal_pub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr img_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tracking_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr tracking_cfg_sub_;
    rclcpp::TimerBase::SharedPtr load_timer_;

    std::atomic<bool> tracking_enabled_{false};
    std::atomic<int> pan_sign_live_{1};
    std::atomic<int> tilt_sign_live_{1};

    std::shared_ptr<hailort::VDevice> vdevice_;
    std::shared_ptr<hailort::InferModel> infer_model_;
    std::unique_ptr<hailort::ConfiguredInferModel> configured_;

    std::unique_ptr<hailort::Buffer> input_buffer_;
    std::unique_ptr<hailort::Buffer> output_buffer_;

    std::string input_name_;
    std::string output_name_;

    int input_w_{0};
    int input_h_{0};
    size_t input_frame_size_{0};
    size_t output_frame_size_{0};

    std::vector<std::string> labels_;
    int track_class_id_cached_{-1};

    double pan_deg_{90.0};
    double tilt_deg_{90.0};

    // Auto-follow state
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr follow_enable_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr follow_target_area_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr follow_max_linear_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr ultrasonic_sub_;
    std::atomic<double> ultrasonic_range_m_{999.0};
    rclcpp::TimerBase::SharedPtr follow_safety_timer_;
    std::atomic<bool> follow_enabled_{false};
    std::atomic<double> follow_target_bbox_area_{0.04};
    std::atomic<double> follow_max_linear_{0.3};
    steady_clock::time_point last_follow_detection_time_{steady_clock::duration::zero()};
    steady_clock::time_point follow_prev_time_{steady_clock::duration::zero()};
    double follow_integral_linear_{0.0};
    double follow_integral_angular_{0.0};
    double follow_prev_err_linear_{0.0};
    double follow_prev_err_angular_{0.0};

    // Gimbal scan-to-reacquire state
    enum class ScanState { IDLE, SWEEPING };
    ScanState scan_state_{ScanState::IDLE};
    steady_clock::time_point scan_started_at_{steady_clock::duration::zero()};
    bool scan_direction_right_{true};
    int scan_sweep_count_{0};

    steady_clock::time_point last_infer_time_;

    // ── Depth estimation (shares vdevice_ with detector) ─────────────

    void on_depth_image(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        if (!depth_enabled_.load() || !depth_configured_) {
            return;
        }

        const double fps = std::max(1.0, get_parameter("depth_inference_fps").as_double());
        const auto min_dt = std::chrono::duration_cast<steady_clock::duration>(
            std::chrono::duration<double>(1.0 / fps));
        const auto now = steady_clock::now();
        if ((now - last_depth_infer_time_) < min_dt) {
            return;
        }
        last_depth_infer_time_ = now;

        if (msg->data.empty()) {
            return;
        }
        cv::Mat raw(1, static_cast<int>(msg->data.size()), CV_8UC1,
                    const_cast<unsigned char*>(msg->data.data()));
        cv::Mat bgr = cv::imdecode(raw, cv::IMREAD_COLOR);
        if (bgr.empty()) {
            return;
        }

        const int orig_w = bgr.cols;
        const int orig_h = bgr.rows;

        cv::Mat resized;
        cv::resize(bgr, resized, cv::Size(depth_input_w_, depth_input_h_), 0, 0, cv::INTER_LINEAR);
        cv::Mat rgb;
        cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
        if (!rgb.isContinuous()) {
            rgb = rgb.clone();
        }

        if (static_cast<size_t>(rgb.total() * rgb.elemSize()) < depth_input_frame_size_) {
            return;
        }

        std::memcpy(depth_input_buffer_->data(), rgb.data, depth_input_frame_size_);

        std::map<std::string, hailort::MemoryView> buffers;
        buffers.emplace(depth_input_name_, hailort::MemoryView(*depth_input_buffer_));
        buffers.emplace(depth_output_name_, hailort::MemoryView(*depth_output_buffer_));

        auto bindings_exp = depth_configured_->create_bindings(buffers);
        if (!bindings_exp) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "depth create_bindings failed (status=%d)",
                                 static_cast<int>(bindings_exp.status()));
            return;
        }
        auto bindings = bindings_exp.release();

        const auto st = depth_configured_->run(bindings, 100ms);
        if (HAILO_SUCCESS != st) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "depth hailo run failed status=%d", static_cast<int>(st));
            return;
        }

        const float *depth_data = reinterpret_cast<const float*>(depth_output_buffer_->data());
        cv::Mat depth_small(depth_input_h_, depth_input_w_, CV_32FC1, const_cast<float*>(depth_data));

        cv::Mat depth_full;
        cv::resize(depth_small, depth_full, cv::Size(orig_w, orig_h), 0, 0, cv::INTER_LINEAR);
        cv::max(depth_full, 0.0f, depth_full);

        // Publish raw depth (32FC1)
        if (depth_pub_->get_subscription_count() > 0) {
            auto depth_msg = std::make_unique<sensor_msgs::msg::Image>();
            depth_msg->header = msg->header;
            depth_msg->height = static_cast<uint32_t>(depth_full.rows);
            depth_msg->width = static_cast<uint32_t>(depth_full.cols);
            depth_msg->encoding = "32FC1";
            depth_msg->is_bigendian = false;
            depth_msg->step = static_cast<uint32_t>(depth_full.cols * sizeof(float));
            const size_t data_size = depth_msg->step * depth_full.rows;
            depth_msg->data.resize(data_size);
            std::memcpy(depth_msg->data.data(), depth_full.data, data_size);
            depth_pub_->publish(std::move(depth_msg));
        }

        // Publish colorized depth (JPEG) for web UI
        if (depth_colorized_pub_->get_subscription_count() > 0) {
            const double max_depth = std::max(0.1, get_parameter("depth_max_depth_m").as_double());
            const int colormap = static_cast<int>(get_parameter("depth_colormap").as_int());
            const int jpeg_quality = static_cast<int>(get_parameter("depth_jpeg_quality").as_int());

            cv::Mat depth_norm;
            depth_full.convertTo(depth_norm, CV_8UC1, 255.0 / max_depth);

            cv::Mat depth_color;
            cv::applyColorMap(depth_norm, depth_color, colormap);

            std::vector<int> enc_params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality};
            std::vector<uchar> jpeg_buf;
            cv::imencode(".jpg", depth_color, jpeg_buf, enc_params);

            auto color_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
            color_msg->header = msg->header;
            color_msg->format = "jpeg";
            color_msg->data = std::move(jpeg_buf);
            depth_colorized_pub_->publish(std::move(color_msg));
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr depth_colorized_pub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr depth_img_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr depth_enable_sub_;

    std::shared_ptr<hailort::InferModel> depth_infer_model_;
    std::unique_ptr<hailort::ConfiguredInferModel> depth_configured_;
    std::unique_ptr<hailort::Buffer> depth_input_buffer_;
    std::unique_ptr<hailort::Buffer> depth_output_buffer_;
    std::string depth_input_name_;
    std::string depth_output_name_;
    int depth_input_w_{0};
    int depth_input_h_{0};
    size_t depth_input_frame_size_{0};
    size_t depth_output_frame_size_{0};

    std::atomic<bool> depth_enabled_{true};
    steady_clock::time_point last_depth_infer_time_{steady_clock::duration::zero()};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HailoDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
