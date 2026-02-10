/**
 * @file hailo_depth_node.cpp
 * @brief ROS 2 node for monocular depth estimation using Hailo-8 with fast_depth model.
 *
 * Subscribes to a CompressedImage topic (e.g. front_camera/compressed from Pi Camera Module 3),
 * runs fast_depth inference on Hailo-8, and publishes:
 *   - depth/image (sensor_msgs/Image, 32FC1) — raw metric depth for SLAM / obstacle avoidance
 *   - depth/colorized/compressed (sensor_msgs/CompressedImage, JPEG) — colorized depth for web UI
 */

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>

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
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class HailoDepthNode final : public rclcpp::Node
{
public:
    HailoDepthNode() : rclcpp::Node("hailo_depth")
    {
        declare_parameter<std::string>("hef_path", "");
        declare_parameter<std::string>("model_name", "");
        declare_parameter<std::string>("input_topic", "front_camera/compressed");
        declare_parameter<std::string>("depth_image_topic", "depth/image");
        declare_parameter<std::string>("depth_colorized_topic", "depth/colorized/compressed");
        declare_parameter<std::string>("enable_topic", "depth/enable");
        declare_parameter<double>("inference_fps", 5.0);
        declare_parameter<int>("colormap", 20);       // COLORMAP_TURBO = 20
        declare_parameter<int>("jpeg_quality", 75);
        declare_parameter<double>("max_depth_m", 10.0); // NYU Depth V2 range

        const auto input_topic = get_parameter("input_topic").as_string();
        const auto depth_image_topic = get_parameter("depth_image_topic").as_string();
        const auto depth_colorized_topic = get_parameter("depth_colorized_topic").as_string();
        const auto enable_topic = get_parameter("enable_topic").as_string();

        sub_image_ = create_subscription<sensor_msgs::msg::CompressedImage>(
            input_topic, rclcpp::SensorDataQoS(),
            std::bind(&HailoDepthNode::on_image, this, std::placeholders::_1));

        pub_depth_ = create_publisher<sensor_msgs::msg::Image>(depth_image_topic, 10);
        pub_depth_colorized_ = create_publisher<sensor_msgs::msg::CompressedImage>(depth_colorized_topic, 10);

        sub_enable_ = create_subscription<std_msgs::msg::Bool>(
            enable_topic, 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                enabled_.store(msg->data);
                RCLCPP_INFO(get_logger(), "Depth estimation %s", msg->data ? "ENABLED" : "DISABLED");
            });

        // Start enabled by default
        enabled_.store(true);

        RCLCPP_INFO(get_logger(), "HailoDepthNode created — subscribing to '%s'", input_topic.c_str());
        RCLCPP_INFO(get_logger(), "Publishing depth image on '%s', colorized on '%s'",
                     depth_image_topic.c_str(), depth_colorized_topic.c_str());

        // Attempt initial model load
        try_load_model();
    }

private:
    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_image_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_depth_colorized_;

    // Hailo resources
    std::shared_ptr<hailort::VDevice> vdevice_;
    std::shared_ptr<hailort::InferModel> infer_model_;
    std::unique_ptr<hailort::ConfiguredInferModel> configured_;
    std::unique_ptr<hailort::Buffer> input_buffer_;
    std::unique_ptr<hailort::Buffer> output_buffer_;

    std::string input_name_;
    std::string output_name_;
    int input_w_ = 0;
    int input_h_ = 0;
    size_t input_frame_size_ = 0;
    size_t output_frame_size_ = 0;

    // State
    std::atomic<bool> enabled_{true};
    std::chrono::steady_clock::time_point last_infer_time_{};

    void try_load_model()
    {
        if (configured_) {
            return;
        }

        const auto hef_path = get_parameter("hef_path").as_string();
        if (hef_path.empty()) {
            RCLCPP_WARN(get_logger(), "hef_path is empty; set -p hef_path:=/path/to/fast_depth.hef");
            return;
        }

        const auto model_name = get_parameter("model_name").as_string();

        // Create VDevice (Hailo-8)
        auto vdevice_exp = hailort::VDevice::create_shared();
        if (!vdevice_exp) {
            RCLCPP_ERROR(get_logger(), "Failed to create VDevice (status=%d)",
                         static_cast<int>(vdevice_exp.status()));
            return;
        }
        vdevice_ = vdevice_exp.release();

        // Load HEF
        auto infer_model_exp = vdevice_->create_infer_model(hef_path, model_name);
        if (!infer_model_exp) {
            RCLCPP_ERROR(get_logger(), "Failed to create InferModel from '%s' (status=%d)",
                         hef_path.c_str(), static_cast<int>(infer_model_exp.status()));
            vdevice_.reset();
            return;
        }
        infer_model_ = infer_model_exp.release();

        // Configure input: UINT8, NHWC (RGB)
        auto in_exp = infer_model_->input();
        if (!in_exp) {
            RCLCPP_ERROR(get_logger(), "InferModel::input failed (status=%d)",
                         static_cast<int>(in_exp.status()));
            reset_model();
            return;
        }
        auto input_stream = in_exp.release();
        input_stream.set_format_type(HAILO_FORMAT_TYPE_UINT8);
        input_stream.set_format_order(HAILO_FORMAT_ORDER_NHWC);

        // Configure output: FLOAT32, NHWC (dense depth map, NOT NMS)
        auto out_exp = infer_model_->output();
        if (!out_exp) {
            RCLCPP_ERROR(get_logger(), "InferModel::output failed (status=%d)",
                         static_cast<int>(out_exp.status()));
            reset_model();
            return;
        }
        auto output_stream = out_exp.release();

        // fast_depth output is a dense 224x224x1 depth map — request float32 for metric depth
        output_stream.set_format_type(HAILO_FORMAT_TYPE_FLOAT32);

        // Configure the model on the device
        auto configured_exp = infer_model_->configure();
        if (!configured_exp) {
            RCLCPP_ERROR(get_logger(), "InferModel::configure failed (status=%d)",
                         static_cast<int>(configured_exp.status()));
            reset_model();
            return;
        }
        configured_ = std::make_unique<hailort::ConfiguredInferModel>(configured_exp.release());

        // Get input dimensions
        const auto shape = input_stream.shape();
        input_h_ = static_cast<int>(shape.height);
        input_w_ = static_cast<int>(shape.width);

        input_frame_size_ = input_stream.get_frame_size();
        output_frame_size_ = output_stream.get_frame_size();

        // Allocate IO buffers
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

        RCLCPP_INFO(get_logger(),
                     "Loaded fast_depth HEF '%s' (input=%dx%d bytes=%zu, output bytes=%zu, depth pixels=%zu)",
                     hef_path.c_str(), input_w_, input_h_,
                     input_frame_size_, output_frame_size_,
                     output_frame_size_ / sizeof(float));
    }

    void reset_model()
    {
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
        if (!enabled_.load()) {
            return;
        }

        if (!configured_) {
            // Lazy-load: retry once if model wasn't loaded at construction
            try_load_model();
            if (!configured_) {
                return;
            }
        }

        // Rate-limit inference
        const double fps = std::max(1.0, get_parameter("inference_fps").as_double());
        const auto min_dt = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(1.0 / fps));
        const auto now = std::chrono::steady_clock::now();
        if ((now - last_infer_time_) < min_dt) {
            return;
        }
        last_infer_time_ = now;

        // Decode JPEG
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

        // Resize to model input size and convert BGR→RGB
        cv::Mat resized;
        cv::resize(bgr, resized, cv::Size(input_w_, input_h_), 0, 0, cv::INTER_LINEAR);
        cv::Mat rgb;
        cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
        if (!rgb.isContinuous()) {
            rgb = rgb.clone();
        }

        if (static_cast<size_t>(rgb.total() * rgb.elemSize()) < input_frame_size_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "RGB buffer too small for input frame size");
            return;
        }

        // Copy input data
        std::memcpy(input_buffer_->data(), rgb.data, input_frame_size_);

        // Create bindings and run inference
        std::map<std::string, hailort::MemoryView> buffers;
        buffers.emplace(input_name_, hailort::MemoryView(*input_buffer_));
        buffers.emplace(output_name_, hailort::MemoryView(*output_buffer_));

        auto bindings_exp = configured_->create_bindings(buffers);
        if (!bindings_exp) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "create_bindings failed (status=%d)",
                                 static_cast<int>(bindings_exp.status()));
            return;
        }
        auto bindings = bindings_exp.release();

        const auto st = configured_->run(bindings, 100ms);
        if (HAILO_SUCCESS != st) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Hailo run failed status=%d", static_cast<int>(st));
            return;
        }

        // Interpret output as float32 depth map (input_h_ x input_w_)
        const size_t depth_pixels = static_cast<size_t>(input_h_) * static_cast<size_t>(input_w_);
        const float *depth_data = reinterpret_cast<const float*>(output_buffer_->data());

        // Build a float32 Mat from Hailo output
        cv::Mat depth_small(input_h_, input_w_, CV_32FC1, const_cast<float*>(depth_data));

        // Resize depth back to original camera resolution
        cv::Mat depth_full;
        cv::resize(depth_small, depth_full, cv::Size(orig_w, orig_h), 0, 0, cv::INTER_LINEAR);

        // Clamp negative values (if any) to 0
        cv::max(depth_full, 0.0f, depth_full);

        // Publish raw depth as sensor_msgs/Image (32FC1)
        if (pub_depth_->get_subscription_count() > 0) {
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
            pub_depth_->publish(std::move(depth_msg));
        }

        // Publish colorized depth as CompressedImage (JPEG) for web UI
        if (pub_depth_colorized_->get_subscription_count() > 0) {
            const double max_depth = std::max(0.1, get_parameter("max_depth_m").as_double());
            const int colormap = static_cast<int>(get_parameter("colormap").as_int());
            const int jpeg_quality = static_cast<int>(get_parameter("jpeg_quality").as_int());

            // Normalize depth to 0-255 range
            cv::Mat depth_norm;
            depth_full.convertTo(depth_norm, CV_8UC1, 255.0 / max_depth);

            // Apply colormap (TURBO by default for intuitive depth visualization)
            cv::Mat depth_color;
            cv::applyColorMap(depth_norm, depth_color, colormap);

            // Encode as JPEG
            std::vector<int> encode_params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality};
            std::vector<uchar> jpeg_buf;
            cv::imencode(".jpg", depth_color, jpeg_buf, encode_params);

            auto color_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
            color_msg->header = msg->header;
            color_msg->format = "jpeg";
            color_msg->data = std::move(jpeg_buf);
            pub_depth_colorized_->publish(std::move(color_msg));
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HailoDepthNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
