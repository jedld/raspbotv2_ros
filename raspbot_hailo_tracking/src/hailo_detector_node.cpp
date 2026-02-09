#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/bool.hpp>
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
        declare_parameter<int>("person_class_id", -1);
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
        declare_parameter<double>("tilt_neutral_deg", 90.0);

        const auto input_topic = get_parameter("input_topic").as_string();
        const auto detections_topic = get_parameter("detections_topic").as_string();
        const auto tracking_enable_topic = get_parameter("tracking_enable_topic").as_string();
        const auto tracking_config_topic = get_parameter("tracking_config_topic").as_string();
        const auto gimbal_topic = get_parameter("gimbal_topic").as_string();

        detections_pub_ = create_publisher<std_msgs::msg::String>(detections_topic, 10);
        gimbal_pub_ = create_publisher<geometry_msgs::msg::Vector3>(gimbal_topic, 10);

        tracking_enabled_.store(get_parameter("tracking_enabled").as_bool());
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

        img_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
            input_topic, rclcpp::SensorDataQoS(),
            std::bind(&HailoDetectorNode::on_image, this, std::placeholders::_1));

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

        // Auto-pick person_class_id if user didn't set one.
        if (get_parameter("person_class_id").as_int() < 0) {
            for (size_t i = 0; i < labels_.size(); i++) {
                if (labels_[i] == "person") {
                    person_class_id_cached_ = static_cast<int>(i);
                    break;
                }
            }
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
        int person_id = get_parameter("person_class_id").as_int();
        if (person_id < 0) {
            person_id = person_class_id_cached_;
        }

        const float score_thresh = static_cast<float>(get_parameter("score_threshold").as_double());

        const Det *best = nullptr;
        for (const auto &d : dets) {
            if (d.score < score_thresh) continue;
            if (person_id >= 0 && d.class_id != person_id) continue;
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
    int person_class_id_cached_{-1};

    double pan_deg_{90.0};
    double tilt_deg_{90.0};

    steady_clock::time_point last_infer_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HailoDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
