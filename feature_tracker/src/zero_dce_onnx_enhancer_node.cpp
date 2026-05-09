#include <algorithm>
#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <onnxruntime_cxx_api.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

namespace
{
const int kImageHeight = 480;
const int kImageWidth = 640;
const int kImageChannels = 3;

double elapsedMs(const std::chrono::steady_clock::time_point &start)
{
    const auto end = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end - start).count();
}

float clamp01(float value)
{
    return std::max(0.0f, std::min(1.0f, value));
}
}  // namespace

class ZeroDceOnnxEnhancerNode
{
public:
    ZeroDceOnnxEnhancerNode()
        : private_nh_("~"),
          env_(ORT_LOGGING_LEVEL_WARNING, "zero_dce_onnx_enhancer")
    {
        private_nh_.param<std::string>("input_topic", input_topic_, "/camera/color/image_raw");
        private_nh_.param<std::string>("output_topic", output_topic_, "/zero_dce/image_enhanced");
        private_nh_.param<std::string>("model_path", model_path_,
                                       std::string("feature_tracker/models/zero_dce_plus_480x640_sf12.onnx"));
        private_nh_.param<int>("queue_size", queue_size_, 2);
        private_nh_.param<int>("intra_op_num_threads", intra_op_num_threads_, 1);
        private_nh_.param<bool>("publish_original_on_error", publish_original_on_error_, true);
        private_nh_.param<bool>("profile", profile_, false);

        initSession();

        publisher_ = private_nh_.advertise<sensor_msgs::Image>(output_topic_, queue_size_);
        subscriber_ = private_nh_.subscribe(input_topic_, queue_size_, &ZeroDceOnnxEnhancerNode::imageCallback, this,
                                            ros::TransportHints().tcpNoDelay());

        ROS_INFO("Zero-DCE++ ONNX enhancer subscribed to %s, publishing %s, model=%s, intra_op_num_threads=%d",
                 input_topic_.c_str(), output_topic_.c_str(), model_path_.c_str(), intra_op_num_threads_);
    }

private:
    void initSession()
    {
        Ort::SessionOptions options;
        options.SetIntraOpNumThreads(std::max(1, intra_op_num_threads_));
        options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

        session_.reset(new Ort::Session(env_, model_path_.c_str(), options));

        Ort::AllocatorWithDefaultOptions allocator;
        char *input_name = session_->GetInputName(0, allocator);
        char *output_name = session_->GetOutputName(0, allocator);
        input_name_ = input_name;
        output_name_ = output_name;
        allocator.Free(input_name);
        allocator.Free(output_name);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        const auto start = std::chrono::steady_clock::now();
        try
        {
            cv::Mat rgb = messageToRgb(msg);
            if (rgb.rows != kImageHeight || rgb.cols != kImageWidth)
            {
                throw std::runtime_error("Zero-DCE++ ONNX expects 640x480 images");
            }

            cv::Mat enhanced_bgr = enhance(rgb);
            sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8,
                                                               enhanced_bgr)
                                                .toImageMsg();
            publisher_.publish(out_msg);
            if (profile_)
            {
                ROS_INFO("Zero-DCE++ ONNX inference %.3f ms", elapsedMs(start));
            }
        }
        catch (const std::exception &exc)
        {
            ROS_WARN_THROTTLE(1.0, "Zero-DCE++ ONNX enhancement failed: %s", exc.what());
            if (publish_original_on_error_)
            {
                publisher_.publish(msg);
            }
        }
    }

    cv::Mat messageToRgb(const sensor_msgs::ImageConstPtr &msg)
    {
        if (msg->encoding == sensor_msgs::image_encodings::RGB8)
        {
            return cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8)->image;
        }

        if (msg->encoding == sensor_msgs::image_encodings::BGR8)
        {
            cv::Mat rgb;
            cv::Mat bgr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
            cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
            return rgb;
        }

        if (msg->encoding == sensor_msgs::image_encodings::MONO8 || msg->encoding == "8UC1")
        {
            cv::Mat rgb;
            cv::Mat gray = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
            cv::cvtColor(gray, rgb, cv::COLOR_GRAY2RGB);
            return rgb;
        }

        cv::Mat rgb;
        cv::Mat bgr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
        return rgb;
    }

    cv::Mat enhance(const cv::Mat &rgb)
    {
        std::vector<float> input(kImageChannels * kImageHeight * kImageWidth);
        for (int y = 0; y < kImageHeight; ++y)
        {
            const cv::Vec3b *row = rgb.ptr<cv::Vec3b>(y);
            for (int x = 0; x < kImageWidth; ++x)
            {
                for (int c = 0; c < kImageChannels; ++c)
                {
                    input[c * kImageHeight * kImageWidth + y * kImageWidth + x] =
                        static_cast<float>(row[x][c]) / 255.0f;
                }
            }
        }

        std::array<int64_t, 4> input_shape = {{1, kImageChannels, kImageHeight, kImageWidth}};
        Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info, input.data(), input.size(), input_shape.data(), input_shape.size());

        const char *input_names[] = {input_name_.c_str()};
        const char *output_names[] = {output_name_.c_str()};
        auto output_tensors =
            session_->Run(Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, 1);

        const float *output = output_tensors.front().GetTensorMutableData<float>();
        cv::Mat enhanced_rgb(kImageHeight, kImageWidth, CV_8UC3);
        for (int y = 0; y < kImageHeight; ++y)
        {
            cv::Vec3b *row = enhanced_rgb.ptr<cv::Vec3b>(y);
            for (int x = 0; x < kImageWidth; ++x)
            {
                for (int c = 0; c < kImageChannels; ++c)
                {
                    const float value = output[c * kImageHeight * kImageWidth + y * kImageWidth + x];
                    row[x][c] = static_cast<unsigned char>(clamp01(value) * 255.0f + 0.5f);
                }
            }
        }

        cv::Mat enhanced_bgr;
        cv::cvtColor(enhanced_rgb, enhanced_bgr, cv::COLOR_RGB2BGR);
        return enhanced_bgr;
    }

    ros::NodeHandle private_nh_;
    ros::Publisher publisher_;
    ros::Subscriber subscriber_;

    std::string input_topic_;
    std::string output_topic_;
    std::string model_path_;
    std::string input_name_;
    std::string output_name_;
    int queue_size_;
    int intra_op_num_threads_;
    bool publish_original_on_error_;
    bool profile_;

    Ort::Env env_;
    std::unique_ptr<Ort::Session> session_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "zero_dce_onnx_enhancer");
    ZeroDceOnnxEnhancerNode node;
    ros::spin();
    return 0;
}
