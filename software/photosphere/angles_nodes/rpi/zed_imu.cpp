#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensorcapture.hpp"
#include <nlohmann/json.hpp> //nlohmann package for json
#include <fstream>
#include <iostream>
#include <filesystem>

using json = nlohmann::json;

class ZedOCJsonCapture : public rclcpp::Node
{
public:
    ZedOCJsonCapture() : Node("zed_oc_json_capture"), ring_(1), degrees_(0.0f), json_path_("data.json")
    {
        if (!capture_.initialize())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize ZED Open Capture.");
            rclcpp::shutdown();
        }

        RCLCPP_INFO(this->get_logger(), "Press 'x' to capture data for each photo...");

        // Create the publisher
        json_pub_ = this->create_publisher<std_msgs::msg::String>("json_data_topic", 10);

        loop_thread_ = std::thread([this]()
                                   { this->wait_for_input(); });
    }

    ~ZedOCJsonCapture()
    {
        if (loop_thread_.joinable())
            loop_thread_.join();
    }

private:
    void wait_for_input()
    {
        while (rclcpp::ok())
        {
            char key;
            std::cin >> key;
            if (key == 'x' || key == 'X')
            {
                capture_and_publish();
                degrees_ += 22;
                if (degrees_ >= 360)
                    degrees_ = 0;
            }
        }
    }

    void capture_and_publish()
    {
        sl_oc::sensors::data::Imu imu_data;
        if (!capture_.getLastIMUData(imu_data))
        {
            RCLCPP_WARN(this->get_logger(), "Failed to get IMU data.");
            return;
        }

        std::string file_name = "img-r" + std::to_string(ring_) + "-" + std::to_string(int(degrees_)) + ".jpg";

        json new_entry = {
            {"degrees", degrees_},
            {"ring", ring_},
            {"sensors", {{"fileUri", file_name}, {"roll_pitch_yaw", {{"pitch", imu_data.pose.pitch}, {"roll", imu_data.pose.roll}, {"yaw", imu_data.pose.yaw}}}}}};

        json full_data;
        if (std::filesystem::exists(json_path_))
        {
            std::ifstream in(json_path_);
            in >> full_data;
        }
        else
        {
            full_data["angleViewX"] = 5.6447997;
            full_data["angleViewY"] = 4.2447996;
            full_data["focalLength"] = 1294.56427; // fx of left camera
            full_data["pictures"] = json::array();
        }

        full_data["pictures"].push_back(new_entry);

        std::ofstream out(json_path_);
        out << full_data.dump(2);

        // Convert JSON to string and publish to ROS2 topic
        std_msgs::msg::String msg;
        msg.data = full_data.dump(2); // Convert JSON to string with indent

        json_pub_->publish(msg); // Publish JSON data

        RCLCPP_INFO(this->get_logger(), "Data captured and published to json_data_topic");
    }

    sl_oc::sensors::SensorCapture capture_;
    std::thread loop_thread_;
    const int ring_;
    float degrees_;
    std::string json_path_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr json_pub_;
};
