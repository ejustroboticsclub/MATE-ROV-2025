#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

using json = nlohmann::json;

class JsonSaver : public rclcpp::Node
{
public:
    JsonSaver() : Node("json_saver")
    {
        json_sub_ = this->create_subscription<std_msgs::msg::String>(
            "json_data_topic", 10, std::bind(&JsonSaver::save_json_data, this, std::placeholders::_1));
    }

private:
    void save_json_data(const std_msgs::msg::String::SharedPtr msg)
    {
        try
        {
            json full_data = json::parse(msg->data); // Parse the received string to JSON
            std::ofstream out("data.json");          // json file path
            out << full_data.dump(2);                // Save JSON to the laptop
            RCLCPP_INFO(this->get_logger(), "Data saved to data_on_laptop.json");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error parsing JSON: %s", e.what());
        }
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr json_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JsonSaver>());
    rclcpp::shutdown();
    return 0;
}
