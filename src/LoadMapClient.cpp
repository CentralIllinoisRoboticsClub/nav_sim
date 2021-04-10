#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include <boost/circular_buffer.hpp>

class LoadMapClientNode : public rclcpp::Node
{
public:
    LoadMapClientNode() : Node("load_map_client")
    {
        m_map_path = declare_parameter("map_yaml_file", "default.yaml");
        m_call_count = 0;

        m_timer = create_wall_timer(std::chrono::seconds(2),
                      std::bind(&LoadMapClientNode::updateServerThreads, this) );
    }

    void updateServerThreads()
    {
    	if(true || m_call_count < 3)
    	{
    		++m_call_count;
    		m_threads.push_back(std::thread(std::bind(&LoadMapClientNode::callLoadMap, this) ));
    	}
    }

    void callLoadMap()
    {
        auto client = this->create_client<nav2_msgs::srv::LoadMap>("map_server/load_map");
        while (!client->wait_for_service(std::chrono::milliseconds(100)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the LoadMap server to be up...");
        }

        auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
        request->map_url = m_map_path;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Load Map Service call failed");
        }
    }

private:
    rclcpp::TimerBase::SharedPtr m_timer;
    std::vector<std::thread> m_threads;
    std::string m_map_path;
    uint32_t m_call_count;
    //boost::circular_buffer<std::thread> m_threads;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LoadMapClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
