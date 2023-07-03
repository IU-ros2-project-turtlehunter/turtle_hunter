#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

/**
 * This node is responsible for controlling the hunter, it navigates it towards a victim.
 */
class Hunter_Node : public rclcpp::Node
{
public:
    Hunter_Node()
            : Node("hunter_node")
    {

    }
private:

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Hunter_Node>());
    rclcpp::shutdown();
    return 0;
}
