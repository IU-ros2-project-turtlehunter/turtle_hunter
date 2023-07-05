#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
#include "geometry_msgs/msg/twist.hpp"

#define INITIAL_TURTLES 4 // consider making this as a parameter of a node (should be the same for both game_proc_node
//  and this node, so maybe should be passed inside launch file)

using namespace std::chrono_literals;
using turtle_pos = turtlesim::msg::Pose;
using Twist = geometry_msgs::msg::Twist;

struct coord {
    double x;
    double y;
};

double distance (coord c1, coord c2) {
    return sqrt(pow((c1.x - c2.x), 2) + pow((c1.y - c2.y),2));
}

/**
 * This node is responsible for controlling the hunter, it navigates it towards a victim.
 */
class Hunter_Node : public rclcpp::Node {
public: Hunter_Node() : Node("hunter_node")
    {
        timer_ = this->create_wall_timer(10ms, std::bind(&Hunter_Node::navigation_callback, this));

        turtles_positions.assign(INITIAL_TURTLES, {});
        movement_publisher = this->create_publisher<Twist>("/turtle1/cmd_vel", 10);
        for (unsigned int turtle = 1; turtle <= INITIAL_TURTLES; turtle++) {
            std::string topic_name = "/turtle" + std::to_string(turtle) + "/pose";
            position_checkers.push_back(this->create_subscription<turtle_pos>(topic_name, 10,
             [turtle, this](const turtle_pos & msg) { this->pos_check_callback(msg, turtle); }));
        }

        determine_next_victim();
    }
private:
    /**
     * Adjusts the direction of turtle
     */
    void navigation_callback() {
        auto twist_msg = Twist();

        double dist = distance(turtles_positions[0], turtles_positions[next_target - 1]);
        double hunt_x = turtles_positions[0].x;
        double hunt_y = turtles_positions[0].y;
        double vict_x = turtles_positions[next_target - 1].x;
        double vict_y = turtles_positions[next_target - 1].y;

        twist_msg.linear.x = dist * 2;
        double desired_theta = atan2((vict_y - hunt_y), (vict_x - hunt_x));

        twist_msg.angular.z = (desired_theta - hunter_theta) * 10;

        this->movement_publisher->publish(twist_msg);

        /*
         * Now the determination of the closest victim is done every 10ms...
         */
        determine_next_victim();
    }

    void pos_check_callback(const turtle_pos & msg, unsigned int turtle_num) {  // just store the acknowledged position
        turtles_positions[turtle_num-1] = {msg.x, msg.y};
        if (turtle_num == 1)
            hunter_theta = msg.theta;
    }

    void determine_next_victim() {
        /*
         * This code is assuming that we are hunting the closest victim.
         */
        double min_dist = INT32_MAX;
        for (int i = 2; i < INITIAL_TURTLES; i++) {
            double current = distance(turtles_positions[0], turtles_positions[i - 1]);
            if (min_dist > current) {
                min_dist = current;
                next_target = i;
            }
        }
    }

    unsigned int next_target = 0;  // for storing what turtle is going to be the next victim.
    double hunter_theta = 0;
    std::vector<coord> turtles_positions; // for storing turtles positions
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<rclcpp::Subscription<turtle_pos>::SharedPtr> position_checkers;  // for retrieving turtle poses
    rclcpp::Publisher<Twist>::SharedPtr movement_publisher;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Hunter_Node>());
    rclcpp::shutdown();
    return 0;
}
