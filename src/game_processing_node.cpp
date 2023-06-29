#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"

#include <vector>
#include <chrono>
#include <functional>
#include <string>

#define INITIAL_TURTLES 4  // can't be less than 2 please...
#define COLLISION_BOUNDARY 0.2

using namespace std::chrono;

using turtle_pos = turtlesim::msg::Pose;
using spawn_srv = turtlesim::srv::Spawn;
using kill_srv = turtlesim::srv::Kill;

struct coord {
    double x;
    double y;
};

double distance (coord c1, coord c2) {
    return sqrt(pow((c1.x - c2.x), 2) + pow((c1.y - c2.y),2));
}

/**
 * This node is responsible for handling elimination process (whether the hunter and a victim collide), spawning
 * process, and other events occurring within the simulation.
 */
class Game_Processing_Node : public rclcpp::Node {
public:
    Game_Processing_Node() : rclcpp::Node("game_processor"), turtles_currently(INITIAL_TURTLES) {

        spawner = this->create_client<spawn_srv>("/spawn");

        for (unsigned int turtle = 1; turtle <= INITIAL_TURTLES; turtle++) {
            std::string topic_name = "/turtle" + std::to_string(turtle) + "/pose";

            position_checkers.push_back(this->create_subscription<turtle_pos>(topic_name, 10,
            [turtle, this](const turtle_pos & msg) { this->pos_check_callback(msg, turtle); }));
        }
    }
private:

    /**
     * This function calls for /spawn service of the turtlesim, for spawning a turtle in a random spot on the map
     */
    void spawn_new_victim() {

    }

    /**
     * Calls for /kill service of the turtlesim, killing a selected turtle
     */
    void kill_turtle() {

    }

    /**
     * Within this function the check for turtle collision (hunter, turtle1, and others) should be done
     * @param msg, description below
     * float32 x
     * float32 y
     * float32 theta
     *
     * float32 linear_velocity
     * float32 angular_velocity
     */
    void pos_check_callback(const turtle_pos & msg, unsigned int turtle_num) {
        turtles_positions[turtle_num-1] = {msg.x, msg.y};

        if (turtle_num == 1) return;  // do not have to check collisions between hunter and hunter :)

        auto hunter_position = turtles_positions[0];
        auto this_turtle_position = turtles_positions[turtle_num-1];

        /*
         * If hunter reached a victim
         */
        if (distance(hunter_position, this_turtle_position) < COLLISION_BOUNDARY) {

        }

    }

    std::vector<coord> turtles_positions; // for storing turtles positions

    rclcpp::Client<spawn_srv>::SharedPtr spawner;  // for calling /spawn service of the turtlesim
    rclcpp::Client<kill_srv>::SharedPtr killer;    // for calling /kill  service of the turtlesim

    std::vector<rclcpp::Subscription<turtle_pos>::SharedPtr> position_checkers;  // for retrieving positions of turtles
    unsigned int turtles_currently;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Game_Processing_Node>());
    rclcpp::shutdown();
    return 0;
}
