#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include "trect/srv/start.hpp"


///////////////////////////
// Helper Functions
///////////////////////////


/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */
class turtle_rect : public rclcpp::Node
{
    

    public:
        turtle_rect(): Node("turtle_rect")
        {
            using namespace std::chrono_literals;
            using std::placeholders::_1;


            // Get parameters
            this->declare_parameter("max_xdot");
            this->declare_parameter("max_wdot");
            this->declare_parameter("freq");

            this->get_parameter("max_xdot", max_xdot);
            this->get_parameter("max_wdot", max_wdot);
            this->get_parameter("freq", freq);

            // Set up publishers and subscribers
            vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
            twist_sub = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10, std::bind(&turtle_rect::pose_callback, this, _1));

            // Set up services and clients
            auto start = [this](const std::shared_ptr<trect::srv::Start::Request> request,
            std::shared_ptr<trect::srv::Start::Response> response) -> void {  

                x = request->x;
                y = request->y;
                w = request->w;
                h = requesr->h;
                response->success = true;
            };    
            service = this->create_service<trect::srv::Start>("start", start);

            timer = this->create_wall_timer(std::chrono::milliseconds(1000/freq), std::bind(&turtle_rect::timer_callback, this));
        }

    private:

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr twist_sub;
        rclcpp::Service<trect::srv::Start>::SharedPtr service;
        rclcpp::TimerBase::SharedPtr timer;
        geometry_msgs::msg::Twist twist;
        turtlesim::msg::Pose pose;
        double max_xdot;
        double max_wdot;
        int freq;
        double w;
        double h;
        double x;
        double y;


        void timer_callback()
        {
            twist.linear.x = max_xdot;
            twist.angular.z = max_wdot;
            vel_pub->publish(twist);
        }

        void pose_callback(turtlesim::msg::Pose::SharedPtr msg){
            pose = *msg;
        }       

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<turtle_rect>());
    rclcpp::shutdown();
    return 0;
}