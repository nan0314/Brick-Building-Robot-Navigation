#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include "trect/srv/start.hpp"
#include <std_srvs/srv/empty.hpp>
#include <turtlesim/srv/set_pen.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>
#include <turtlesim/srv/teleport_relative.hpp>


///////////////////////////
// Helper Functions
///////////////////////////

constexpr double PI=3.14159265358979323846; // The value of pi


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

                w = request->w;
                h = request->h;

                initialize_turtlesim(request->x,request->y);

                response->success = true;
            };    
            service = this->create_service<trect::srv::Start>("start", start);

            abs_tel = this->create_client<turtlesim::srv::TeleportAbsolute>("turtle1/teleport_absolute"); 
            rel_tel = this->create_client<turtlesim::srv::TeleportRelative>("turtle1/teleport_relative"); 
            pen_color = this->create_client<turtlesim::srv::SetPen>("turtle1/set_pen"); 
            erase = this->create_client<std_srvs::srv::Empty>("/clear"); 

            timer = this->create_wall_timer(std::chrono::milliseconds(1000/freq), std::bind(&turtle_rect::timer_callback, this));
        }

    private:


        // turtle state machine states
        enum State{
            IDLE,
            BOTTOM,
            RIGHT,
            TOP,
            LEFT,
            ROTATE,
        };


        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr twist_sub;
        rclcpp::Service<trect::srv::Start>::SharedPtr service;
        rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr abs_tel;
        rclcpp::Client<turtlesim::srv::TeleportRelative>::SharedPtr rel_tel;
        rclcpp::Client<std_srvs::srv::Empty>::SharedPtr erase;
        rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen_color;
        rclcpp::TimerBase::SharedPtr timer;
        geometry_msgs::msg::Twist twist;
        turtlesim::msg::Pose pose;
        State state = IDLE;              // current state of turtle initialized to idle
        State prev;                      // previous state of turtle
        double max_xdot;
        double max_wdot;
        double target;
        int freq;
        double w;
        double h;

        


        void timer_callback()
        {
            // state machine to move turtle in rectangle
            switch(state){

                // idle case turtle does not move
                case IDLE:

                    twist.linear.x = 0;
                    twist.angular.z = 0;                
                    vel_pub->publish(twist);
                    break;

                // Bottom case turtle moves forward until reaching end of bottom line
                case BOTTOM:

                    // if target has not been reached move forward else switch to Rotate state
                    if(pose.x<target){
                        twist.linear.x = max_xdot;
                        vel_pub->publish(twist);
                    } else{
                        state = ROTATE;
                        prev = BOTTOM;
                        target = PI/2;  // sets target to pi/2 radians for Rotate case
                    }
                    break;

                // Right case turtle moves forward until reaching end of right line
                case RIGHT:
                    
                    // if target has not been reached move forward else switch to Rotate case
                    if(pose.y<target){
                        twist.linear.x = max_xdot;
                        vel_pub->publish(twist);
                    } else{
                        state = ROTATE;
                        prev = RIGHT;
                        target = PI;    // sets target to pi radians for Rotate case
                    }
                    break;

                // Top case turtle moves forward until reaching end of top line
                case TOP:

                    // if target has not been reached move forward else switch to Rotate case
                    if(pose.x>target){
                        twist.linear.x = max_xdot;
                        vel_pub->publish(twist);
                    } else{
                        state = ROTATE;
                        prev = TOP;
                        target = -PI/2; // Sets target to -pi/2 radians for rotate case
                    }
                    break;
                
                // Left case turtle moves forward until reaching end of left line
                case LEFT:

                    // if target has not been reached move forward else switch to Rotate case
                    if (pose.y>target){
                        twist.linear.x = max_xdot;
                        vel_pub->publish(twist);
                    } else{
                        state = ROTATE;
                        prev = LEFT;
                        target = 0;
                    }
                    break;

                // Rotate case turtle rotates until target is reached then switches to next case based
                // on the previous case
                case ROTATE:
                    if (fabs(pose.theta - target) > 0.01){
                        twist.linear.x = 0;
                        twist.angular.z = max_wdot;
                        vel_pub->publish(twist);
                    } else{
                        twist.angular.z = 0;
                        switch(prev){
                            case BOTTOM:
                                state = RIGHT;
                                target = pose.y + h;
                                break;
                            case RIGHT:
                                state = TOP;
                                target = pose.x - w;
                                break;
                            case TOP:
                                state = LEFT;
                                target = pose.y - h;
                                break;
                            case LEFT:
                                state = IDLE;
                                break; 
                            case ROTATE:
                                RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"),"Prev should not take on case 'ROTATE'");
                                break;
                            case IDLE:
                                RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"),"Prev should not take on case 'IDLE'");
                                break;
                        }
                    }
                    break;
            }
        }

        void pose_callback(turtlesim::msg::Pose::SharedPtr msg){
            pose = *msg;
        }

        void initialize_turtlesim(double x, double y){
            // initialize client requests
            auto init_pos = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
            auto pos = std::make_shared<turtlesim::srv::TeleportRelative::Request>();
            auto color = std::make_shared<turtlesim::srv::SetPen::Request>();
            auto empty = std::make_shared<std_srvs::srv::Empty::Request>();

            // set pen to white
            color->r = 255;
            color->g = 255;
            color->b = 255;
            color->width = 3;
            color->off = 0;
            pen_color->async_send_request(color);

            // place turtle at user specified position
            init_pos->x = x;
            init_pos->y = y;
            init_pos->theta = 0;
            abs_tel->async_send_request(init_pos);

            // clear the background
            erase->async_send_request(empty);

            // draw desired trajectory
            pos->linear = w;     
            rel_tel->async_send_request(pos);              // Draws bottom side of rectangle

            pos->linear = h;     
            pos->angular = PI/2;
            rel_tel->async_send_request(pos);              // Rotates turtle 90 degrees cc, draws right side

            pos->linear = w;     
            rel_tel->async_send_request(pos);              // Rotates turtle 90 degrees, draws top side

            pos->linear = h;
            rel_tel->async_send_request(pos);              // Rotates turtle 90 degrees, draws left side

            pos->linear = 0;     
            rel_tel->async_send_request(pos);              // Rotates turtle 90 degrees (returns to start position)

            // change pen color to black
            color->r = 0;
            color->g = 0;
            color->b = 0;
            color->width = 3;
            color->off = 0;
            pen_color->async_send_request(color);

            // cause robot to start following trajectory
            state = BOTTOM;
            target = x + w;     // create first target, end of bottom line
        }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<turtle_rect>());
    rclcpp::shutdown();
    return 0;
}