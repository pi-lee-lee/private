#include <functional>
#include <memory>
#include <thread>

#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "action_turtle/action/dist_turtle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace action_turtle_cpp
{
    class ActionTurtleServer : public rclcpp::Node
    {
    public:
        using Turtle = action_turtle::action::DistTurtle;
        using GoalHandleTurtle = rclcpp_action::ServerGoalHandle<Turtle>;

        explicit ActionTurtleServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("turtle_action_server", options)
        {
            using namespace std::placeholders;
            this->action_server_ = rclcpp_action::create_server<Turtle>(
                this,
                "tutle_action",
                [this](const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Turtle::Goal> goal)
                { return this->handle_goal(uuid, goal); },
                [this](const std::shared_ptr<GoalHandleTurtle> goal_handle)
                { return this->handle_cancel(goal_handle); },
                [this](const std::shared_ptr<GoalHandleTurtle> goal_handle)
                { return this->handle_accepted(goal_handle); });
            this->pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
            this->sub_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10, [this](const turtlesim::msg::Pose t_pose)
                                                                         { this->callback_turtle_pose(t_pose); });
        }

    private:
        rclcpp_action::Server<Turtle>::SharedPtr action_server_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;

        std::mutex pose_mtx_;
        turtlesim::msg::Pose pose_;

        void callback_turtle_pose(const turtlesim::msg::Pose pose)
        {
            std::lock_guard<std::mutex> lk(pose_mtx_);
            pose_ = pose;
        }

        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Turtle::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "Received goal request with order %.2f, %.2f, %.2f", goal->linear_x, goal->angular_z, goal->dist);
            (void)uuid;

            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleTurtle> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleTurtle> goal_handle)
        {
            using namespace std::placeholders;
            // this needs to return quickly to avoid blocking the executor, so spin up a new thread
            std::thread{[this](const std::shared_ptr<GoalHandleTurtle> goal_handle){ this->execute(goal_handle); }, goal_handle}
                .detach();
            // std::thread{std::bind(&ActionTurtleServer::execute, this, _1), goal_handle}.detach();
        }

        void execute(const std::shared_ptr<GoalHandleTurtle> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Executing goal");
            rclcpp::Rate loop_rate(0.5);

            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<Turtle::Feedback>();
            feedback->remained_dist = goal->dist;
            auto result = std::make_shared<Turtle::Result>();

            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = goal->linear_x;
            cmd.angular.z = goal->angular_z;

            while (rclcpp::ok()) //rclcpp::ok() 는 현재 인스턴스가 살아있음을 의미한다. 
            {
                if (goal_handle->is_canceling())
                {
                    std::lock_guard<std::mutex> lk(pose_mtx_);
                    result->pos_x = this->pose_.x;
                    result->pos_y = this->pose_.y;
                    result->pox_theta = this->pose_.theta;
                    result->result_dist = goal->dist - feedback->remained_dist;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "Goal canceled");
                    return;
                }

                float pose_x;
                float pose_y;
                {
                    std::lock_guard<std::mutex> lk(pose_mtx_);
                    pose_x = this->pose_.x;
                    pose_y = this->pose_.y;
                }
                
                this->pub_->publish(cmd);
                loop_rate.sleep();

                {
                    std::lock_guard<std::mutex> lk(pose_mtx_);
                    feedback->remained_dist -= std::hypot(pose_x - this->pose_.x, pose_y - this->pose_.y);
                }

                if (feedback->remained_dist > 0)
                {
                    goal_handle->publish_feedback(feedback);
                    RCLCPP_INFO(this->get_logger(), "Publish feedback");
                }else
                {
                    break;
                }
            }

            if (rclcpp::ok())
            {                       
                auto result = std::make_shared<Turtle::Result>();
                std::lock_guard<std::mutex> lk(pose_mtx_);
                result->pos_x = this->pose_.x;
                result->pos_y = this->pose_.y;
                result->pox_theta = this->pose_.theta;
                result->result_dist = goal->dist;
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            }
            // Check if there is a cancel request
        }
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(action_turtle_cpp::ActionTurtleServer); // 노드 등록 현재 액션은 라이브러리형태로 제작되어 다른 코드에서도 수행가능하다. 노드에 등록해야 rclcpp가 인식가능하다. 