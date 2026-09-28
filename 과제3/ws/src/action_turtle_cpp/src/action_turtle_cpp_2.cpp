#include <cmath>
#include <memory>

#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "action_turtle/action/dist_turtle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace action_turtle_cpp
{
    // 이벤트 기반 액션 서버: 별도 스레드 없이 pose 콜백이 goal 을 한 스텝씩 진행한다.
    // 모든 콜백이 기본 콜백 그룹(MutuallyExclusive)에서 차례로 실행되므로 mutex 가 필요 없다.
    class ActionTurtleServer2 : public rclcpp::Node
    {
    public:
        using Turtle = action_turtle::action::DistTurtle;
        using GoalHandleTurtle = rclcpp_action::ServerGoalHandle<Turtle>;

        explicit ActionTurtleServer2(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("turtle_action_server", options)
        {
            this->action_server_ = rclcpp_action::create_server<Turtle>(
                this,
                "turtle",
                [this](const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Turtle::Goal> goal)
                { return this->handle_goal(uuid, goal); },
                [this](const std::shared_ptr<GoalHandleTurtle> goal_handle)
                { return this->handle_cancel(goal_handle); },
                [this](const std::shared_ptr<GoalHandleTurtle> goal_handle)
                { this->handle_accepted(goal_handle); });
            this->pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
            this->sub_ = this->create_subscription<turtlesim::msg::Pose>(
                "turtle1/pose", 10,
                [this](const turtlesim::msg::Pose &t_pose)
                { this->callback_turtle_pose(t_pose); });
        }

    private:
        rclcpp_action::Server<Turtle>::SharedPtr action_server_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;

        std::shared_ptr<GoalHandleTurtle> active_goal_;
        turtlesim::msg::Pose last_pose_;
        bool has_last_pose_ = false;
        double traveled_ = 0.0;

        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Turtle::Goal> goal)
        {
            (void)uuid;
            RCLCPP_INFO(this->get_logger(), "Received goal: linear_x %.2f, angular_z %.2f, dist %.2f",
                        goal->linear_x, goal->angular_z, goal->dist);

            // 한 번에 하나의 goal 만 처리한다.
            if (this->active_goal_)
            {
                RCLCPP_WARN(this->get_logger(), "Goal rejected: another goal is active");
                return rclcpp_action::GoalResponse::REJECT;
            }
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleTurtle> goal_handle)
        {
            (void)goal_handle;
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            // 실제 취소 처리는 다음 pose 콜백에서 is_canceling() 으로 확인한다.
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleTurtle> goal_handle)
        {
            // 스레드를 만들지 않고 goal 만 저장한 뒤 즉시 반환한다.
            this->active_goal_ = goal_handle;
            this->traveled_ = 0.0;
            this->has_last_pose_ = false;
            RCLCPP_INFO(this->get_logger(), "Executing goal");
        }

        // goal 진행의 한 스텝. turtlesim 의 pose 발행 주기(약 62.5Hz)로 호출된다.
        void callback_turtle_pose(const turtlesim::msg::Pose &t_pose)
        {
            if (!this->active_goal_)
            {
                return;
            }
            const auto goal = this->active_goal_->get_goal();

            if (this->has_last_pose_)
            {
                this->traveled_ += std::hypot(t_pose.x - this->last_pose_.x, t_pose.y - this->last_pose_.y);
            }
            this->last_pose_ = t_pose;
            this->has_last_pose_ = true;

            const double remained = goal->dist - this->traveled_;

            if (this->active_goal_->is_canceling() || remained <= 0.0)
            {
                this->pub_->publish(geometry_msgs::msg::Twist()); // 정지

                auto result = std::make_shared<Turtle::Result>();
                result->pos_x = t_pose.x;
                result->pos_y = t_pose.y;
                result->pox_theta = t_pose.theta;
                result->result_dist = this->traveled_;

                if (this->active_goal_->is_canceling())
                {
                    this->active_goal_->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "Goal canceled");
                }
                else
                {
                    this->active_goal_->succeed(result);
                    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                }
                this->active_goal_.reset();
                return;
            }

            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = goal->linear_x;
            cmd.angular.z = goal->angular_z;
            this->pub_->publish(cmd);

            auto feedback = std::make_shared<Turtle::Feedback>();
            feedback->remained_dist = remained;
            this->active_goal_->publish_feedback(feedback);
        }
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(action_turtle_cpp::ActionTurtleServer2)
