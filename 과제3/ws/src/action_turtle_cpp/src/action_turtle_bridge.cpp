#include <functional>
#include <memory>
#include <thread>
#include <string>

#include <sys/socket.h> // socket, bind, listen, accept, recv, send, shutdown
#include <netinet/in.h> // sockaddr_in, htons, INADDR_ANY
#include <unistd.h>     // close
#include <nlohmann/json.hpp>

#include "turtlesim/msg/pose.hpp"

#include "action_turtle/action/dist_turtle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace action_turtle_cpp
{
    class ActionTurtleBridge : public rclcpp::Node
    {
    public:
        using Turtle = action_turtle::action::DistTurtle;
        using GoalHandleTurtle = rclcpp_action::ClientGoalHandle<Turtle>;

        explicit ActionTurtleBridge(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("turtle_action_bridge", options)
        {
            this->action_client_ = rclcpp_action::create_client<Turtle>(this, "tutle_action");
            this->sub_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10, [this](const turtlesim::msg::Pose t_pose)
                                                                         { this->callback_turtle_pose(t_pose); });

            this->listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);

            int opt = 1;
            ::setsockopt(this->listen_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

            sockaddr_in addr{};
            addr.sin_family = AF_INET;
            addr.sin_addr.s_addr = INADDR_ANY;
            addr.sin_port = htons(9090);

            if (::bind(this->listen_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0 || ::listen(this->listen_fd_, 5) < 0)
            {
                std::string err = std::strerror(errno); // close() 가 errno 를 바꿀 수 있어 먼저 저장
                ::close(this->listen_fd_);
                throw std::runtime_error("소켓 준비 실패: " + err);
            }
            server_thread_ = std::thread([this]()
                                         { this->server_loop(); }); // 클라이언트 접속 처리
        }

        ~ActionTurtleBridge() override
        {
            running_ = false;
            ::shutdown(this->listen_fd_, SHUT_RDWR);
            {
                std::lock_guard<std::mutex> lk(this->client_mtx_);
                if (this->client_ >= 0)
                    ::shutdown(this->client_, SHUT_RDWR); // 블록된 recv() 를 깨움
            }
            if (server_thread_.joinable())
                server_thread_.join();
            ::close(this->listen_fd_);
        }

    private:
        rclcpp_action::Client<Turtle>::SharedPtr action_client_;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;

        std::mutex pose_mtx_;
        turtlesim::msg::Pose pose_;

        std::thread server_thread_;
        std::atomic<bool> running_{true};
        int listen_fd_{-1};
        int client_{-1};
        std::mutex client_mtx_;

        void callback_turtle_pose(const turtlesim::msg::Pose pose)
        {
            std::lock_guard<std::mutex> lk(pose_mtx_);
            pose_ = pose;
        }

        void send_goal(float x, float z, float dist)
        {

            if (!this->action_client_->wait_for_action_server(std::chrono::seconds(3)))
            {
                RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
                this->send_to_client({{"type", "error"}, {"code", "server_not_ready"}, {"message", "액션 서버 없음"}});
                return;
            }

            Turtle::Goal goal;
            goal.linear_x = x;
            goal.angular_z = z;
            goal.dist = dist;
            auto send_option = rclcpp_action::Client<Turtle>::SendGoalOptions();
            send_option.goal_response_callback = [this](const GoalHandleTurtle::SharedPtr &goal_handle)
            { this->goal_response_callback(goal_handle); };
            send_option.feedback_callback = [this](const GoalHandleTurtle::SharedPtr &goal_handle, const std::shared_ptr<const Turtle::Feedback> feedback)
            { this->feedback_callback(goal_handle, feedback); };
            send_option.result_callback = [this](const GoalHandleTurtle::WrappedResult &result)
            { this->result_callback(result); };
            this->action_client_->async_send_goal(goal, send_option);
        }

        void goal_response_callback(const GoalHandleTurtle::SharedPtr &goal_handle)
        {
            if (!goal_handle)
            {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
                this->send_to_client({{"type", "goal_response"}, {"accepted", false}});
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
                this->send_to_client({{"type", "goal_response"}, {"accepted", true}});
            }
        }

        void feedback_callback(const GoalHandleTurtle::SharedPtr &goal_handle, const std::shared_ptr<const Turtle::Feedback> feedback)
        {
            (void)goal_handle; // 쓰지 않는 인자 처리 매개변수중 사용하지 않는 인지는 unused 방지용으로 void 처리한다.
            RCLCPP_INFO(this->get_logger(), "Feedback: remained_dist=%.2f", feedback->remained_dist);
            this->send_to_client({{"type", "feedback"}, {"remained_dist", feedback->remained_dist}});
        }
        void result_callback(const GoalHandleTurtle::WrappedResult &result)
        {
            std::string status;
            switch (result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
                status = "succeeded";
                break;
            case rclcpp_action::ResultCode::ABORTED:
                status = "aborted";
                break;
            case rclcpp_action::ResultCode::CANCELED:
                status = "canceled";
                break;
            default:
                status = "unknown";
                break;
            }
            RCLCPP_INFO(this->get_logger(), "Result: %s", status.c_str());

            this->send_to_client({
                {"type", "result"},
                {"status", status},
                {"pos_x", result.result->pos_x},
                {"pos_y", result.result->pos_y},
                {"pos_theta", result.result->pox_theta},
                {"result_dist", result.result->result_dist},
            });
        }

        void send_pose()
        {
            turtlesim::msg::Pose p;
            {
                std::lock_guard<std::mutex> lk(pose_mtx_); // 구독 콜백(spin 스레드)과 공유
                p = pose_;
            }
            this->send_to_client({{"type", "pose"}, {"x", p.x}, {"y", p.y}, {"theta", p.theta}});
        }

        void send_to_client(const nlohmann::json &j)
        {
            std::string line = j.dump() + "\n"; // 한 줄 JSON + 구분자
            std::lock_guard<std::mutex> lk(client_mtx_);
            if (client_ < 0)
                return; // 접속한 클라이언트 없음
            ::send(client_, line.data(), line.size(), MSG_NOSIGNAL);
        }

        void server_loop()
        {
            while (running_)
            {
                char buf[1024];
                std::string pending;

                int fd = ::accept(listen_fd_, nullptr, nullptr);
                if (fd < 0)
                {
                    if (!running_)
                        break;
                    continue;
                }

                {
                    std::lock_guard<std::mutex> lk(client_mtx_);
                    client_ = fd;
                }

                while (running_)
                {
                    ssize_t n = ::recv(fd, buf, sizeof(buf), 0); // 잠금 없이 대기
                    if (n <= 0)
                        break;

                    pending.append(buf, n);
                    size_t pos;
                    while ((pos = pending.find('\n')) != std::string::npos)
                    { // 완성된 줄마다
                        handle_message(pending.substr(0, pos));
                        pending.erase(0, pos + 1);
                    }
                }

                {
                    std::lock_guard<std::mutex> lk(client_mtx_);
                    ::close(client_);
                    client_ = -1; // 이후 콜백은 보내지 않음
                }
            }
        }

        void handle_message(const std::string &line)
        {
            auto j = nlohmann::json::parse(line, nullptr, false); // 실패해도 예외 대신 discarded
            if (j.is_discarded())
            {
                RCLCPP_WARN(get_logger(), "잘못된 JSON: %s", line.c_str());
                this->send_to_client({{"type", "error"},
                                      {"code", "invalid_json"},
                                      {"message", "JSON 파싱 실패"}});
                return;
            }

            const std::string type = j.value("type", "");
            if (type == "goal")
            {
                send_goal(j.value("linear_x", 0.0f), j.value("angular_z", 0.0f), j.value("dist", 0.0f));
            }
            else if (type == "pose")
            {
                send_pose();
            }
            else
            {
                RCLCPP_WARN(get_logger(), "알 수 없는 type: %s", type.c_str());
                this->send_to_client({
                    {"type", "error"},
                    {"code", "unknown_type"},
                    {"message", "알 수 없는 type: " + type},
                    {"received", j},
                });
            }
        }
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(action_turtle_cpp::ActionTurtleBridge)