#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <sys/socket.h> // socket, bind, listen, accept, recv, send, shutdown
#include <netinet/in.h> // sockaddr_in, htons, INADDR_ANY
#include <unistd.h>     // close
#include <nlohmann/json.hpp>

#include "action_turtle/action/dist_turtle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_srvs/srv/empty.hpp"
#include "turtlesim/action/rotate_absolute.hpp"
#include "turtlesim/msg/pose.hpp"

namespace action_turtle_cpp
{
    class ActionTurtleBridge2 : public rclcpp::Node
    {
    public:
        using DistTurtle = action_turtle::action::DistTurtle;
        using DistGoalHandle = rclcpp_action::ClientGoalHandle<DistTurtle>;
        using RotateAbsolute = turtlesim::action::RotateAbsolute;
        using RotateGoalHandle = rclcpp_action::ClientGoalHandle<RotateAbsolute>;
        using Empty = std_srvs::srv::Empty;

        explicit ActionTurtleBridge2(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
            : Node("turtle_action_bridge_2", options)
        {
            const int port = this->declare_parameter<int>("port", 9090);

            // 액션 클라이언트
            this->dist_client_ = rclcpp_action::create_client<DistTurtle>(this, "tutle_action");
            this->rotate_client_ = rclcpp_action::create_client<RotateAbsolute>(this, "turtle1/rotate_absolute");

            // 서비스 클라이언트
            this->reset_client_ = this->create_client<Empty>("reset");

            // 토픽 구독
            this->pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
                "turtle1/pose", 10,
                [this](const turtlesim::msg::Pose &pose)
                {
                    std::lock_guard<std::mutex> lk(this->pose_mtx_);
                    this->pose_ = pose;
                    this->has_pose_ = true;
                });

            // 소켓 준비가 끝난 뒤 스레드를 시작한다
            this->open_server_socket(port);
            this->server_thread_ = std::thread([this]()
                                               { this->server_loop(); });
            RCLCPP_INFO(this->get_logger(), "브리지 대기 중: TCP %d", port);
        }

        ~ActionTurtleBridge2() override
        {
            this->running_ = false;
            ::shutdown(this->listen_fd_, SHUT_RDWR); // 블록된 accept() 를 깨움
            {
                std::lock_guard<std::mutex> lk(this->client_mtx_);
                if (this->client_ >= 0)
                    ::shutdown(this->client_, SHUT_RDWR); // 블록된 recv() 를 깨움
            }
            if (this->server_thread_.joinable())
                this->server_thread_.join();
            ::close(this->listen_fd_);
        }

    private:
        static constexpr float kMoveDist = 2.0f;                 // 방향키 한 번의 이동 거리
        static constexpr double kQuarterTurn = 1.5; // 방향키 한 번의 회전량 (90°)

        rclcpp_action::Client<DistTurtle>::SharedPtr dist_client_;
        rclcpp_action::Client<RotateAbsolute>::SharedPtr rotate_client_;
        rclcpp::Client<Empty>::SharedPtr reset_client_;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;

        std::mutex pose_mtx_;
        turtlesim::msg::Pose pose_;
        bool has_pose_{false};

        std::mutex goal_mtx_;
        DistGoalHandle::SharedPtr dist_goal_; // 진행 중인 전진/후진 goal

        std::thread server_thread_;
        std::atomic<bool> running_{true};
        int listen_fd_{-1};
        int client_{-1};
        std::mutex client_mtx_;

        void open_server_socket(int port)
        {
            this->listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
            if (this->listen_fd_ < 0)
                throw std::runtime_error(std::string("socket 실패: ") + std::strerror(errno));

            int opt = 1;
            ::setsockopt(this->listen_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

            sockaddr_in addr{};
            addr.sin_family = AF_INET;
            addr.sin_addr.s_addr = INADDR_ANY;
            addr.sin_port = htons(static_cast<uint16_t>(port));

            if (::bind(this->listen_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0 ||
                ::listen(this->listen_fd_, 5) < 0)
            {
                std::string err = std::strerror(errno); // close() 가 errno 를 바꿀 수 있어 먼저 저장
                ::close(this->listen_fd_);              // 생성자 예외 시 소멸자가 호출되지 않으므로 직접 닫음
                throw std::runtime_error("소켓 준비 실패 (port " + std::to_string(port) + "): " + err);
            }
        }

        void server_loop()
        {
            char buf[1024];
            while (this->running_)
            {
                int fd = ::accept(this->listen_fd_, nullptr, nullptr);
                if (fd < 0)
                {
                    if (!this->running_)
                        break;
                    continue;
                }
                {
                    std::lock_guard<std::mutex> lk(this->client_mtx_);
                    this->client_ = fd;
                }
                RCLCPP_INFO(this->get_logger(), "클라이언트 접속");

                std::string pending;
                while (this->running_)
                {
                    ssize_t n = ::recv(fd, buf, sizeof(buf), 0); // 잠금 없이 대기
                    if (n <= 0)
                        break; // 0: 정상 종료, -1: 오류

                    pending.append(buf, n);
                    size_t pos;
                    while ((pos = pending.find('\n')) != std::string::npos)
                    {
                        this->handle_message(pending.substr(0, pos));
                        pending.erase(0, pos + 1);
                    }
                }

                {
                    std::lock_guard<std::mutex> lk(this->client_mtx_);
                    ::close(this->client_);
                    this->client_ = -1;
                }
                RCLCPP_INFO(this->get_logger(), "클라이언트 종료");
            }
        }

        void send_to_client(const nlohmann::json &j)
        {
            std::string line = j.dump() + "\n";
            std::lock_guard<std::mutex> lk(this->client_mtx_);
            if (this->client_ < 0)
                return;
            ::send(this->client_, line.data(), line.size(), MSG_NOSIGNAL);
        }

        void send_error(const std::string &code, const std::string &message)
        {
            RCLCPP_WARN(this->get_logger(), "%s: %s", code.c_str(), message.c_str());
            this->send_to_client({{"type", "error"}, {"code", code}, {"message", message}});
        }


        void handle_message(const std::string &line)
        {
            auto j = nlohmann::json::parse(line, nullptr, false);
            if (j.is_discarded())
            {
                this->send_error("invalid_json", "JSON 파싱 실패");
                return;
            }

            try
            {
                const std::string type = j.value("type", "");
                if (type == "move")
                    this->handle_move(j);
                else if (type == "goal")
                    this->send_dist_goal(j.value("linear_x", 0.0f), j.value("angular_z", 0.0f), j.value("dist", kMoveDist));
                else if (type == "reset")
                    this->send_reset();
                else if (type == "pose")
                    this->send_pose();
                else
                    this->send_error("unknown_type", "알 수 없는 type: " + type);
            }
            catch (const nlohmann::json::exception &e) // 필드 타입이 틀리거나 객체가 아닌 JSON
            {
                this->send_error("invalid_field", e.what());
            }
        }

        void handle_move(const nlohmann::json &j)
        {
            const float linear_x = j.value("linear_x", 0.0f);
            const float angular_z = j.value("angular_z", 0.0f);

            if (linear_x != 0.0f)
                this->send_dist_goal(linear_x, 0.0f, kMoveDist); // ↑ ↓
            else if (angular_z != 0.0f)
                this->send_rotate_goal(angular_z > 0.0f ? kQuarterTurn : -kQuarterTurn); // ← →
            else
                this->send_error("invalid_field", "linear_x 또는 angular_z 가 0 이 아니어야 함");
        }

        void send_dist_goal(float x, float z, float dist)
        {
            if (!this->dist_client_->action_server_is_ready())
            {
                this->send_error("server_not_ready", "tutle_action 서버 없음");
                return;
            }
            this->cancel_dist_goal(); // 이전 이동이 남아 있으면 멈춤

            DistTurtle::Goal goal;
            goal.linear_x = x;
            goal.angular_z = z;
            goal.dist = dist;

            auto opt = rclcpp_action::Client<DistTurtle>::SendGoalOptions();
            opt.goal_response_callback = [this](const DistGoalHandle::SharedPtr &handle)
            {
                {
                    std::lock_guard<std::mutex> lk(this->goal_mtx_);
                    this->dist_goal_ = handle;
                }
                this->send_to_client({{"type", "goal_response"}, {"action", "move"}, {"accepted", handle != nullptr}});
            };
            opt.feedback_callback = [this](DistGoalHandle::SharedPtr, const std::shared_ptr<const DistTurtle::Feedback> fb)
            {
                this->send_to_client({{"type", "feedback"}, {"action", "move"}, {"remained_dist", fb->remained_dist}});
            };
            opt.result_callback = [this](const DistGoalHandle::WrappedResult &r)
            {
                {
                    std::lock_guard<std::mutex> lk(this->goal_mtx_);
                    if (this->dist_goal_ && this->dist_goal_->get_goal_id() == r.goal_id)
                        this->dist_goal_.reset();
                }
                this->send_to_client({
                    {"type", "result"},
                    {"action", "move"},
                    {"status", status_string(r.code)},
                    {"pos_x", r.result->pos_x},
                    {"pos_y", r.result->pos_y},
                    {"pos_theta", r.result->pox_theta},
                    {"result_dist", r.result->result_dist},
                });
            };
            this->dist_client_->async_send_goal(goal, opt);
        }

        void cancel_dist_goal()
        {
            DistGoalHandle::SharedPtr handle;
            {
                std::lock_guard<std::mutex> lk(this->goal_mtx_);
                handle = this->dist_goal_;
                this->dist_goal_.reset();
            }
            if (!handle)
                return;
            try
            {
                this->dist_client_->async_cancel_goal(handle);
            }
            catch (const rclcpp_action::exceptions::UnknownGoalHandleError &)
            {
                // 그사이 이미 끝난 goal — 취소할 것이 없음
            }
        }

        void send_rotate_goal(float theta2)
        {
            if (!this->rotate_client_->action_server_is_ready())
            {
                this->send_error("server_not_ready", "turtle1/rotate_absolute 서버 없음 (turtlesim 실행 확인)");
                return;
            }

            float theta;
            {
                std::lock_guard<std::mutex> lk(this->pose_mtx_);
                if (!this->has_pose_)
                {
                    this->send_error("no_pose", "아직 pose 를 받지 못함");
                    return;
                }
                theta = this->pose_.theta;
            }
            this->cancel_dist_goal(); // 이동 중이면 멈추고 회전

            RotateAbsolute::Goal goal;
            goal.theta = static_cast<float>(std::atan2(std::sin(theta + theta2), std::cos(theta + theta2))); // -π ~ π 로 정규화

            auto opt = rclcpp_action::Client<RotateAbsolute>::SendGoalOptions();
            opt.goal_response_callback = [this](const RotateGoalHandle::SharedPtr &handle)
            {
                this->send_to_client({{"type", "goal_response"}, {"action", "rotate"}, {"accepted", handle != nullptr}});
            };
            opt.feedback_callback = [this](RotateGoalHandle::SharedPtr, const std::shared_ptr<const RotateAbsolute::Feedback> fb)
            {
                this->send_to_client({{"type", "feedback"}, {"action", "rotate"}, {"remaining", fb->remaining}});
            };
            opt.result_callback = [this](const RotateGoalHandle::WrappedResult &r)
            {
                this->send_to_client({
                    {"type", "result"},
                    {"action", "rotate"},
                    {"status", status_string(r.code)},
                    {"delta", r.result->delta},
                });
            };
            this->rotate_client_->async_send_goal(goal, opt);
        }


        void send_reset()
        {
            if (!this->reset_client_->service_is_ready())
            {
                this->send_error("server_not_ready", "/reset 서비스 없음 (turtlesim 실행 확인)");
                return;
            }
            this->cancel_dist_goal(); // 이동 중이면 멈추고 초기화

            this->reset_client_->async_send_request(
                std::make_shared<Empty::Request>(),
                [this](rclcpp::Client<Empty>::SharedFuture) // 응답 콜백은 spin 스레드에서 실행
                { this->send_to_client({{"type", "reset"}, {"ok", true}}); });
        }


        void send_pose()
        {
            turtlesim::msg::Pose p;
            {
                std::lock_guard<std::mutex> lk(this->pose_mtx_);
                if (!this->has_pose_)
                {
                    this->send_error("no_pose", "아직 pose 를 받지 못함");
                    return;
                }
                p = this->pose_;
            }
            this->send_to_client({{"type", "pose"}, {"x", p.x}, {"y", p.y}, {"theta", p.theta}});
        }

        static std::string status_string(rclcpp_action::ResultCode code)
        {
            switch (code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
                return "succeeded";
            case rclcpp_action::ResultCode::ABORTED:
                return "aborted";
            case rclcpp_action::ResultCode::CANCELED:
                return "canceled";
            default:
                return "unknown";
            }
        }
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(action_turtle_cpp::ActionTurtleBridge2)
