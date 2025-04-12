#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/motor_cmd.hpp"
#include "unitree_go/msg/bms_cmd.hpp"
#include "motor_crc.h"
#include "stand_action/action/stand_action.hpp"

class StandActionServer : public rclcpp::Node
{
public:
    using StandAction = stand_action::action::StandAction;
    using GoalHandleStandAction = rclcpp_action::ServerGoalHandle<StandAction>;

    StandActionServer() : Node("stand_action_server"), runing_time_(0.0)
    {
        // Crear servidor de acciones
        this->action_server_ = rclcpp_action::create_server<StandAction>(
            this,
            "stand_action",
            std::bind(&StandActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&StandActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&StandActionServer::handle_accepted, this, std::placeholders::_1));

        // Crear publisher para el tópico "/lowcmd"
        cmd_puber_ = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);
        init_cmd(); // Inicializar comandos
    }

private:
    rclcpp_action::Server<StandAction>::SharedPtr action_server_;
    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr cmd_puber_;
    unitree_go::msg::LowCmd low_cmd_;
    double stand_up_joint_pos_[12] = {0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763,
                                      0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763};
    double stand_down_joint_pos_[12] = {0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375, 0.0473455,
                                        1.22187, -2.44375, -0.0473455, 1.22187, -2.44375};
    double dt_ = 0.002;
    double runing_time_;
    bool active_;

    // Inicializar comandos
    void init_cmd()
    {
        for (int i = 0; i < 20; i++)
        {
            low_cmd_.motor_cmd[i].mode = 0x01; // Torque mode
            low_cmd_.motor_cmd[i].q = PosStopF;
            low_cmd_.motor_cmd[i].kp = 0;
            low_cmd_.motor_cmd[i].dq = VelStopF;
            low_cmd_.motor_cmd[i].kd = 0;
            low_cmd_.motor_cmd[i].tau = 0;
        }
    }

    // Manejar nueva meta
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const StandAction::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Recibida nueva meta: stand_up=%s", goal->stand_up ? "true" : "false");
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Manejar cancelación
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleStandAction> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Solicitud de cancelación recibida");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // Manejar meta aceptada
    void handle_accepted(const std::shared_ptr<GoalHandleStandAction> goal_handle)
    {
        std::thread{std::bind(&StandActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    // Ejecutar acción
    void execute(const std::shared_ptr<GoalHandleStandAction> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Ejecutando acción...");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<StandAction::Feedback>();
        auto result = std::make_shared<StandAction::Result>();

        runing_time_ = 0.0; // Reiniciar tiempo
        while (runing_time_ < 3.0)
        {
            // Calcular transición
            double phase = tanh(runing_time_ / 2.4);
            for (int i = 0; i < 12; i++)
            {
                if (goal->stand_up)
                {
                    low_cmd_.motor_cmd[i].q = phase * stand_up_joint_pos_[i] + (1 - phase) * stand_down_joint_pos_[i];
                }
                else
                {
                    low_cmd_.motor_cmd[i].q = phase * stand_down_joint_pos_[i] + (1 - phase) * stand_up_joint_pos_[i];
                }
                low_cmd_.motor_cmd[i].dq = 0;
                low_cmd_.motor_cmd[i].kp = 50.0;
                low_cmd_.motor_cmd[i].kd = 3.5;
                low_cmd_.motor_cmd[i].tau = 0;
            }

            get_crc(low_cmd_);
            cmd_puber_->publish(low_cmd_);

            // Actualizar progreso
            runing_time_ += dt_;
            feedback->progress = runing_time_ / 3.0;
            goal_handle->publish_feedback(feedback);
            rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(dt_ * 1000)));
        }

        // Finalizar acción
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Acción completada");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StandActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}