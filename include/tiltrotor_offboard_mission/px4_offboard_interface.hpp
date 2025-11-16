#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

class VehicleCommandClient
{
public:
    explicit VehicleCommandClient(rclcpp::Node & node)
    {
        vehicle_command_pub_ =
            node.create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
        node_ = &node;
    }

    void send_command(uint16_t command,
                      float param1 = 0.0f,
                      float param2 = 0.0f)
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;

        vehicle_command_pub_->publish(msg);
    }

    void arm()
    {
        send_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
        RCLCPP_INFO(node_->get_logger(), "Arm command sent");
    }

    void disarm()
    {
        send_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
        RCLCPP_INFO(node_->get_logger(), "Disarm command sent");
    }

    void set_offboard_mode()
    {
        // PX4 문서: main_mode = 1, sub_mode = 6 → Offboard
        send_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
        RCLCPP_INFO(node_->get_logger(), "Offboard mode command sent");
    }

private:
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Node * node_{nullptr};
};

class PX4OffboardInterface
{
public:
    explicit PX4OffboardInterface(rclcpp::Node & node)
    : cmd_client_(node)
    {
        offboard_control_mode_pub_ =
            node.create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_pub_ =
            node.create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        node_ = &node;
    }

    VehicleCommandClient & command_client() { return cmd_client_; }

    // position 기반 OffboardControlMode
    void publish_position_control_mode()
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_pub_->publish(msg);
    }

    // 간단 position+yaw setpoint
    void publish_position_setpoint(float x, float y, float z, float yaw)
    {
        px4_msgs::msg::TrajectorySetpoint sp{};
        sp.position = {x, y, z};
        sp.yaw = yaw;
        sp.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_pub_->publish(sp);
    }

    // TrajectorySetpoint 전체를 직접 보낼 때
    void publish_trajectory(const px4_msgs::msg::TrajectorySetpoint & sp)
    {
        px4_msgs::msg::TrajectorySetpoint msg = sp;
        msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_pub_->publish(msg);
    }

private:
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    VehicleCommandClient cmd_client_;
    rclcpp::Node * node_{nullptr};
};

class HoverController
{
public:
    HoverController(float tgt_alt_m = -5.0f, float tgt_yaw_rad = -3.14f)
    : target_altitude_m_(tgt_alt_m),
      target_yaw_rad_(tgt_yaw_rad)
    {}

    void set_target_altitude(float alt_m) { target_altitude_m_ = alt_m; }
    void set_target_yaw(float yaw_rad)    { target_yaw_rad_ = yaw_rad; }

    px4_msgs::msg::TrajectorySetpoint create_setpoint() const
    {
        px4_msgs::msg::TrajectorySetpoint sp{};
        sp.position = {0.0f, 0.0f, target_altitude_m_};
        sp.yaw = target_yaw_rad_;
        return sp;
    }

    float target_altitude() const { return target_altitude_m_; }

private:
    float target_altitude_m_{-5.0f}; // NED: 위로 갈수록 음수
    float target_yaw_rad_{-3.14f};
};
