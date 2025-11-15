/****************************************************************************
 * PX4 Offboard mission skeleton (refactored from example)
 ****************************************************************************/

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <atomic>
#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

// ─────────────────────────────────────
// 1. 간단한 미션 Phase 정의
// ─────────────────────────────────────
enum class MissionPhase {
	IDLE,           // 아직 아무것도 안 하는 상태
	TAKEOFF_HOVER,  // 지금 예제처럼 (0,0,-5) 호버
	DONE            // 나중에 LAND 등 끝났을 때 쓸 상태
};

// ─────────────────────────────────────
// 2. 메인 노드 클래스
// ─────────────────────────────────────
class OffboardMissionNode : public rclcpp::Node
{
public:
	OffboardMissionNode()
	: Node("offboard_mission_node")
	{
		// ▼ PX4로 보내는 Publisher들
		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ =
			this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

		// ▼ PX4에서 받는 상태 (Odometry) 구독 — z값 정도만 저장
		vehicle_odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
			"/fmu/out/vehicle_odometry",
			10,
			[this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
			{
				current_z_ = msg->position[2];  // NED: z (Down 방향, 위로 갈수록 음수)
				// RCLCPP_INFO(this->get_logger(), "z = %.2f", current_z_);
			}
		);

		offboard_setpoint_counter_ = 0;
		phase_ = MissionPhase::IDLE;

		// ▼ 100 ms마다 한 번씩 호출되는 메인 루프
		auto timer_callback = [this]() -> void {

			// 1) 최초 1회 IDLE → TAKEOFF_HOVER 진입
			if (phase_ == MissionPhase::IDLE && offboard_setpoint_counter_ == 0) {
				phase_ = MissionPhase::TAKEOFF_HOVER;
				RCLCPP_INFO(this->get_logger(), "Phase changed: IDLE -> TAKEOFF_HOVER");
			}

			// 2) 일정 횟수 setpoint 보낸 뒤 Offboard 모드 전환 + ARM
			if (offboard_setpoint_counter_ == 10) {
				// PX4 모드를 Offboard 로 변경
				publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				// ARM
				arm();
			}

			// 3) OffboardControlMode + TrajectorySetpoint 퍼블리시
			publish_offboard_control_mode();
			publish_trajectory_setpoint_for_phase();  // Phase 기반 setpoint

			// 4) 카운터 증가 (지금은 테스트용으로 11까지만 증가)
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}

			// TODO: 나중에 current_z_ 값 보고
			//       목표 고도 도달 시 다음 Phase로 전환 같은 로직 넣으면 됨.
		};

		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

	void arm();
	void disarm();

private:
	// 타이머
	rclcpp::TimerBase::SharedPtr timer_;

	// Publisher들
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	// Subscriber들
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;

	// 내부 상태
	std::atomic<uint64_t> timestamp_{0};
	uint64_t offboard_setpoint_counter_{0};

	MissionPhase phase_{MissionPhase::IDLE};

	// PX4 상태 변수(간단히 z만)
	float current_z_{0.0f};

	// 내부 함수들
	void publish_offboard_control_mode();
	void publish_trajectory_setpoint_for_phase(); // ★ 기존 publish_trajectory_setpoint를 대체
	void publish_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);
};

// ─────────────────────────────────────
// 3. Vehicle ARM / DISARM
// ─────────────────────────────────────
void OffboardMissionNode::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
	RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

void OffboardMissionNode::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
	RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

// ─────────────────────────────────────
// 4. OffboardControlMode 퍼블리시
// ─────────────────────────────────────
void OffboardMissionNode::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

// ─────────────────────────────────────
// 5. Phase 기반 Trajectory Setpoint 생성
// ─────────────────────────────────────
void OffboardMissionNode::publish_trajectory_setpoint_for_phase()
{
	TrajectorySetpoint msg{};

	switch (phase_) {
	case MissionPhase::TAKEOFF_HOVER:
		// ★ 기존 예제와 동일: (0,0,-5)에서 호버
		msg.position = {0.0f, 0.0f, -5.0f};
		msg.yaw = -3.14f; // [-PI:PI]
		break;

	case MissionPhase::IDLE:
		// IDLE일 때는 아직 의미 있는 setpoint를 안 보내도 되지만,
		// Offboard 유지를 위해 (0,0,0) 유지로 보낼 수도 있음.
		msg.position = {0.0f, 0.0f, 0.0f};
		msg.yaw = 0.0f;
		break;

	case MissionPhase::DONE:
		// DONE 상태에서의 처리 (나중에 LAND 후 등)
		// 일단은 간단히 현재와 비슷하게 둠.
		msg.position = {0.0f, 0.0f, -5.0f};
		msg.yaw = 0.0f;
		break;
	}

	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

// ─────────────────────────────────────
// 6. VehicleCommand 퍼블리시
// ─────────────────────────────────────
void OffboardMissionNode::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

// ─────────────────────────────────────
// 7. main 함수
// ─────────────────────────────────────
int main(int argc, char *argv[])
{
	std::cout << "Starting offboard mission node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardMissionNode>());
	rclcpp::shutdown();

	return 0;
}
