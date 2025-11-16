/****************************************************************************
 * PX4 Offboard mission with simple Mission State Machine
 *
 * - MissionPhase FSM:
 *   IDLE -> TAKEOFF -> HOVER -> DONE
 *
 *   나중에 FORWARD_TRANSITION / PATH_FLIGHT / BACK_TRANSITION / LAND
 *   로 확장할 수 있도록 enum과 틀만 미리 만들어 둠.
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
#include <cmath>   // std::fabs

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

// ─────────────────────────────────────
// 1. 미션 Phase 정의 (확장 버전)
// ─────────────────────────────────────
enum class MissionPhase {
	IDLE,               // 아무것도 안 함 (대기)
	TAKEOFF,            // 이륙 (목표 고도로 올라가는 중)
	HOVER,              // 목표 고도 호버
	FORWARD_TRANSITION, // VTOL -> 고정익 전환
	PATH_FLIGHT,        // 고정익 경로 비행
	BACK_TRANSITION,    // 고정익 -> VTOL 전환
	LAND,               // 착륙
	DONE                // 미션 종료
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

		vehicle_odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
			"/fmu/out/vehicle_odometry",
			rclcpp::SensorDataQoS(),  // ← 이걸로 바꾸기
			[this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
			{
				current_z_ = msg->position[2];
			}
		);

		offboard_setpoint_counter_ = 0;
		phase_ = MissionPhase::IDLE;
		phase_tick_ = 0;
		target_altitude_m_ = -5.0f; // NED 기준 -5m (위로 5m)

		// ▼ 100 ms마다 한 번씩 호출되는 메인 루프
		auto timer_callback = [this]() -> void {
			this->on_timer();
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

	// 미션 FSM 관련 상태
	MissionPhase phase_{MissionPhase::IDLE};
	uint64_t     phase_tick_{0};          // 해당 Phase에서 경과 tick 수 (100ms 단위)
	float        target_altitude_m_{-5.0f}; // NED z target (위로 갈수록 음수)

	// PX4 상태 변수(간단히 z만)
	float current_z_{0.0f};

	// 내부 함수들
	void on_timer();  // 타이머 콜백 본체

	// 상태 머신 관련
	void update_mission_state();
	void change_phase(MissionPhase new_phase);
	const char* phase_to_string(MissionPhase p);

	// PX4 퍼블리시 함수들
	void publish_offboard_control_mode();
	void publish_trajectory_setpoint_for_phase();
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
// 4. Phase 이름 문자열 변환
// ─────────────────────────────────────
const char* OffboardMissionNode::phase_to_string(MissionPhase p)
{
	switch (p) {
	case MissionPhase::IDLE:               return "IDLE";
	case MissionPhase::TAKEOFF:            return "TAKEOFF";
	case MissionPhase::HOVER:              return "HOVER";
	case MissionPhase::FORWARD_TRANSITION: return "FORWARD_TRANSITION";
	case MissionPhase::PATH_FLIGHT:        return "PATH_FLIGHT";
	case MissionPhase::BACK_TRANSITION:    return "BACK_TRANSITION";
	case MissionPhase::LAND:               return "LAND";
	case MissionPhase::DONE:               return "DONE";
	default:                               return "UNKNOWN";
	}
}

// ─────────────────────────────────────
// 5. Phase 변경 공용 함수
// ─────────────────────────────────────
void OffboardMissionNode::change_phase(MissionPhase new_phase)
{
	if (phase_ == new_phase) {
		return;
	}

	RCLCPP_INFO(
		this->get_logger(),
		"Phase change: %s -> %s",
		phase_to_string(phase_),
		phase_to_string(new_phase));

	phase_ = new_phase;
	phase_tick_ = 0; // 새 Phase 진입 시 tick 리셋
}

// ─────────────────────────────────────
// 6. 상태 전환 로직 (FSM 본체)
// ─────────────────────────────────────
void OffboardMissionNode::update_mission_state()
{
	switch (phase_) {

	case MissionPhase::IDLE:
		// ★ 첫 시작에서는 IDLE -> TAKEOFF으로 1회만 진입
		if (offboard_setpoint_counter_ == 0) {
			change_phase(MissionPhase::TAKEOFF);
		}
		break;

	case MissionPhase::TAKEOFF:
	{
		// current_z_ (예: -5.0f)와 target_altitude_m_ (예: -5.0f) 비교
		const float z_err = current_z_ - target_altitude_m_;
		if (std::fabs(z_err) < 0.3f) { // ±0.3m 이내면 목표 고도 도달로 간주
			change_phase(MissionPhase::HOVER);
		}
		break;
	}

	case MissionPhase::HOVER:
		// HOVER 상태를 10초 정도 유지 후 DONE으로 전환
		// 100ms * 100tick = 10초 근사
		if (phase_tick_ > 100) {
			change_phase(MissionPhase::DONE);
		}
		break;

	case MissionPhase::FORWARD_TRANSITION:
		// TODO: Phase 4에서 구현 예정
		break;

	case MissionPhase::PATH_FLIGHT:
		// TODO: Phase 4에서 구현 예정
		break;

	case MissionPhase::BACK_TRANSITION:
		// TODO: Phase 4에서 구현 예정
		break;

	case MissionPhase::LAND:
		// TODO: Phase 4에서 구현 예정
		break;

	case MissionPhase::DONE:
		// 미션 종료 상태: 필요하면 여기서 disarm, land 명령 등 수행
		break;
	}

	// 현재 Phase에서의 경과 tick 증가
	phase_tick_++;
}

// ─────────────────────────────────────
// 7. 타이머 콜백 본체
// ─────────────────────────────────────
void OffboardMissionNode::on_timer()
{
	// 1) 일정 횟수 setpoint 보낸 뒤 Offboard 모드 전환 + ARM
	if (offboard_setpoint_counter_ == 10) {
		// PX4 모드를 Offboard 로 변경
		publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
		// ARM
		arm();
	}

	// 2) 상태 머신 업데이트 (Phase 전환)
	update_mission_state();

	// 3) OffboardControlMode + TrajectorySetpoint 퍼블리시
	publish_offboard_control_mode();
	publish_trajectory_setpoint_for_phase();

	// 4) 카운터 증가
	offboard_setpoint_counter_++;
}

// ─────────────────────────────────────
// 8. OffboardControlMode 퍼블리시
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
// 9. Phase 기반 Trajectory Setpoint 생성
// ─────────────────────────────────────
void OffboardMissionNode::publish_trajectory_setpoint_for_phase()
{
	TrajectorySetpoint msg{};

	switch (phase_) {

	case MissionPhase::IDLE:
		// IDLE일 때는 굳이 의미있는 setpoint가 필요 없지만,
		// Offboard 유지용으로 (0,0,0)을 계속 보낼 수 있다.
		msg.position = {0.0f, 0.0f, 0.0f};
		msg.yaw = 0.0f;
		break;

	case MissionPhase::TAKEOFF:
		// 목표 고도까지 올라가는 중: 목표 고도 setpoint
		msg.position = {0.0f, 0.0f, target_altitude_m_};
		msg.yaw = -3.14f; // [-PI:PI]
		break;

	case MissionPhase::HOVER:
		// TAKEOFF와 같은 setpoint를 유지해도 실제로는 호버
		msg.position = {0.0f, 0.0f, target_altitude_m_};
		msg.yaw = -3.14f;
		break;

	case MissionPhase::FORWARD_TRANSITION:
		// TODO: VTOL -> 고정익 전환 시 setpoint 정의 예정
		msg.position = {0.0f, 0.0f, target_altitude_m_};
		msg.yaw = -3.14f;
		break;

	case MissionPhase::PATH_FLIGHT:
		// TODO: waypoint 기반 setpoint 생성 예정
		msg.position = {0.0f, 0.0f, target_altitude_m_};
		msg.yaw = -3.14f;
		break;

	case MissionPhase::BACK_TRANSITION:
		// TODO: 고정익 -> VTOL 전환 시 setpoint
		msg.position = {0.0f, 0.0f, target_altitude_m_};
		msg.yaw = -3.14f;
		break;

	case MissionPhase::LAND:
		// TODO: 착륙 시 내려가는 프로파일 등
		msg.position = {0.0f, 0.0f, -0.5f}; // 예: 지면 근처까지
		msg.yaw = -3.14f;
		break;

	case MissionPhase::DONE:
		// 미션 종료 후: 일단 목표 고도 유지 (필요시 바꿔도 됨)
		msg.position = {0.0f, 0.0f, target_altitude_m_};
		msg.yaw = -3.14f;
		break;
	}

	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

// ─────────────────────────────────────
// 10. VehicleCommand 퍼블리시
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
// 11. main 함수
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
