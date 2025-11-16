/****************************************************************************
 * PX4 Offboard mission with Mission State Machine + PX4 interface layer
 *
 * - MissionPhase FSM:
 *   IDLE -> TAKEOFF -> HOVER -> DONE
 *
 * - Phase 3 목표:
 *   * VehicleStatus Subscriber 추가
 *   * OffboardPublisher 모듈화 (PX4OffboardInterface)
 *   * VehicleCommandClient 구현
 *   * Hover 로직 분리 (HoverController)
 ****************************************************************************/

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

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
// 0. 미션 Phase 정의
// ─────────────────────────────────────
enum class MissionPhase {
	IDLE,
	TAKEOFF,
	HOVER,
	FORWARD_TRANSITION,
	PATH_FLIGHT,
	BACK_TRANSITION,
	LAND,
	DONE
};

// ─────────────────────────────────────
// 1. VehicleCommandClient: VehicleCommand 전용 헬퍼
// ─────────────────────────────────────
class VehicleCommandClient
{
public:
	explicit VehicleCommandClient(rclcpp::Node & node)
	{
		vehicle_command_pub_ =
			node.create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
		node_ = &node;
	}

	void send_command(uint16_t command,
	                  float param1 = 0.0f,
	                  float param2 = 0.0f)
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
		msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;

		vehicle_command_pub_->publish(msg);
	}

	// 편의 함수들
	void arm()
	{
		send_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
		RCLCPP_INFO(node_->get_logger(), "Arm command sent");
	}

	void disarm()
	{
		send_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
		RCLCPP_INFO(node_->get_logger(), "Disarm command sent");
	}

	void set_offboard_mode()
	{
		// PX4 문서 기준: main_mode = 1, sub_mode = 6 → Offboard
		send_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
		RCLCPP_INFO(node_->get_logger(), "Offboard mode command sent");
	}

private:
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;
	rclcpp::Node * node_{nullptr};
};

// ─────────────────────────────────────
// 2. PX4OffboardInterface: OffboardControlMode + TrajectorySetpoint
// ─────────────────────────────────────
class PX4OffboardInterface
{
public:
	explicit PX4OffboardInterface(rclcpp::Node & node)
	: cmd_client_(node)
	{
		offboard_control_mode_pub_ =
			node.create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_pub_ =
			node.create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		node_ = &node;
	}

	VehicleCommandClient & command_client() { return cmd_client_; }

	// position 모드 OffboardControlMode 보내기
	void publish_position_control_mode()
	{
		OffboardControlMode msg{};
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
		TrajectorySetpoint sp{};
		sp.position = {x, y, z};
		sp.yaw = yaw;
		sp.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
		trajectory_setpoint_pub_->publish(sp);
	}

	// TrajectorySetpoint 전체를 넘겨서 퍼블리시
	void publish_trajectory(const TrajectorySetpoint & sp)
	{
		TrajectorySetpoint msg = sp;
		msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
		trajectory_setpoint_pub_->publish(msg);
	}

private:
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
	VehicleCommandClient cmd_client_;
	rclcpp::Node * node_{nullptr};
};

// ─────────────────────────────────────
// 3. HoverController: Hover setpoint 생성 전용
// ─────────────────────────────────────
class HoverController
{
public:
	HoverController(float tgt_alt_m = -5.0f, float tgt_yaw_rad = -3.14f)
	: target_altitude_m_(tgt_alt_m),
	  target_yaw_rad_(tgt_yaw_rad)
	{}

	void set_target_altitude(float alt_m) { target_altitude_m_ = alt_m; }
	void set_target_yaw(float yaw_rad) { target_yaw_rad_ = yaw_rad; }

	// 현재는 (0,0,z) 고정 호버 — 나중에 x,y도 미션에 맞게 확장 가능
	TrajectorySetpoint create_setpoint() const
	{
		TrajectorySetpoint sp{};
		sp.position = {0.0f, 0.0f, target_altitude_m_};
		sp.yaw = target_yaw_rad_;
		return sp;
	}

	float target_altitude() const { return target_altitude_m_; }

private:
	float target_altitude_m_{-5.0f}; // NED: 위로 갈수록 음수
	float target_yaw_rad_{-3.14f};
};

// ─────────────────────────────────────
// 4. 메인 노드: Mission Layer
// ─────────────────────────────────────
class OffboardMissionNode : public rclcpp::Node
{
public:
	OffboardMissionNode()
	: Node("offboard_mission_node"),
	  px4_if_(*this),
	  hover_ctrl_(-5.0f, -3.14f)
	{
		// ▼ Vehicle Odometry (고도용)
		vehicle_odometry_sub_ =
			this->create_subscription<VehicleOdometry>(
				"/fmu/out/vehicle_odometry",
				rclcpp::SensorDataQoS(),
				[this](const VehicleOdometry::SharedPtr msg)
				{
					current_z_ = msg->position[2];  // NED: z (Down, 위로 갈수록 음수)
				});

		// ▼ Vehicle Status (arming 상태, nav_state 등)
		vehicle_status_sub_ =
			this->create_subscription<VehicleStatus>(
				"/fmu/out/vehicle_status",
				rclcpp::SensorDataQoS(),
				[this](const VehicleStatus::SharedPtr msg)
				{
					last_vehicle_status_ = *msg;
					has_vehicle_status_ = true;
					// 필요하면 디버그 로그:
					// RCLCPP_INFO(this->get_logger(), "VehicleStatus: armed=%d", msg->arming_state);
				});

		offboard_setpoint_counter_ = 0;
		phase_ = MissionPhase::IDLE;
		phase_tick_ = 0;

		target_altitude_m_ = hover_ctrl_.target_altitude();

		// ▼ 100 ms 메인 루프
		auto timer_callback = [this]() -> void {
			this->on_timer();
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

private:
	// 타이머
	rclcpp::TimerBase::SharedPtr timer_;

	// Interface Layer
	PX4OffboardInterface px4_if_;
	HoverController      hover_ctrl_;

	// Subscriber들
	rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
	rclcpp::Subscription<VehicleStatus>::SharedPtr   vehicle_status_sub_;

	// Vehicle Status 보관
	VehicleStatus last_vehicle_status_{};
	bool          has_vehicle_status_{false};

	// 내부 상태
	std::atomic<uint64_t> timestamp_{0};
	uint64_t offboard_setpoint_counter_{0};

	MissionPhase phase_{MissionPhase::IDLE};
	uint64_t     phase_tick_{0};
	float        target_altitude_m_{-5.0f};  // HoverController와 동일하게 유지

	float current_z_{0.0f}; // NED z

	// 내부 함수들
	void on_timer();

	// 상태 머신
	void update_mission_state();
	void change_phase(MissionPhase new_phase);
	const char* phase_to_string(MissionPhase p);

	// Hover/Takeoff 등 Phase별 setpoint 생성
	void publish_setpoint_for_phase();
};

// ─────────────────────────────────────
// 4-1. Phase 이름 문자열 변환
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
// 4-2. Phase 변경 공용 함수
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
	phase_tick_ = 0;
}

// ─────────────────────────────────────
// 4-3. 상태 전환 로직 (FSM)
// ─────────────────────────────────────
void OffboardMissionNode::update_mission_state()
{
	switch (phase_) {

	case MissionPhase::IDLE:
		// 첫 시작에서 TAKEOFF 진입
		if (offboard_setpoint_counter_ == 0) {
			change_phase(MissionPhase::TAKEOFF);
		}
		break;

	case MissionPhase::TAKEOFF:
	{
		const float z_err = current_z_ - target_altitude_m_;
		// RCLCPP_INFO(this->get_logger(), "TAKEOFF: z=%.2f target=%.2f err=%.2f",
		//             current_z_, target_altitude_m_, z_err);

		if (std::fabs(z_err) < 0.3f) {
			change_phase(MissionPhase::HOVER);
		}
		break;
	}

	case MissionPhase::HOVER:
		// 약 10초 호버 후 DONE으로
		if (phase_tick_ > 100) {
			change_phase(MissionPhase::DONE);
		}
		break;

	case MissionPhase::FORWARD_TRANSITION:
		// TODO: Phase 4에서 구현
		break;

	case MissionPhase::PATH_FLIGHT:
		// TODO
		break;

	case MissionPhase::BACK_TRANSITION:
		// TODO
		break;

	case MissionPhase::LAND:
		// TODO
		break;

	case MissionPhase::DONE:
		// 필요시 여기서 disarm 등 호출 가능
		break;
	}

	phase_tick_++;
}

// ─────────────────────────────────────
// 4-4. Phase별 setpoint 생성 + 퍼블리시
// ─────────────────────────────────────
void OffboardMissionNode::publish_setpoint_for_phase()
{
	switch (phase_) {

	case MissionPhase::IDLE:
		// Offboard 유지용 (0,0,0)
		px4_if_.publish_position_setpoint(0.0f, 0.0f, 0.0f, 0.0f);
		break;

	case MissionPhase::TAKEOFF:
	{
		// TAKEOFF도 HoverController의 목표 고도 setpoint 사용
		TrajectorySetpoint sp = hover_ctrl_.create_setpoint();
		px4_if_.publish_trajectory(sp);
		break;
	}

	case MissionPhase::HOVER:
	{
		// 완전 분리된 HoverController에서 setpoint 생성
		TrajectorySetpoint sp = hover_ctrl_.create_setpoint();
		px4_if_.publish_trajectory(sp);
		break;
	}

	case MissionPhase::FORWARD_TRANSITION:
		// TODO: VTOL -> Fixed-wing 전환 시 setpoint
		px4_if_.publish_position_setpoint(0.0f, 0.0f, target_altitude_m_, -3.14f);
		break;

	case MissionPhase::PATH_FLIGHT:
		// TODO: waypoint 기반 경로 비행 setpoint
		px4_if_.publish_position_setpoint(0.0f, 0.0f, target_altitude_m_, -3.14f);
		break;

	case MissionPhase::BACK_TRANSITION:
		// TODO: Fixed-wing -> VTOL
		px4_if_.publish_position_setpoint(0.0f, 0.0f, target_altitude_m_, -3.14f);
		break;

	case MissionPhase::LAND:
		// TODO: 착륙 setpoint
		px4_if_.publish_position_setpoint(0.0f, 0.0f, -0.5f, -3.14f);
		break;

	case MissionPhase::DONE:
		// 미션 종료 후: 일단 호버 setpoint 유지
		px4_if_.publish_position_setpoint(0.0f, 0.0f, target_altitude_m_, -3.14f);
		break;
	}
}

// ─────────────────────────────────────
// 4-5. 타이머 콜백
// ─────────────────────────────────────
void OffboardMissionNode::on_timer()
{
	// 1) 일정 횟수 setpoint 보낸 뒤 Offboard 모드 전환 + ARM
	if (offboard_setpoint_counter_ == 10) {
		px4_if_.command_client().set_offboard_mode();
		px4_if_.command_client().arm();
	}

	// 2) 상태 머신 업데이트
	update_mission_state();

	// 3) OffboardControlMode + Phase별 setpoint
	px4_if_.publish_position_control_mode();
	publish_setpoint_for_phase();

	// 4) 카운터 증가
	offboard_setpoint_counter_++;
}

// ─────────────────────────────────────
// 5. main 함수
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
