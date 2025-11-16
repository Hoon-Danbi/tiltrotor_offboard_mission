/****************************************************************************
 * Offboard mission node
 * - Phase 2: Mission FSM
 * - Phase 3: PX4 인터페이스 분리 버전
 ****************************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>

#include "tiltrotor_offboard_mission/px4_offboard_interface.hpp"

using namespace std::chrono_literals;
using px4_msgs::msg::VehicleOdometry;
using px4_msgs::msg::VehicleStatus;

// ─────────────────────────────────────
// MissionPhase 정의
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
// OffboardMissionNode (Mission / FSM 레이어)
// ─────────────────────────────────────
class OffboardMissionNode : public rclcpp::Node
{
public:
	OffboardMissionNode()
	: Node("offboard_mission_node"),
	  px4_if_(*this),
	  hover_ctrl_(-5.0f, -3.14f)
	{
		// Vehicle Odometry (z 사용)
		vehicle_odometry_sub_ =
			this->create_subscription<VehicleOdometry>(
				"/fmu/out/vehicle_odometry",
				rclcpp::SensorDataQoS(),
				[this](const VehicleOdometry::SharedPtr msg)
				{
					current_z_ = msg->position[2];
				});

		// Vehicle Status (arming, nav_state 등)
		vehicle_status_sub_ =
			this->create_subscription<VehicleStatus>(
				"/fmu/out/vehicle_status",
				rclcpp::SensorDataQoS(),
				[this](const VehicleStatus::SharedPtr msg)
				{
					last_vehicle_status_ = *msg;
					has_vehicle_status_ = true;
				});

		offboard_setpoint_counter_ = 0;
		phase_ = MissionPhase::IDLE;
		phase_tick_ = 0;
		target_altitude_m_ = hover_ctrl_.target_altitude();

		auto timer_callback = [this]() -> void {
			this->on_timer();
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

private:
	// 타이머
	rclcpp::TimerBase::SharedPtr timer_;

	// PX4 인터페이스
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
	float        target_altitude_m_{-5.0f};

	float current_z_{0.0f}; // NED z

	// 내부 함수들
	void on_timer();

	void update_mission_state();
	void change_phase(MissionPhase new_phase);
	const char* phase_to_string(MissionPhase p);

	void publish_setpoint_for_phase();
};

// ─────────────────────────────────────
// helper: Phase → 문자열
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
// 상태 전환 로직 (FSM)
// ─────────────────────────────────────
void OffboardMissionNode::update_mission_state()
{
	switch (phase_) {

	case MissionPhase::IDLE:
		if (offboard_setpoint_counter_ == 0) {
			change_phase(MissionPhase::TAKEOFF);
		}
		break;

	case MissionPhase::TAKEOFF:
	{
		const float z_err = current_z_ - target_altitude_m_;
		if (std::fabs(z_err) < 0.3f) {
			change_phase(MissionPhase::HOVER);
		}
		break;
	}

	case MissionPhase::HOVER:
		if (phase_tick_ > 100) {
			change_phase(MissionPhase::DONE);
		}
		break;

	case MissionPhase::FORWARD_TRANSITION:
		// TODO: 이후 Phase 4에서 구현
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
		// 필요시 disarm 등 호출 가능
		break;
	}

	phase_tick_++;
}

// ─────────────────────────────────────
// Phase별 setpoint 생성 + 퍼블리시
// ─────────────────────────────────────
void OffboardMissionNode::publish_setpoint_for_phase()
{
	switch (phase_) {

	case MissionPhase::IDLE:
		px4_if_.publish_position_setpoint(0.0f, 0.0f, 0.0f, 0.0f);
		break;

	case MissionPhase::TAKEOFF:
	{
		auto sp = hover_ctrl_.create_setpoint();
		px4_if_.publish_trajectory(sp);
		break;
	}

	case MissionPhase::HOVER:
	{
		auto sp = hover_ctrl_.create_setpoint();
		px4_if_.publish_trajectory(sp);
		break;
	}

	case MissionPhase::FORWARD_TRANSITION:
		px4_if_.publish_position_setpoint(0.0f, 0.0f, target_altitude_m_, -3.14f);
		break;

	case MissionPhase::PATH_FLIGHT:
		px4_if_.publish_position_setpoint(0.0f, 0.0f, target_altitude_m_, -3.14f);
		break;

	case MissionPhase::BACK_TRANSITION:
		px4_if_.publish_position_setpoint(0.0f, 0.0f, target_altitude_m_, -3.14f);
		break;

	case MissionPhase::LAND:
		px4_if_.publish_position_setpoint(0.0f, 0.0f, -0.5f, -3.14f);
		break;

	case MissionPhase::DONE:
		px4_if_.publish_position_setpoint(0.0f, 0.0f, target_altitude_m_, -3.14f);
		break;
	}
}

// ─────────────────────────────────────
// 타이머 콜백
// ─────────────────────────────────────
void OffboardMissionNode::on_timer()
{
	// 1) Offboard 모드 전환 + ARM
	if (offboard_setpoint_counter_ == 10) {
		px4_if_.command_client().set_offboard_mode();
		px4_if_.command_client().arm();
	}

	// 2) 상태 업데이트
	update_mission_state();

	// 3) OffboardControlMode + setpoint
	px4_if_.publish_position_control_mode();
	publish_setpoint_for_phase();

	// 4) 카운터 증가
	offboard_setpoint_counter_++;
}

// ─────────────────────────────────────
// main
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
