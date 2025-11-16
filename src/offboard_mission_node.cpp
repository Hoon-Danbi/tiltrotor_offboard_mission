#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vtol_vehicle_status.hpp"

using namespace std::chrono_literals;

using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleOdometry;
using px4_msgs::msg::VtolVehicleStatus;

struct Waypoint {
  float x;    // NED x [m]
  float y;    // NED y [m]
  float alt;  // +Up [m]
};

class OffboardMissionNode : public rclcpp::Node
{
public:
  OffboardMissionNode()
  : Node("offboard_mission_node"),
    wp1_{0.0f, 0.0f, 30.0f},
    wp2_{200.0f, 200.0f, 30.0f}
  {
    using std::placeholders::_1;

    // PX4 ROS2 브리지와 맞는 토픽 이름
    offboard_control_mode_pub_ =
      this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_pub_ =
      this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_pub_ =
      this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

    // Odometry 구독 (/fmu/out/vehicle_odometry)
    odom_sub_ = this->create_subscription<VehicleOdometry>(
      "/fmu/out/vehicle_odometry",
      rclcpp::SensorDataQoS(),
      std::bind(&OffboardMissionNode::on_odometry, this, _1));

    // VTOL 상태 구독
    vtol_status_sub_ = this->create_subscription<VtolVehicleStatus>(
      "/fmu/out/vtol_vehicle_status",
      rclcpp::SensorDataQoS(),
      std::bind(&OffboardMissionNode::on_vtol_status, this, _1));

    // 기본 상태값
    timestamp_         = 0;
    current_px_        = 0.0f;
    current_py_        = 0.0f;
    current_pz_        = 0.0f;
    current_alt_m_     = 0.0f;

    current_vx_        = 0.0f;
    current_vy_        = 0.0f;
    current_vz_        = 0.0f;
    current_speed_xy_  = 0.0f;

    odom_received_     = false;

    vtol_state_        = VtolVehicleStatus::VEHICLE_VTOL_STATE_UNDEFINED;
    vtol_cmd_sent_     = false;
    offboard_setpoint_counter_ = 0;

    // WP1 → WP2 방향 정보 계산
    yaw_leg_wp1_to_wp2_ = compute_leg_yaw_rad(wp1_, wp2_);

    const float leg_dx = wp2_.x - wp1_.x;
    const float leg_dy = wp2_.y - wp1_.y;
    leg_length_ = std::sqrt(leg_dx * leg_dx + leg_dy * leg_dy);

    if (leg_length_ > 1e-3f) {
      leg_dir_x_ = leg_dx / leg_length_;
      leg_dir_y_ = leg_dy / leg_length_;
    } else {
      leg_dir_x_ = 1.0f;
      leg_dir_y_ = 0.0f;
    }

    phase_ = MissionPhase::MC_TAKEOFF_TO_WP1;
    phase_enter_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Offboard mission node started.");

    timer_ = this->create_wall_timer(
      50ms, std::bind(&OffboardMissionNode::on_timer, this));
  }

private:
  enum class MissionPhase {
    MC_TAKEOFF_TO_WP1 = 0,
    MC_HOLD_AT_WP1,
    TRANSITION_AND_GUIDE_TO_WP2,
    FW_LOITER_AT_WP2,
    DONE
  };

	// ---- 클래스 private: 위쪽에 상수 몇 개 추가 ----
	static constexpr float V_TRANSITION_MIN_XY_      = 8.0f;   // 지금 네가 쓴 값
	static constexpr float ALT_TRANSITION_MIN_       = 25.0f;  // transition 최소 고도
	static constexpr float FW_GUIDE_AHEAD_DIST_      = 80.0f;  // 레그 위로 미리 보는 거리 [m]
	static constexpr float FW_REACH_WP2_DIST_XY_     = 20.0f;  // WP2 도달 XY 허용 오차
	static constexpr float FW_REACH_WP2_ALT_ERR_     = 5.0f;   // WP2 도달 고도 오차

  // ===== Phase 문자열 변환 =====
  const char* phase_to_string(MissionPhase p) const
  {
    switch (p) {
      case MissionPhase::MC_TAKEOFF_TO_WP1:             return "MC_TAKEOFF_TO_WP1";
      case MissionPhase::MC_HOLD_AT_WP1:                return "MC_HOLD_AT_WP1";
      case MissionPhase::TRANSITION_AND_GUIDE_TO_WP2:   return "TRANSITION_AND_GUIDE_TO_WP2";
      case MissionPhase::FW_LOITER_AT_WP2:              return "FW_LOITER_AT_WP2";
      case MissionPhase::DONE:                          return "DONE";
      default:                                          return "UNKNOWN";
    }
  }

  // ===== Odometry 업데이트 =====
  void on_odometry(const VehicleOdometry::SharedPtr msg)
  {
    // 위치: NED (x,y,z), z는 +Down
    current_px_ = msg->position[0];
    current_py_ = msg->position[1];
    current_pz_ = msg->position[2];

    // 속도: NED (vx,vy,vz)
    current_vx_ = msg->velocity[0];
    current_vy_ = msg->velocity[1];
    current_vz_ = msg->velocity[2];

    current_speed_xy_ = std::sqrt(current_vx_ * current_vx_ +
                                  current_vy_ * current_vy_);

    current_alt_m_ = -current_pz_;  // +Up 기준 고도
    odom_received_ = true;

    // 디버그용 (1초에 한 번)
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Odom: x=%.1f y=%.1f alt=%.1f (z=%.1f) | vN=%.1f vE=%.1f v_xy=%.1f",
      current_px_, current_py_, current_alt_m_, current_pz_,
      current_vx_, current_vy_, current_speed_xy_);
  }

  // ===== VTOL 상태 업데이트 =====
  void on_vtol_status(const VtolVehicleStatus::SharedPtr msg)
  {
    vtol_state_ = msg->vehicle_vtol_state;

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "VTOL state = %u", (unsigned)vtol_state_);
  }

  // ===== 공통: Phase 변경 =====
  void set_phase(MissionPhase new_phase)
  {
    if (new_phase == phase_) {
      return;
    }

    RCLCPP_INFO(
      this->get_logger(), "Phase change: %s -> %s",
      phase_to_string(phase_), phase_to_string(new_phase));

    phase_ = new_phase;
    phase_enter_time_ = this->now();

    switch (phase_) {
      case MissionPhase::MC_TAKEOFF_TO_WP1:
        break;

      case MissionPhase::MC_HOLD_AT_WP1:
        // WP1 → WP2 방향 yaw 계산 (이미 있지만 재계산해도 무방)
        yaw_leg_wp1_to_wp2_ = compute_leg_yaw_rad(wp1_, wp2_);
        break;

      case MissionPhase::TRANSITION_AND_GUIDE_TO_WP2:
        vtol_cmd_sent_ = false;
        vtol_cmd_last_time_ = this->now();
        break;

      case MissionPhase::FW_LOITER_AT_WP2:
        break;

      case MissionPhase::DONE:
        break;
    }
  }

  // ===== 레그 방향 yaw 계산 (from → to) =====
  float compute_leg_yaw_rad(const Waypoint & from, const Waypoint & to)
  {
    float dx = to.x - from.x;
    float dy = to.y - from.y;
    return std::atan2(dy, dx);  // rad
  }

  // ===== VehicleCommand 전송 공통 함수 =====
  void publish_vehicle_command(
    uint16_t command,
    float param1 = 0.0f,
    float param2 = 0.0f)
  {
    VehicleCommand msg{};
    msg.timestamp = timestamp_;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;

    vehicle_command_pub_->publish(msg);
  }

  void arm()
  {
    RCLCPP_INFO(this->get_logger(), "Arm command sent");
    publish_vehicle_command(
      VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
      1.0f);
  }

  void disarm()
  {
    RCLCPP_INFO(this->get_logger(), "Disarm command sent");
    publish_vehicle_command(
      VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
      0.0f);
  }

  void send_vtol_transition_to_fw()
  {
    // MAV_CMD_DO_VTOL_TRANSITION, param1 = 4 (FW)
    publish_vehicle_command(
      VehicleCommand::VEHICLE_CMD_DO_VTOL_TRANSITION,
      4.0f);
  }

  void publish_offboard_control_mode()
  {
    OffboardControlMode msg{};
    msg.timestamp = timestamp_;

    msg.position = true;
    msg.velocity = true;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;

    offboard_control_mode_pub_->publish(msg);
  }

  // ===== Phase별 핸들러 =====

  // 회전익: WP1(0,0,30)까지 position setpoint만 사용
  void handle_mc_takeoff_to_wp1()
  {
    TrajectorySetpoint sp{};
    sp.timestamp = timestamp_;

    sp.position[0] = wp1_.x;
    sp.position[1] = wp1_.y;
    sp.position[2] = -wp1_.alt;   // +Up → NED Down

    sp.velocity[0] = NAN;
    sp.velocity[1] = NAN;
    sp.velocity[2] = NAN;

    sp.yaw = yaw_leg_wp1_to_wp2_;

    trajectory_setpoint_pub_->publish(sp);

    if (!odom_received_) {
      return;
    }

    float dx = wp1_.x - current_px_;
    float dy = wp1_.y - current_py_;
    float dist_xy = std::sqrt(dx * dx + dy * dy);
    float alt_err = std::fabs(current_alt_m_ - wp1_.alt);

    // 조건 조금 느슨하게 (2m / 1m)
    if (dist_xy < 2.0f && alt_err < 1.0f) {
      set_phase(MissionPhase::MC_HOLD_AT_WP1);
    }
  }

  // 회전익: WP1에서 위치 고정, yaw를 다음 레그 방향으로 미리 돌려놈
  void handle_mc_hold_at_wp1()
  {
    TrajectorySetpoint sp{};
    sp.timestamp = timestamp_;

    sp.position[0] = wp1_.x;
    sp.position[1] = wp1_.y;
    sp.position[2] = -wp1_.alt;

    sp.velocity[0] = NAN;
    sp.velocity[1] = NAN;
    sp.velocity[2] = NAN;

    sp.yaw = yaw_leg_wp1_to_wp2_;

    trajectory_setpoint_pub_->publish(sp);

    double hold_time = (this->now() - phase_enter_time_).seconds();
    if (hold_time > 2.0) {  // 2초 정도 유지 후 다음 phase
      set_phase(MissionPhase::TRANSITION_AND_GUIDE_TO_WP2);
    }
  }

  // ===== 천이 + WP2까지 유도 비행 =====
void handle_transition_and_guide_to_wp2()
{
  auto now_ts = this->now();

  // odom 없으면 아무 것도 못 함
  if (!odom_received_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "[TRANS] No odometry yet in transition phase");
    return;
  }

  // 현재 속도 크기 (수평)
  const float vN = current_vx_;
  const float vE = current_vy_;
  const float v_xy = std::sqrt(vN * vN + vE * vE);

  // ----- 1) 아직 FW 모드가 아니라면: MC에서 가속 + transition 명령 -----
  if (vtol_state_ != VtolVehicleStatus::VEHICLE_VTOL_STATE_FW) {

    // 우선 MC 모드일 때는 그냥 WP2 (또는 레그 방향) 쪽으로 position setpoint를 계속 줘서
    // 속도를 서서히 올리도록 한다.
    TrajectorySetpoint sp{};
    sp.timestamp = timestamp_;

    // 레그 방향으로 충분히 멀리 있는 점을 position setpoint로 준다 (MC position controller 이용)
    const float dx_leg = wp2_.x - wp1_.x;
    const float dy_leg = wp2_.y - wp1_.y;
    const float leg_len = std::sqrt(dx_leg * dx_leg + dy_leg * dy_leg);
    float ux = 0.0f;
    float uy = 0.0f;
    if (leg_len > 1e-3f) {
      ux = dx_leg / leg_len;
      uy = dy_leg / leg_len;
    }

    const float accel_dist = 80.0f;  // MC에서 가속하면서 당겨갈 거리
    float target_x = wp1_.x + ux * accel_dist;
    float target_y = wp1_.y + uy * accel_dist;

    sp.position[0] = target_x;
    sp.position[1] = target_y;
    sp.position[2] = -wp2_.alt;  // 고도는 30m로 맞춰놓고

    sp.velocity[0] = NAN;
    sp.velocity[1] = NAN;
    sp.velocity[2] = NAN;

    float yaw_to_leg = std::atan2(uy, ux);
    sp.yaw = yaw_to_leg;

    trajectory_setpoint_pub_->publish(sp);

    // transition 조건 체크
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "[TRANS] Waiting: speed=%.1f (>=%.1f?) alt=%.1f (>=%.1f?)",
      v_xy, V_TRANSITION_MIN_XY_, current_alt_m_, ALT_TRANSITION_MIN_);

    if (!vtol_cmd_sent_ &&
        (v_xy >= V_TRANSITION_MIN_XY_) &&
        (current_alt_m_ >= ALT_TRANSITION_MIN_))
    {
      // 조건 만족 → transition 명령 1회 전송
      send_vtol_transition_to_fw();
      vtol_cmd_sent_ = true;
      vtol_cmd_last_time_ = now_ts;

      RCLCPP_INFO(
        this->get_logger(),
        "[TRANS] Sending VTOL FW transition command (vtol_state=%u, v_xy=%.1f, alt=%.1f)",
        (unsigned)vtol_state_, v_xy, current_alt_m_);
    }

    // 아직 FW 아니면 여기서 종료
    return;
  }

  // ----- 2) 여기부터는 FW 모드: 레그 상의 당근(position)으로 유도 -----

  // 레그 벡터와 단위벡터
  const float dx_leg = wp2_.x - wp1_.x;
  const float dy_leg = wp2_.y - wp1_.y;
  const float leg_len = std::sqrt(dx_leg * dx_leg + dy_leg * dy_leg);
  if (leg_len < 1e-3f) {
    // 레그 길이가 거의 0이면 그냥 WP2로 바로 쏴버림
    TrajectorySetpoint sp{};
    sp.timestamp = timestamp_;
    sp.position[0] = wp2_.x;
    sp.position[1] = wp2_.y;
    sp.position[2] = -wp2_.alt;
    sp.velocity[0] = NAN;
    sp.velocity[1] = NAN;
    sp.velocity[2] = NAN;
    sp.yaw = std::atan2(wp2_.y - current_py_, wp2_.x - current_px_);
    trajectory_setpoint_pub_->publish(sp);
    return;
  }

  const float ux = dx_leg / leg_len;
  const float uy = dy_leg / leg_len;

  // 현재 위치를 레그 좌표계로 투영
  const float rx = current_px_ - wp1_.x;
  const float ry = current_py_ - wp1_.y;
  const float along = rx * ux + ry * uy;
  const float cross = rx * uy - ry * ux;  // sign은 그냥 디버깅용

  // 레그 위의 당근점 계산
  float s = along + FW_GUIDE_AHEAD_DIST_;  // 앞쪽으로 한 80m 정도 미리 본다
  if (s < 0.0f) s = 0.0f;
  if (s > leg_len) s = leg_len;

  const float carrot_x = wp1_.x + ux * s;
  const float carrot_y = wp1_.y + uy * s;

  const float dx_c = carrot_x - current_px_;
  const float dy_c = carrot_y - current_py_;
  const float dist_carrot = std::sqrt(dx_c * dx_c + dy_c * dy_c);

  TrajectorySetpoint sp{};
  sp.timestamp = timestamp_;

  sp.position[0] = carrot_x;
  sp.position[1] = carrot_y;
  sp.position[2] = -wp2_.alt;  // 고도는 계속 30m 목표

  sp.velocity[0] = NAN;
  sp.velocity[1] = NAN;
  sp.velocity[2] = NAN;

  sp.yaw = std::atan2(dy_c, dx_c);

  trajectory_setpoint_pub_->publish(sp);

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "[FW-GUIDE] along=%.1f/%.1f cross=%.1f dist_carrot=%.1f v_xy=%.1f",
    along, leg_len, cross, dist_carrot, v_xy);

  // ----- 3) WP2 도달 판정 -----
  const float dx_wp2 = wp2_.x - current_px_;
  const float dy_wp2 = wp2_.y - current_py_;
  const float dist_wp2_xy = std::sqrt(dx_wp2 * dx_wp2 + dy_wp2 * dy_wp2);
  const float alt_err = std::fabs(current_alt_m_ - wp2_.alt);

  if (dist_wp2_xy < FW_REACH_WP2_DIST_XY_ && alt_err < FW_REACH_WP2_ALT_ERR_) {
    set_phase(MissionPhase::FW_LOITER_AT_WP2);
  }
}

  // 고정익: WP2 도달 후 PX4 내부 ORBIT(Loiter) 모드로 전환
  void handle_fw_loiter_at_wp2()
  {
    RCLCPP_INFO(this->get_logger(),
                "Reached WP2. Switching PX4 to ORBIT (standard mode) and ending Offboard mission.");

    VehicleCommand cmd{};
    cmd.timestamp        = timestamp_;
    cmd.target_system    = 1;
    cmd.target_component = 1;
    cmd.source_system    = 1;
    cmd.source_component = 1;
    cmd.from_external    = true;

    // MAV_CMD_DO_SET_STANDARD_MODE = 262
    cmd.command = 262;

    // MAV_STANDARD_MODE_ORBIT = 2
    cmd.param1 = 2.0f;

    vehicle_command_pub_->publish(cmd);

    set_phase(MissionPhase::DONE);
  }

  // ===== 타이머 콜백 =====
  void on_timer()
  {
    if (phase_ == MissionPhase::DONE) {
      return;
    }

    // PX4 timestamp: ROS2 now를 microseconds로 변환해서 사용
    timestamp_ = this->now().nanoseconds() / 1000;

    // OffboardControlMode는 매 주기 publish
    publish_offboard_control_mode();

    // Offboard 모드 진입/arm 패턴 (예: 10주기 후)
    if (offboard_setpoint_counter_ == 10) {
      RCLCPP_INFO(this->get_logger(), "Offboard mode command sent");
      publish_vehicle_command(
        VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
        1, 6);  // 1: custom, 6: offboard
      arm();
    }
    if (offboard_setpoint_counter_ < 11) {
      offboard_setpoint_counter_++;
    }

    // Phase별 TrajectorySetpoint / 명령 생성
    switch (phase_) {
      case MissionPhase::MC_TAKEOFF_TO_WP1:
        handle_mc_takeoff_to_wp1();
        break;

      case MissionPhase::MC_HOLD_AT_WP1:
        handle_mc_hold_at_wp1();
        break;

      case MissionPhase::TRANSITION_AND_GUIDE_TO_WP2:
        handle_transition_and_guide_to_wp2();
        break;

      case MissionPhase::FW_LOITER_AT_WP2:
        handle_fw_loiter_at_wp2();
        break;

      case MissionPhase::DONE:
        break;
    }
  }

  // ===== 멤버 변수 =====
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<OffboardControlMode>::SharedPtr    offboard_control_mode_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr     trajectory_setpoint_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr         vehicle_command_pub_;

  rclcpp::Subscription<VehicleOdometry>::SharedPtr     odom_sub_;
  rclcpp::Subscription<VtolVehicleStatus>::SharedPtr   vtol_status_sub_;

  uint64_t timestamp_;

  // 현 위치/고도/속도
  float current_px_;
  float current_py_;
  float current_pz_;      // NED +Down
  float current_alt_m_;   // +Up

  float current_vx_;      // NED x [m/s]
  float current_vy_;      // NED y [m/s]
  float current_vz_;      // NED z [m/s]
  float current_speed_xy_;

  bool  odom_received_;

  // VTOL 상태 (uORB vehicle_vtol_state)
  uint8_t      vtol_state_;
  bool         vtol_cmd_sent_;
  rclcpp::Time vtol_cmd_last_time_;

  // 미션
  Waypoint wp1_;
  Waypoint wp2_;

  MissionPhase  phase_;
  rclcpp::Time  phase_enter_time_;
  float         yaw_leg_wp1_to_wp2_;

  // WP1↔WP2 레그 정보
  float leg_dir_x_;
  float leg_dir_y_;
  float leg_length_;

  int offboard_setpoint_counter_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardMissionNode>());
  rclcpp::shutdown();
  return 0;
}
