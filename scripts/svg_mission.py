#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
svg_mission.py
--------------
SVG 경로를 따라 PX4 드론을 비행시키는 ROS Melodic 노드.
RVizPainter를 연동하여 3D 시각화(마커, 궤적, 현재 위치 등)를 지원합니다.
"""

import os
import sys
import math
import threading
import rospy
import tf
import tf.transformations  # 쿼터니언 -> 오일러 변환용
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from svg_parser import parse_svg
from rviz_painter import RVizPainter  # RViz 시각화 모듈 임포트

_POS_YAW_MASK = (
    PositionTarget.IGNORE_VX |
    PositionTarget.IGNORE_VY |
    PositionTarget.IGNORE_VZ |
    PositionTarget.IGNORE_AFX |
    PositionTarget.IGNORE_AFY |
    PositionTarget.IGNORE_AFZ |
    PositionTarget.IGNORE_YAW_RATE
)


class SvgMissionNode(object):
    def __init__(self):
        rospy.init_node('svg_mission_node', anonymous=True)

        # ─── 파라미터 ──────────────────────────────────────────
        svg_file        = rospy.get_param('~svg_file', '')
        self.scale      = rospy.get_param('~scale', 0.05)
        self.alt        = rospy.get_param('~altitude', 5.0)
        self.wp_radius  = rospy.get_param('~wp_radius', 0.4)
        self.wp_timeout = rospy.get_param('~wp_timeout', 10.0)
        self.cruise_v   = rospy.get_param('~cruise_velocity', 1.0)  # m/s
        self.takeoff_min_alt = rospy.get_param('~takeoff_min_alt', 1.5)

        if not svg_file or not os.path.exists(svg_file):
            rospy.logerr("[SvgMission] SVG 파일 없음: %s" % svg_file)
            sys.exit(1)

        self.waypoints = parse_svg(svg_file, self.scale)
        if not self.waypoints:
            rospy.logerr("[SvgMission] Waypoint 없음")
            sys.exit(1)
        rospy.loginfo("[SvgMission] Waypoint %d개 로드 완료" % len(self.waypoints))

        # ─── 시각화 객체 초기화 (RVizPainter) ─────────────────
        # MAVROS의 local position은 주로 'map' 또는 'local_origin' 프레임을 사용합니다.
        self.painter = RVizPainter(frame_id='map', trail_min_dist=0.1, trail_max_points=0)
        
        # 시작 전 전체 계획 경로 및 웨이포인트 시각화
        self.painter.draw_waypoints(self.waypoints, z=self.alt)
        self.painter.draw_planned_path(self.waypoints, z=self.alt)

        # ─── 상태 ─────────────────────────────────────────────
        self.state = State()
        self.pose  = PoseStamped()

        self._lock = threading.Lock()
        self._tgt = [0.0, 0.0, 0.0, 0.0]
        self._streaming = True
        self._pose_received = False

        # ─── 통신 ─────────────────────────────────────────────
        self.raw_pub = rospy.Publisher(
            '/mavros/setpoint_raw/local', PositionTarget, queue_size=1
        )

        rospy.Subscriber('/mavros/state',               State,       self._cb_state)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._cb_pose)

        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        self.arming_srv   = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode',   SetMode)

        t = threading.Thread(target=self._setpoint_loop)
        t.daemon = True
        t.start()
        rospy.loginfo("[SvgMission] setpoint 스트리밍 스레드 시작 (pose 대기 후 publish)")

        # ─── TF Broadcaster (드론 프레임 발행) ─────────────────
        self.tf_br = tf.TransformBroadcaster()
        rospy.loginfo("[SvgMission] TF Broadcaster 초기화 완료")

    # ══════════════════════════════════════════════════════════════
    #  콜백 / 헬퍼
    # ══════════════════════════════════════════════════════════════

    def _cb_state(self, msg):
        self.state = msg

    def _cb_pose(self, msg):
        self.pose = msg
        if not self._pose_received:
            self._pose_received = True
            
        if rospy.is_shutdown() or not self._streaming:
            return

        # --- TF Broadcast: map → drone 프레임 변환 ---
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        q = msg.pose.orientation
        
        # TF broadcast (RViz의 "Drone Body Axes"가 이를 추적)
        try:
            self.tf_br.sendTransform(
                (x, y, z),
                (q.x, q.y, q.z, q.w),
                msg.header.stamp,
                "drone",
                "map"
            )
        except rospy.ROSException as e:
            rospy.logwarn("[SvgMission] TF broadcast 중단: %s" % e)

        # --- RViz 현재 위치 및 잔상 업데이트 ---
        # Painter 업데이트 (전체 quaternion 전달)
        if hasattr(self, 'painter'):
            try:
                self.painter.update_trail(x, y, z)
                self.painter.draw_current_pose(x, y, z, quaternion=(q.x, q.y, q.z, q.w))
            except rospy.ROSException as e:
                rospy.logwarn("[SvgMission] RViz painter publish 중단: %s" % e)

    def _set_tgt(self, x, y, z, yaw=None):
        """target setpoint 갱신. yaw가 None이면 기존 yaw 유지."""
        with self._lock:
            self._tgt[0] = x
            self._tgt[1] = y
            self._tgt[2] = z
            if yaw is not None:
                self._tgt[3] = yaw
                
        # --- RViz 목표 Setpoint 마커 업데이트 ---
        if hasattr(self, 'painter'):
            self.painter.draw_target(x, y, z)

    def _get_tgt(self):
        with self._lock:
            return list(self._tgt)

    def _make_raw_msg(self, x, y, z, yaw):
        msg = PositionTarget()
        msg.header.stamp     = rospy.Time.now()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask        = _POS_YAW_MASK
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z
        msg.yaw = yaw
        return msg

    def _setpoint_loop(self):
        rate = rospy.Rate(20)

        while not rospy.is_shutdown() and not self._pose_received:
            rate.sleep()

        with self._lock:
            if self._tgt[0] == 0.0 and self._tgt[1] == 0.0 and self._tgt[2] == 0.0:
                self._tgt[0] = self.pose.pose.position.x
                self._tgt[1] = self.pose.pose.position.y
                self._tgt[2] = self.pose.pose.position.z
                self._tgt[3] = 0.0

        rospy.loginfo("[SvgMission] setpoint 스트리밍 활성 (초기 target=현재 위치)")

        while not rospy.is_shutdown() and self._streaming:
            x, y, z, yaw = self._get_tgt()
            self.raw_pub.publish(self._make_raw_msg(x, y, z, yaw))
            rate.sleep()

    def _dist(self, x, y, z):
        dx = self.pose.pose.position.x - x
        dy = self.pose.pose.position.y - y
        dz = self.pose.pose.position.z - z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def _yaw_toward(self, x, y):
        dx = x - self.pose.pose.position.x
        dy = y - self.pose.pose.position.y
        if dx*dx + dy*dy < 0.01:
            return self._get_tgt()[3]
        return math.atan2(dy, dx)

    # ══════════════════════════════════════════════════════════════
    #  STAGE 1 ~ 3
    # ══════════════════════════════════════════════════════════════

    def _wait_fcu_and_pose(self):
        rospy.loginfo("[SvgMission] FCU 연결 대기...")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.state.connected:
            rate.sleep()
        rospy.loginfo("[SvgMission] FCU 연결 완료")

        rospy.loginfo("[SvgMission] 첫 pose 수신 대기...")
        while not rospy.is_shutdown() and not self._pose_received:
            rate.sleep()
        p = self.pose.pose.position
        rospy.loginfo("[SvgMission] 초기 위치: (%.2f, %.2f, %.2f)" % (p.x, p.y, p.z))
        return True

    def _takeoff_auto(self):
        rospy.sleep(2.0)

        rospy.loginfo("[SvgMission] AUTO.TAKEOFF 모드 전환...")
        if not self.set_mode_srv(custom_mode='AUTO.TAKEOFF').mode_sent:
            rospy.logerr("[SvgMission] AUTO.TAKEOFF 모드 전환 실패")
            return False
        rospy.sleep(1.0)

        rospy.loginfo("[SvgMission] ARM 요청...")
        if not self.arming_srv(value=True).success:
            rospy.logerr("[SvgMission] ARM 실패")
            return False
        rospy.loginfo("[SvgMission] ARM 성공 — 이륙 진행...")

        rate  = rospy.Rate(2)
        start = rospy.Time.now()

        while not rospy.is_shutdown():
            alt     = self.pose.pose.position.z
            mode    = self.state.mode
            elapsed = (rospy.Time.now() - start).to_sec()
            rospy.loginfo("[SvgMission] 고도: %.2f m | 모드: %s | 경과 %.1fs" %
                          (alt, mode, elapsed))

            if mode == 'AUTO.RTL':
                rospy.logerr("[SvgMission] RTL 발동 — NAV_RCL_ACT=0 확인 필요")
                return False

            if mode == 'AUTO.LOITER':
                rospy.loginfo("[SvgMission] AUTO.LOITER 진입 → 이륙 완료 (%.2f m)" % alt)
                rospy.sleep(2.0)
                return True

            if alt >= self.takeoff_min_alt and elapsed >= 5.0:
                rospy.logwarn("[SvgMission] LOITER 미진입이지만 고도 %.2f m 도달 — 진행" % alt)
                rospy.sleep(1.0)
                return True

            if elapsed > 30.0:
                rospy.logerr("[SvgMission] 이륙 타임아웃 — 고도 %.2f m" % alt)
                return False

            rate.sleep()

        return False

    def _wait_for_landing(self, timeout=60.0):
        rospy.loginfo("[SvgMission] 착륙 완료 대기 (%ds) ..." % int(timeout))
        rate  = rospy.Rate(2)
        start = rospy.Time.now()

        while not rospy.is_shutdown():
            alt  = self.pose.pose.position.z
            mode = self.state.mode
            if alt <= 0.25 or mode != 'AUTO.LAND':
                rospy.loginfo("[SvgMission] 착륙 완료 감지 (alt=%.2f, mode=%s)" % (alt, mode))
                return True
            if (rospy.Time.now() - start).to_sec() > timeout:
                rospy.logwarn("[SvgMission] 착륙 대기 타임아웃 (alt=%.2f, mode=%s)" % (alt, mode))
                return False
            rate.sleep()

        return False

    def _enter_offboard(self):
        cx = self.pose.pose.position.x
        cy = self.pose.pose.position.y
        cz = self.pose.pose.position.z
        self._set_tgt(cx, cy, cz, yaw=0.0)

        rospy.loginfo("[SvgMission] OFFBOARD pre-stream 2초 (현재 %.2f m 유지)..." % cz)
        rospy.sleep(2.0)

        rospy.loginfo("[SvgMission] OFFBOARD 모드 요청...")
        for attempt in range(1, 6):
            resp = self.set_mode_srv(custom_mode='OFFBOARD')
            if resp.mode_sent:
                deadline = rospy.Time.now() + rospy.Duration(3.0)
                rate = rospy.Rate(10)
                while not rospy.is_shutdown() and rospy.Time.now() < deadline:
                    if self.state.mode == 'OFFBOARD':
                        rospy.loginfo("[SvgMission] OFFBOARD 진입 확인!")
                        return True
                    rate.sleep()

            rospy.logwarn("[SvgMission] OFFBOARD 재시도 %d/5 (현재 모드: %s)" % (
                attempt, self.state.mode))
            rospy.sleep(1.0)

        rospy.logerr("[SvgMission] OFFBOARD 진입 실패")
        return False

    def _goto(self, x, y, z, timeout=None, label='', yaw=None):
        d0 = self._dist(x, y, z)
        if timeout is None:
            timeout = max(self.wp_timeout, d0 / max(self.cruise_v, 0.1) + 3.0)

        if yaw is None:
            yaw = self._yaw_toward(x, y)

        self._set_tgt(x, y, z, yaw=yaw)

        if label:
            rospy.loginfo(
                "[SvgMission] %s → (%.2f, %.2f, %.2f) yaw=%.0f° (d=%.2f m, t≤%.1fs)" %
                (label, x, y, z, math.degrees(yaw), d0, timeout))

        rate  = rospy.Rate(5)
        start = rospy.Time.now()

        while not rospy.is_shutdown():
            if self.state.mode != 'OFFBOARD':
                rospy.logwarn_throttle(2.0,
                    "[SvgMission] OFFBOARD 이탈 (모드: %s) — 재진입 시도" %
                    self.state.mode)
                self.set_mode_srv(custom_mode='OFFBOARD')
                rospy.sleep(0.3)

            dist    = self._dist(x, y, z)
            elapsed = (rospy.Time.now() - start).to_sec()

            if dist < self.wp_radius:
                rospy.loginfo("[SvgMission]   ✓ 도달 (잔여 %.2f m, %.1f s)" %
                              (dist, elapsed))
                return True

            if elapsed > timeout:
                cur = self.pose.pose.position
                rospy.logwarn(
                    "[SvgMission]   ✗ 타임아웃 %.1f s | 현재 (%.2f,%.2f,%.2f) dist=%.2f m" %
                    (elapsed, cur.x, cur.y, cur.z, dist))
                return False

            rate.sleep()

        return False

    # ══════════════════════════════════════════════════════════════
    #  메인 시퀀스
    # ══════════════════════════════════════════════════════════════

    def run(self):
        try:
            # 1) FCU 연결 + 첫 pose 수신
            if not self._wait_fcu_and_pose():
                return

            # 2) AUTO.TAKEOFF + ARM (지상에서)
            if not self._takeoff_auto():
                rospy.logerr("[SvgMission] 이륙 실패 — 미션 중단")
                return

            # 3) 공중에서 OFFBOARD 전환
            if not self._enter_offboard():
                return

            # 4) 목표 고도까지 상승 (현재 xy 유지)
            cx = self.pose.pose.position.x
            cy = self.pose.pose.position.y
            cz = self.pose.pose.position.z
            rospy.loginfo("[SvgMission] 현재 %.2f m → 목표 %.1f m 상승" % (cz, self.alt))
            if not self._goto(cx, cy, self.alt, timeout=30.0,
                              label='고도 상승', yaw=0.0):
                rospy.logerr("[SvgMission] 고도 상승 실패 — 미션 중단")
                return

            # 5) SVG 경로 추종
            total = len(self.waypoints)
            rospy.loginfo("=" * 60)
            rospy.loginfo("[SvgMission] SVG 경로 추종 시작! (%d WP, %.1f m)" %
                          (total, self.alt))
            rospy.loginfo("=" * 60)

            for i, (wx, wy) in enumerate(self.waypoints):
                if rospy.is_shutdown():
                    break
                
                # 웨이포인트 이동
                self._goto(wx, wy, self.alt, label="WP [%d/%d]" % (i + 1, total))
                
                # --- RViz 도달한 웨이포인트 회색 마커로 변경 ---
                self.painter.mark_waypoint_done(i, self.waypoints, z=self.alt)

            # 6) 귀환 + 착륙
            rospy.loginfo("[SvgMission] 경로 완료! 원점 귀환...")
            self._goto(0.0, 0.0, self.alt, timeout=30.0, label='귀환', yaw=0.0)

            rospy.loginfo("[SvgMission] AUTO.LAND 전환...")
            self.set_mode_srv(custom_mode='AUTO.LAND')

            self._wait_for_landing(timeout=60.0)
            rospy.loginfo("[SvgMission] 미션 종료")

        finally:
            self._streaming = False


if __name__ == '__main__':
    try:
        node = SvgMissionNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[SvgMission] 노드 종료")