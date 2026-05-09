#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
rviz_painter.py
---------------
RViz 마커 시각화 전용 모듈. svg_mission.py 및 다른 ROS 노드에서
재사용 가능하도록 독립 클래스로 설계.

기능:
  - 웨이포인트 구체(Sphere) 표시
  - 계획된 궤적 라인 표시
  - 드론이 실제 이동한 경로(잔상) 라인 + 점 표시
  - 현재 위치 화살표(Arrow) 표시
  - 텍스트 레이블 표시
  - 모든 마커는 RViz 종료 전까지 유지 (LIFETIME = 0 → 영구)

사용 예시:
    painter = RVizPainter(frame_id='map')
    painter.draw_waypoints(wp_list)          # [(x,y,z), ...]
    painter.draw_planned_path(wp_list)       # 계획 경로 라인
    painter.update_trail(x, y, z)           # 매 pose 콜백에서 호출
    painter.draw_current_pose(x, y, z, yaw) # 현재 위치 화살표
"""

import math
import threading
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


# ────────────────────────────────────────────────────────────────────
#  색상 팔레트 (RGBA, 0.0~1.0)
# ────────────────────────────────────────────────────────────────────
class Color:
    """자주 쓰는 RGBA 색상 상수."""
    @staticmethod
    def rgba(r, g, b, a=1.0):
        c = ColorRGBA()
        c.r, c.g, c.b, c.a = r, g, b, a
        return c

    # 웨이포인트 구체
    WAYPOINT       = None   # 초기화 후 설정
    WAYPOINT_DONE  = None
    # 계획 경로 라인
    PLANNED_PATH   = None
    # 실제 이동 잔상
    TRAIL_LINE     = None
    TRAIL_DOT      = None
    # 현재 위치 화살표
    ARROW          = None
    # 텍스트
    TEXT           = None
    # 목표 setpoint
    TARGET         = None

def _init_colors():
    Color.WAYPOINT      = Color.rgba(0.2, 0.8, 1.0, 0.85)   # 하늘색
    Color.WAYPOINT_DONE = Color.rgba(0.4, 0.4, 0.4, 0.5)    # 회색 (통과한 WP)
    Color.PLANNED_PATH  = Color.rgba(0.0, 1.0, 0.5, 0.6)    # 초록
    Color.TRAIL_LINE    = Color.rgba(1.0, 0.6, 0.0, 0.9)    # 주황
    Color.TRAIL_DOT     = Color.rgba(1.0, 0.3, 0.3, 1.0)    # 빨강
    Color.ARROW         = Color.rgba(1.0, 1.0, 0.0, 1.0)    # 노랑
    Color.TEXT          = Color.rgba(1.0, 1.0, 1.0, 1.0)    # 흰색
    Color.TARGET        = Color.rgba(0.0, 0.5, 1.0, 0.7)    # 파랑

_init_colors()


# ────────────────────────────────────────────────────────────────────
#  마커 ID 관리
# ────────────────────────────────────────────────────────────────────
class _IdPool:
    """
    네임스페이스별 마커 ID 풀.
    각 네임스페이스는 독립적인 ID 공간을 가짐.
    """
    def __init__(self):
        self._counters = {}

    def next(self, ns):
        self._counters[ns] = self._counters.get(ns, -1) + 1
        return self._counters[ns]

    def current(self, ns):
        return self._counters.get(ns, 0)


# ────────────────────────────────────────────────────────────────────
#  RVizPainter 메인 클래스
# ────────────────────────────────────────────────────────────────────
class RVizPainter(object):
    """
    RViz MarkerArray 기반 시각화 클래스.

    Parameters
    ----------
    frame_id : str
        마커 기준 좌표계. 보통 'map' 또는 'odom'.
    topic : str
        publish할 MarkerArray 토픽 이름.
    trail_min_dist : float
        잔상 점을 추가할 최소 이동 거리 [m]. 너무 잦은 추가 방지.
    trail_max_points : int
        잔상 라인 최대 점 수. 초과 시 가장 오래된 점 삭제.
        0이면 무제한.
    """

    # 네임스페이스 상수
    NS_WAYPOINT    = 'waypoints'
    NS_PLANNED     = 'planned_path'
    NS_TRAIL_LINE  = 'trail_line'
    NS_TRAIL_DOTS  = 'trail_dots'
    NS_ARROW       = 'drone_arrow'
    NS_TEXT        = 'labels'
    NS_TARGET      = 'target_setpoint'

    def __init__(self,
                 frame_id='map',
                 topic='/svg_mission/markers',
                 trail_min_dist=0.1,
                 trail_max_points=0):

        self.frame_id        = frame_id
        self.trail_min_dist  = trail_min_dist
        self.trail_max_points = trail_max_points

        self._pub   = rospy.Publisher(topic, MarkerArray, queue_size=10, latch=True)
        self._lock  = threading.Lock()
        self._ids   = _IdPool()

        # 활성 마커 딕셔너리: id → Marker (RViz 재전송용)
        self._active = {}

        # 잔상 내부 상태
        self._trail_points   = []   # [geometry_msgs/Point, ...]
        self._trail_last_pos = None  # (x, y, z)

        # 계획 경로 라인 마커 ID (업데이트용)
        self._planned_line_id = None
        # 현재 위치 화살표 마커 ID
        self._arrow_id        = None
        # 목표 setpoint 마커 ID
        self._target_id       = None

        rospy.loginfo("[RVizPainter] 초기화 완료 (frame='%s', topic='%s')" %
                      (frame_id, topic))

    # ══════════════════════════════════════════════════════════════
    #  내부 헬퍼
    # ══════════════════════════════════════════════════════════════

    def _make_base(self, ns, marker_type, marker_id=None):
        """공통 Marker 기본 설정."""
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp    = rospy.Time.now()
        m.ns              = ns
        m.id              = marker_id if marker_id is not None else self._ids.next(ns)
        m.type            = marker_type
        m.action          = Marker.ADD
        m.lifetime        = rospy.Duration(0)   # ★ 영구 유지
        m.pose.orientation.w = 1.0
        return m

    def _point(self, x, y, z):
        p = Point()
        p.x, p.y, p.z = x, y, z
        return p

    def _publish_one(self, marker):
        """단일 마커를 활성 목록에 등록하고 publish."""
        with self._lock:
            self._active[marker.id] = marker
        arr = MarkerArray()
        arr.markers = [marker]
        try:
            self._pub.publish(arr)
        except rospy.ROSException as e:
            rospy.logwarn("[RVizPainter] publish 실패: %s" % e)

    def _publish_batch(self, markers):
        """여러 마커를 한 번에 publish."""
        with self._lock:
            for m in markers:
                self._active[m.id] = m
        arr = MarkerArray()
        arr.markers = markers
        try:
            self._pub.publish(arr)
        except rospy.ROSException as e:
            rospy.logwarn("[RVizPainter] publish 실패: %s" % e)

    def _delete_marker(self, ns, marker_id):
        """특정 마커 삭제."""
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp    = rospy.Time.now()
        m.ns     = ns
        m.id     = marker_id
        m.action = Marker.DELETE
        with self._lock:
            self._active.pop(marker_id, None)
        arr = MarkerArray()
        arr.markers = [m]
        self._pub.publish(arr)

    # ══════════════════════════════════════════════════════════════
    #  공개 API — 웨이포인트
    # ══════════════════════════════════════════════════════════════

    def draw_waypoints(self, waypoints, z=None, radius=0.25):
        """
        웨이포인트 목록을 구체로 표시.

        Parameters
        ----------
        waypoints : list of (x, y) or (x, y, z)
        z         : 고도. 각 튜플에 z가 없으면 이 값 사용.
        radius    : 구체 반지름 [m]
        """
        markers = []
        for i, wp in enumerate(waypoints):
            x, y = wp[0], wp[1]
            wz   = wp[2] if len(wp) > 2 else (z or 0.0)

            m = self._make_base(self.NS_WAYPOINT, Marker.SPHERE)
            m.pose.position   = self._point(x, y, wz)
            m.scale.x = m.scale.y = m.scale.z = radius * 2
            m.color = Color.WAYPOINT
            markers.append(m)

            # 인덱스 텍스트 레이블
            t = self._make_base(self.NS_TEXT, Marker.TEXT_VIEW_FACING)
            t.pose.position = self._point(x, y, wz + radius + 0.15)
            t.scale.z       = 0.3
            t.color         = Color.TEXT
            t.text          = str(i + 1)
            markers.append(t)

        self._publish_batch(markers)
        rospy.loginfo("[RVizPainter] 웨이포인트 %d개 표시" % len(waypoints))

    def mark_waypoint_done(self, index, waypoints, z=None):
        """
        통과한 웨이포인트를 회색으로 변경.
        draw_waypoints와 같은 순서의 인덱스를 사용.
        """
        # NS_WAYPOINT의 ID는 draw_waypoints 호출 순서대로 부여됨 (0, 1, 2, ...)
        wp = waypoints[index]
        x, y = wp[0], wp[1]
        wz   = wp[2] if len(wp) > 2 else (z or 0.0)

        m = self._make_base(self.NS_WAYPOINT, Marker.SPHERE,
                            marker_id=index)    # draw_waypoints의 ID 규칙과 일치 (0, 1, 2, ...)
        m.pose.position   = self._point(x, y, wz)
        m.scale.x = m.scale.y = m.scale.z = 0.30
        m.color = Color.WAYPOINT_DONE
        self._publish_one(m)

    # ══════════════════════════════════════════════════════════════
    #  공개 API — 계획 경로
    # ══════════════════════════════════════════════════════════════

    def draw_planned_path(self, waypoints, z=None, line_width=0.05):
        """
        계획된 경로를 연결하는 LINE_STRIP 마커 표시.
        반복 호출 시 이전 라인을 덮어씀.

        Parameters
        ----------
        waypoints  : list of (x, y) or (x, y, z)
        z          : 각 튜플에 z가 없을 때 사용할 기본 고도
        line_width : 선 굵기 [m]
        """
        if self._planned_line_id is None:
            self._planned_line_id = self._ids.next(self.NS_PLANNED)

        m = self._make_base(self.NS_PLANNED, Marker.LINE_STRIP,
                            marker_id=self._planned_line_id)
        m.scale.x = line_width
        m.color   = Color.PLANNED_PATH

        for wp in waypoints:
            x, y = wp[0], wp[1]
            wz   = wp[2] if len(wp) > 2 else (z or 0.0)
            m.points.append(self._point(x, y, wz))

        self._publish_one(m)
        rospy.loginfo("[RVizPainter] 계획 경로 라인 표시 (%d점)" % len(waypoints))

    # ══════════════════════════════════════════════════════════════
    #  공개 API — 실제 이동 잔상
    # ══════════════════════════════════════════════════════════════

    def update_trail(self, x, y, z,
                     line_width=0.04, dot_radius=0.06):
        """
        현재 위치를 잔상 기록에 추가하고 RViz에 업데이트.
        매 pose 콜백에서 호출해도 trail_min_dist 미만이면 무시.

        Parameters
        ----------
        x, y, z    : 현재 위치 (ENU)
        line_width : 잔상 라인 굵기 [m]
        dot_radius : 잔상 점 반지름 [m]
        """
        # 최소 이동 거리 필터
        if self._trail_last_pos is not None:
            lx, ly, lz = self._trail_last_pos
            d = math.sqrt((x-lx)**2 + (y-ly)**2 + (z-lz)**2)
            if d < self.trail_min_dist:
                return

        self._trail_last_pos = (x, y, z)
        new_pt = self._point(x, y, z)
        self._trail_points.append(new_pt)

        # 최대 점 수 제한
        if self.trail_max_points > 0:
            while len(self._trail_points) > self.trail_max_points:
                self._trail_points.pop(0)

        markers = []

        # ── 잔상 라인 (LINE_STRIP, 고정 ID로 매번 덮어씀) ──
        line_id = 0   # NS_TRAIL_LINE 내부에서 항상 ID=0
        lm = self._make_base(self.NS_TRAIL_LINE, Marker.LINE_STRIP,
                             marker_id=line_id)
        lm.scale.x = line_width
        lm.color   = Color.TRAIL_LINE
        lm.points  = list(self._trail_points)
        markers.append(lm)

        # ── 잔상 점 (SPHERE_LIST, 고정 ID로 매번 덮어씀) ──
        dot_id = 0    # NS_TRAIL_DOTS 내부에서 항상 ID=0
        dm = self._make_base(self.NS_TRAIL_DOTS, Marker.SPHERE_LIST,
                             marker_id=dot_id)
        dm.scale.x = dm.scale.y = dm.scale.z = dot_radius * 2
        dm.color   = Color.TRAIL_DOT
        dm.points  = list(self._trail_points)
        markers.append(dm)

        self._publish_batch(markers)

    def clear_trail(self):
        """잔상 기록 초기화."""
        self._trail_points   = []
        self._trail_last_pos = None
        self._delete_marker(self.NS_TRAIL_LINE, 0)
        self._delete_marker(self.NS_TRAIL_DOTS, 0)
        rospy.loginfo("[RVizPainter] 잔상 초기화")

    # ══════════════════════════════════════════════════════════════
    #  공개 API — 현재 위치 화살표
    # ══════════════════════════════════════════════════════════════

    def draw_current_pose(self, x, y, z, yaw=0.0, quaternion=None):
        """
        드론 현재 위치를 화살표로 표시. 매 pose 콜백에서 호출 가능.

        Parameters
        ----------
        x, y, z : 위치 (ENU)
        yaw     : 헤딩 [rad] (quaternion이 None일 때만 사용)
        quaternion : (qx, qy, qz, qw) 튜플 (지정시 yaw 무시)
        """
        if self._arrow_id is None:
            self._arrow_id = self._ids.next(self.NS_ARROW)

        m = self._make_base(self.NS_ARROW, Marker.ARROW,
                            marker_id=self._arrow_id)

        # 위치 설정
        m.pose.position = self._point(x, y, z)
        
        # orientation 설정
        if quaternion is not None:
            qx, qy, qz, qw = quaternion
            m.pose.orientation.x = qx
            m.pose.orientation.y = qy
            m.pose.orientation.z = qz
            m.pose.orientation.w = qw
        else:
            # quaternion이 없으면 yaw로 계산
            m.pose.orientation.w = math.cos(yaw * 0.5)
            m.pose.orientation.z = math.sin(yaw * 0.5)
        
        # 화살표 크기 (pose.orientation 적용 후 scale)
        m.scale.x = 0.5     # 길이
        m.scale.y = 0.1     # 샤프트 지름
        m.scale.z = 0.1     # 헤드 지름
        m.color   = Color.ARROW

        self._publish_one(m)

    # ══════════════════════════════════════════════════════════════
    #  공개 API — 목표 setpoint
    # ══════════════════════════════════════════════════════════════

    def draw_target(self, x, y, z, radius=0.2):
        """
        현재 목표 setpoint를 반투명 구체로 표시.
        매 setpoint 갱신 시 호출 가능.
        """
        if self._target_id is None:
            self._target_id = self._ids.next(self.NS_TARGET)

        m = self._make_base(self.NS_TARGET, Marker.SPHERE,
                            marker_id=self._target_id)
        m.pose.position   = self._point(x, y, z)
        m.scale.x = m.scale.y = m.scale.z = radius * 2
        m.color   = Color.TARGET
        self._publish_one(m)

    # ══════════════════════════════════════════════════════════════
    #  공개 API — 텍스트 레이블
    # ══════════════════════════════════════════════════════════════

    def draw_label(self, x, y, z, text, size=0.4):
        """
        지정 위치에 텍스트 레이블 추가.
        매 호출마다 새 ID가 부여되므로 여러 레이블 공존 가능.
        """
        m = self._make_base(self.NS_TEXT, Marker.TEXT_VIEW_FACING)
        m.pose.position = self._point(x, y, z)
        m.scale.z       = size
        m.color         = Color.TEXT
        m.text          = text
        self._publish_one(m)

    # ══════════════════════════════════════════════════════════════
    #  공개 API — 전체 삭제
    # ══════════════════════════════════════════════════════════════

    def clear_all(self):
        """모든 마커 삭제 (DELETEALL 액션 사용)."""
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp    = rospy.Time.now()
        m.action          = Marker.DELETEALL
        arr = MarkerArray()
        arr.markers = [m]
        self._pub.publish(arr)
        with self._lock:
            self._active.clear()
        self._trail_points   = []
        self._trail_last_pos = None
        rospy.loginfo("[RVizPainter] 전체 마커 삭제")

    # ══════════════════════════════════════════════════════════════
    #  공개 API — 전체 재전송 (RViz 재접속 대비)
    # ══════════════════════════════════════════════════════════════

    def republish_all(self):
        """
        활성 마커 전체를 한 번 다시 publish.
        RViz 가 재시작됐을 때 마커가 사라지는 경우 사용.
        """
        with self._lock:
            markers = list(self._active.values())
        if not markers:
            return
        arr = MarkerArray()
        arr.markers = markers
        self._pub.publish(arr)
        rospy.loginfo("[RVizPainter] 마커 %d개 재전송" % len(markers))