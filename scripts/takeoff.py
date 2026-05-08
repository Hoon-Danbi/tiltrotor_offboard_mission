#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
드론 이륙 노드
- MAVROS를 통해 PX4 드론에 시동을 걸고 이륙시키는 노드
"""

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL


class DroneTakeoff:
    def __init__(self):
        rospy.init_node('drone_takeoff_node', anonymous=True)
        
        # 현재 드론 상태
        self.current_state = State()
        
        # 상태 토픽 구독
        self.state_sub = rospy.Subscriber(
            '/mavros/state', State, self.state_callback
        )
        
        # 서비스 클라이언트
        rospy.loginfo("MAVROS 서비스 대기 중...")
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        rospy.wait_for_service('/mavros/cmd/takeoff')
        
        self.arming_client = rospy.ServiceProxy(
            '/mavros/cmd/arming', CommandBool
        )
        self.set_mode_client = rospy.ServiceProxy(
            '/mavros/set_mode', SetMode
        )
        self.takeoff_client = rospy.ServiceProxy(
            '/mavros/cmd/takeoff', CommandTOL
        )
        
        rospy.loginfo("MAVROS 서비스 연결 완료!")
    
    def state_callback(self, msg):
        """드론 상태 콜백"""
        self.current_state = msg
    
    def wait_for_connection(self):
        """FCU 연결 대기"""
        rospy.loginfo("FCU 연결 대기 중...")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.current_state.connected:
            rate.sleep()
        rospy.loginfo("FCU 연결 성공!")
    
    def set_mode(self, mode):
        """비행 모드 변경"""
        rospy.loginfo("모드 변경: %s" % mode)
        try:
            response = self.set_mode_client(custom_mode=mode)
            if response.mode_sent:
                rospy.loginfo("모드 변경 성공: %s" % mode)
                return True
            else:
                rospy.logwarn("모드 변경 실패")
                return False
        except rospy.ServiceException as e:
            rospy.logerr("서비스 호출 실패: %s" % e)
            return False
    
    def arm(self):
        """시동 걸기"""
        rospy.loginfo("시동 ON 시도")
        try:
            response = self.arming_client(value=True)
            if response.success:
                rospy.loginfo("시동 ON 성공!")
                return True
            else:
                rospy.logwarn("시동 ON 실패")
                return False
        except rospy.ServiceException as e:
            rospy.logerr("서비스 호출 실패: %s" % e)
            return False
    
    def takeoff(self, altitude=5.0):
        """이륙"""
        rospy.loginfo("이륙 명령: %.1fm" % altitude)
        try:
            response = self.takeoff_client(
                min_pitch=0.0,
                yaw=0.0,
                latitude=0.0,
                longitude=0.0,
                altitude=altitude
            )
            if response.success:
                rospy.loginfo("이륙 명령 성공!")
                return True
            else:
                rospy.logwarn("이륙 명령 실패")
                return False
        except rospy.ServiceException as e:
            rospy.logerr("서비스 호출 실패: %s" % e)
            return False
    
    def run(self):
        """메인 실행"""
        # 1. FCU 연결 대기
        self.wait_for_connection()
        
        # 2. 잠시 대기 (시스템 안정화)
        rospy.sleep(2)
        
        # 4. 시동
        if not self.arm():
            rospy.logerr("시동 실패. 종료.")
            return
        
        # 3. AUTO.TAKEOFF 모드로 변경
        if not self.set_mode("AUTO.TAKEOFF"):
            rospy.logerr("모드 변경 실패. 종료.")
            return
        
        rospy.sleep(1)
        
        
        rospy.loginfo("=" * 40)
        rospy.loginfo("이륙 시작! 가제보 화면을 확인하세요 🚁")
        rospy.loginfo("=" * 40)
        
        # 5. 상태 모니터링
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rospy.loginfo("상태 - armed: %s, mode: %s" % (
                self.current_state.armed,
                self.current_state.mode
            ))
            rate.sleep()


if __name__ == '__main__':
    try:
        drone = DroneTakeoff()
        drone.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("노드 종료")
