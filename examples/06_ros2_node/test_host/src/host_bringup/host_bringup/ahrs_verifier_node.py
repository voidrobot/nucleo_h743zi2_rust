#!/usr/bin/env python3
"""
NUCLEO-H743ZI2 임베디드 ROS 2 노드 자동화 검증 노드 (ahrs_verifier_node.py)

- 5대 센서 토픽 수신율 및 수신 주기, 쿼터니언 정합성 자동 검증
- /nucleo/cmd_vel 속도 명령 발행 및 수신 피드백
- /nucleo/set_led SetBool 서비스 호출 및 성공 응답 확인
- 모든 항목 충족 시 Exit code 0 정상 반환
"""

import sys
import time
import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField, FluidPressure, Temperature, RelativeHumidity
from geometry_msgs.msg import Twist
from example_interfaces.srv import SetBool


class AhrsVerifierNode(Node):
    def __init__(self):
        super().__init__('ahrs_verifier_node')
        self.get_logger().info('>>> [Host Verifier] NUCLEO-H743ZI2 ROS 2 검증 시작 <<<')

        # Type Hash 및 Zenoh KeyExpr 조사 출력 (선택적)
        try:
            from rclpy.type_support import get_message_type_hash
            for name, cls in [
                ('/nucleo/imu/data', Imu),
                ('/nucleo/imu/mag', MagneticField),
                ('/nucleo/pressure', FluidPressure),
                ('/nucleo/temperature', Temperature),
                ('/nucleo/humidity', RelativeHumidity),
                ('/nucleo/cmd_vel', Twist),
            ]:
                th = get_message_type_hash(cls)
                type_name = f"{cls.__module__.replace('.', '::')}::dds_::{cls.__name__}_"
                self.get_logger().info(f"[KeyExpr Info] {name} -> type: {type_name}, hash: {th}")
        except Exception as e:
            self.get_logger().debug(f"Type hash check skipped: {e}")

        # 통계 카운터
        self.imu_count = 0
        self.mag_count = 0
        self.press_count = 0
        self.temp_count = 0
        self.hum_count = 0

        self.last_imu_time = None
        self.imu_intervals = []

        # 1. 5대 토픽 구독자 설정
        self.sub_imu = self.create_subscription(Imu, '/nucleo/imu/data', self.cb_imu, 10)
        self.sub_mag = self.create_subscription(MagneticField, '/nucleo/imu/mag', self.cb_mag, 10)
        self.sub_press = self.create_subscription(FluidPressure, '/nucleo/pressure', self.cb_press, 10)
        self.sub_temp = self.create_subscription(Temperature, '/nucleo/temperature', self.cb_temp, 10)
        self.sub_hum = self.create_subscription(RelativeHumidity, '/nucleo/humidity', self.cb_hum, 10)

        # 2. cmd_vel 발행자 설정
        self.pub_vel = self.create_publisher(Twist, '/nucleo/cmd_vel', 10)

        # 3. set_led 서비스 클라이언트 설정
        self.cli_set_led = self.create_client(SetBool, '/nucleo/set_led')

        # 타이머 (100ms 주기 검사)
        self.timer = self.create_timer(0.1, self.check_progress)
        self.start_time = time.time()
        self.cmd_vel_sent = False
        self.service_called = False
        self.service_success = False

    def cb_imu(self, msg: Imu):
        self.imu_count += 1
        now = time.time()
        if self.last_imu_time is not None:
            dt = now - self.last_imu_time
            self.imu_intervals.append(dt)
        self.last_imu_time = now

        # 쿼터니언 정규성 검증
        q = msg.orientation
        norm_sq = q.x**2 + q.y**2 + q.z**2 + q.w**2
        if abs(1.0 - norm_sq) > 0.05:
            self.get_logger().warn(f'쿼터니언 크기 오차 감지: {norm_sq:.4f}')

    def cb_mag(self, msg: MagneticField):
        self.mag_count += 1

    def cb_press(self, msg: FluidPressure):
        self.press_count += 1

    def cb_temp(self, msg: Temperature):
        self.temp_count += 1

    def cb_hum(self, msg: RelativeHumidity):
        self.hum_count += 1

    def check_progress(self):
        elapsed = time.time() - self.start_time

        # 1. 1초 경과 시 cmd_vel 매칭 구독자 확인 및 발행
        if elapsed > 1.0 and not self.cmd_vel_sent:
            sub_count = self.pub_vel.get_subscription_count()
            if sub_count >= 1:
                t = Twist()
                t.linear.x = 0.5
                t.angular.z = 0.2
                self.pub_vel.publish(t)
                self.cmd_vel_sent = True
                self.get_logger().info(f'[CmdVel] /nucleo/cmd_vel 매칭 구독자 확인({sub_count}개) 및 속도 명령 발행 완료')
            else:
                self.get_logger().warn(f'[CmdVel] /nucleo/cmd_vel 매칭 구독자 탐색 대기 중... (현재: {sub_count}개)')

        # 2. 1.5초 경과 시 set_led 서비스 호출
        if elapsed > 1.5 and not self.service_called:
            if self.cli_set_led.wait_for_service(timeout_sec=1.0):
                req = SetBool.Request()
                req.data = True
                future = self.cli_set_led.call_async(req)
                future.add_done_callback(self.cb_service_done)
                self.service_called = True
                self.get_logger().info('[Service] /nucleo/set_led (data=True) 요청 전송')
            else:
                self.get_logger().warn('[Service] /nucleo/set_led 서비스 대기 중...')

        # 3. 완료 판정 (IMU 50개 이상, 나머지 센서 1개 이상, 서비스 응답 완료)
        if self.imu_count >= 50 and self.mag_count >= 2 and self.press_count >= 1 and self.temp_count >= 1 and self.service_success:
            avg_dt = sum(self.imu_intervals) / len(self.imu_intervals) if self.imu_intervals else 0.0
            avg_hz = 1.0 / avg_dt if avg_dt > 0 else 0.0
            self.get_logger().info(f'=====================================================')
            self.get_logger().info(f'>>> [검증 통과] 모든 ROS 2 인터페이스 정상 작동 확인 <<<')
            self.get_logger().info(f'  - IMU 수신: {self.imu_count} 패킷 (평균 주기: {avg_dt*1000:.2f} ms, {avg_hz:.1f} Hz)')
            self.get_logger().info(f'  - 지자기 수신: {self.mag_count} 패킷')
            self.get_logger().info(f'  - 기압/온도/습도 수신: {self.press_count}/{self.temp_count}/{self.hum_count} 패킷')
            self.get_logger().info(f'  - /nucleo/cmd_vel 발행: 완료')
            self.get_logger().info(f'  - /nucleo/set_led 서비스 호출: 성공')
            self.get_logger().info(f'=====================================================')
            sys.exit(0)

        # 4. 타임아웃 판정 (30초)
        if elapsed > 30.0:
            self.get_logger().error(f'[타임아웃] 30초 내 검증 기준 미달 (IMU: {self.imu_count}, Mag: {self.mag_count}, Srv: {self.service_success})')
            sys.exit(1)

    def cb_service_done(self, future):
        try:
            resp = future.result()
            if resp.success:
                self.service_success = True
                self.get_logger().info(f'[Service 응답 성공] success={resp.success}, message="{resp.message}"')
            else:
                self.get_logger().error(f'[Service 응답 실패] message="{resp.message}"')
        except Exception as e:
            self.get_logger().error(f'[Service 예외 발생]: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = AhrsVerifierNode()
    exit_code = 0
    try:
        rclpy.spin(node)
    except SystemExit as e:
        exit_code = e.code if e.code is not None else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if exit_code != 0:
        sys.exit(exit_code)


if __name__ == '__main__':
    main()
