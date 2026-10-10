# 프로젝트 개발 히스토리 (History)

본 문서는 NUCLEO-H743ZI2 및 X-NUCLEO-IKS01A3 기반 Rust 임베디드 펌웨어 프로젝트의 일자별 깃 커밋 내역과 주요 엔지니어링 마일스톤을 기록한다.

---

## 2026.10.11

- **공용 미들웨어 크레이트 `crates/zenoh-ros2` 승격 및 OCP 리팩토링 (`39d5445`)**:
  - **독립 크레이트 추출**: STM32/NUCLEO 의존성이 0%인 순수 `no_std`, Zero-allocation ROS 2 클라이언트(`crates/zenoh-ros2`) 신설 (`wire`, `cdr`, `traits`, `types`).
  - **엔티티 캡슐화**: 단조 증가 시퀀스 번호(`seq: i64`) 은닉 `Publisher<T>`, 토픽 매칭 `Subscriber<T>`, 쿼리어블 `ServiceServer<S>`, 자율 등록 `DiscoveryRegistry` 구현.
  - **단일 CPU 최적화 태스크 분리**: `task_sub_cmd_vel` 및 `task_srv_set_led`를 각자의 `loop`를 소유한 `embassy_executor::task`로 분리하고 비동기 `Channel` 연동.
  - **전수 검증**: 호스트 단위 테스트(3/3 PASS), 크로스 컴파일(0 warning), 보드 실기 플래시 및 Docker E2E 하네스 무회귀(Zero-Loss) 100% 통과.

---

## 2026.10.10

- **06_ros2_node 임베디드 ROS 2 노드 구현 및 버그 수정 (`86c5ae1`, `7e4d19a`, `15a86bb`, `2892a61`)**:
  - **Zenoh 1.0 UDP 직통 스택**: 별도 중계 데몬 없이 ROS 2 Jazzy `rmw_zenoh_cpp`와 직접 UDP 통신하는 5대 센서 텔레메트리 및 서비스 노드 구현.
  - **프로토콜 결함 수정**: 토픽별 독립 GID/시퀀스 분리로 `A message was lost!!!` 해결, cmd_vel 구독자 토큰(MS) 등록으로 `pub --once` 매칭 대기 결함 해결.
  - **Docker 격리 검증 하네스**: 호스트 브링업 컨테이너 및 100Hz 주기/지터, 서비스 응답 자동화 E2E 테스트(`run_test.sh`) 구축.
- **05_mixed_cpp_legacy Rust + C++ 혼합 빌드 파이프라인 (`fa87139`, `518ab68`)**:
  - `build.rs` + `clang++`/`arm-none-eabi-g++` 기반 Cortex-M7 Hard-float 크로스 컴파일 구축.
  - 2차 IIR Biquad 저역 통과 필터 C++ 클래스, Zero-Allocation C-ABI 브리지 및 Rust Safe RAII 래퍼 연동 검증.
- **워크스페이스 전역 리팩토링 및 정적 분석 (`30f5e71`, `65465c3`, `e340591`)**:
  - 워크스페이스 전역 매직 넘버, 정수 절삭 및 블로킹 I2C 제거, `clippy` 린트 적용 및 SSOT 원칙 확립.
- **CPU 부하 프로파일링 및 InEKF 차등 검증 (`38b7b18`, `f1f0541`, `211e4c4`)**:
  - Zero-Fork DWT 유휴 역산(Idle Inversion) 계측 및 리눅스 `/proc/stat` 1kHz 틱 샘플링 엔진 구현.
  - `nalgebra` 참조 모델 대비 $SO(3)$ 리 군, 조셉 공분산, 촐레스키 양정치성 19개 오라클 차등 테스트 100% 통과.

---

## 2026.10.09

- **프로젝트 초기화 및 멀티 크레이트 아키텍처 수립 (`f93bdf7`, `b813668`, `64dd3b9`)**:
  - NUCLEO-H743ZI2 및 IKS01A3 툴체인(`thumbv7em-none-eabihf`), 공용 BSP(`crates/nucleo-bsp`) 및 거버넌스(`AGENTS.md`) 구축.
- **01_blinky**: 온보드 3색 LED 순차 점멸, `defmt` + RTT 로깅 파이프라인 및 Embassy 비동기 스케줄링 검증.
- **02_sensor_all_sampling**: 6종 센서 I2C 허브, LSM6DSO 416Hz 오버샘플링/LPF2 안티-에일리어싱, 선점형 `InterruptExecutor` 100Hz RT 제어(지터 0 µs) 달성.
- **03_sensor_web_dashboard**: LAN8742A RMII 이더넷 드라이버, DHCPv4 IP 자동 연동 및 실시간 텔레메트리 포트 80 HTTP/REST 웹서버 서빙.
- **04_ahrs_so3_inekf**: $SO(3)$ 우불변 InEKF 9축 자세 추정기, 정지 감지(ZARU), 1D 지자기 디커플링 및 WebGL 3D 쿼터니언 대시보드 구현.
