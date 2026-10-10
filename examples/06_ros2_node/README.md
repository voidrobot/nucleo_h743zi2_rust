---
title: "NUCLEO-H743ZI2 임베디드 ROS 2 노드 펌웨어 및 Docker 하네스 (06_ros2_node)"
source: "examples/06_ros2_node"
created: "2026-10-10 16:50:00"
modified: "2026-10-11 02:25:00"
description: "온보드 이더넷과 Zenoh 1.0 프로토콜을 결합하여 NUCLEO-H743ZI2를 완전한 ROS 2 독립 센서/제어 노드로 구동하고, zenoh-ros2 크레이트 기반의 멀티태스크 구조와 Docker 격리 하네스로 검증하는 임베디드 예제"
tags:
  - "embedded-rust"
  - "ros2"
  - "zenoh"
  - "rmw_zenoh"
  - "inekf"
  - "docker"
  - "no_std"
  - "harness"
  - "zenoh-ros2"
---

# NUCLEO-H743ZI2 임베디드 ROS 2 노드 펌웨어 및 Docker 하네스 (06_ros2_node)

## 1. 개요 및 배경 (Overview & Context)

### ① 배경 및 목적
본 예제는 NUCLEO-H743ZI2 개발 보드 및 X-NUCLEO-IKS01A3 센서 쉴드 기반으로, 보드 자체를 로봇 시스템의 **독립적인 1등 시민(First-Class Citizen) ROS 2 센서 및 제어 노드**로 승격시키는 임베디드 펌웨어이다.

전통적인 micro-ROS처럼 호스트 PC에 무거운 `micro_ros_agent` 데몬을 상시 유지할 필요 없이, 최신 **ROS 2 Jazzy의 공식 미들웨어인 Zenoh(`rmw_zenoh_cpp`)**를 활용하여 **호스트 브리지 없는 직접 통신(Bridge-less / Peer-to-Peer)**을 실현한다.

### ② 해결 과제
1. **호스트 OS 무오염 원칙 (Zero Host Contamination)**:
   - 호스트 머신에 수 기가바이트의 무거운 ROS 2 패키지나 apt 의존성을 일체 설치하지 않는다.
   - 모든 ROS 2 검증 및 런타임 실행은 `test_host/Dockerfile` 기반의 격리된 OCI/Docker 컨테이너 내부에서만 수행한다.
2. **빌드 부산물 호스트 유출 차단 (Clean Workspace)**:
   - 컨테이너 실행 시 `test_host/src`만 컨테이너 내부 `/ros2_ws/src`로 마운트하여, `colcon build` 시 생성되는 `build/`, `install/`, `log/` 디렉토리는 컨테이너 임시 계층에만 격리한다.
3. **네임스페이스 및 도메인 격리 (Strict Namespacing & Domain ID)**:
   - 다중 센서/로봇 환경에서의 토픽 및 서비스 충돌을 방지하기 위해 모든 엔드포인트는 **`/nucleo/`** 네임스페이스를 기본 적용한다.
   - 펌웨어 내 SSOT 상수 `ROS_DOMAIN_ID`를 Zenoh 키 표현식의 최상위 계층으로 직접 매핑하여 논리적 도메인 격리를 보장한다.
4. **미들웨어 계층의 완전 분리 (`zenoh-ros2`)**:
   - 저수준 프로토콜 직렬화 및 엔티티 상태 머신 코드를 공용 라이브러리 크레이트(`crates/zenoh-ros2`)로 분리하여 비즈니스 로직(센서 획득 및 액추에이터 제어)의 응집도를 극대화한다.

---

## 2. 시스템 아키텍처 및 데이터 흐름 (Architecture & Data Flow)

```mermaid
flowchart TD
    subgraph Target ["NUCLEO-H743ZI2 (Rust Firmware: examples/06_ros2_node)"]
        subgraph HardwareSense ["센서 및 제어 하드웨어 (nucleo-bsp)"]
            Sensors["X-NUCLEO-IKS01A3 (6종 센서)"] -->|"I2C1 Fast Mode (400kHz)"| InEKF_Task["SO(3) InEKF (100Hz RT 선점 NVIC P6)"]
            Sensors -->|"I2C1 폴링"| EnvSense["환경 센서 태스크 (10Hz / 1Hz)"]
            LED["온보드 LED (LD1 Green)"]
        end

        subgraph Middleware ["공용 미들웨어 (crates/zenoh-ros2)"]
            Registry["DiscoveryRegistry<br/>(@ros2_lv 일괄 등록)"]
            Pub_IMU["Publisher&lt;Imu&gt; (100Hz)"]
            Pub_Mag["Publisher&lt;MagneticField&gt; (10Hz)"]
            Pub_Env["Publisher&lt;FluidPressure, Temp, Hum&gt; (1Hz)"]
            Sub_Vel["Subscriber&lt;Twist&gt;"]
            Srv_LED["ServiceServer&lt;SetBool&gt;"]
            Wire["ZenohWire (FRAME / PUSH / PUT / REPLY)"]
            CDR["no_std CdrWriter / CdrReader"]
        end

        subgraph AsyncConcurrency ["Embassy 비동기 채널 및 독립 태스크"]
            InEKF_Task -->|"원자적 스냅샷 갱신"| Snap_IMU["IMU_SNAPSHOT"]
            EnvSense -->|"원자적 스냅샷 갱신"| Snap_ENV["ENV_SNAPSHOT"]
            
            TxTask["Zenoh 송신 태스크 (100Hz / 10Hz / 1Hz)"]
            RxTask["Zenoh UDP 수신 디스패처 (Port 7447)"]
            
            SubTask["독립 subscriber_task<br/>(cmd_vel 속도 수신)"]
            SrvTask["독립 service_task<br/>(set_led 서비스 서버)"]
            
            Chan_Cmd["Channel&lt;Twist, 4&gt;"]
            Chan_Srv["Channel&lt;ServiceReq, 2&gt;"]
        end

        Snap_IMU & Snap_ENV --> TxTask
        TxTask --> Pub_IMU & Pub_Mag & Pub_Env
        Pub_IMU & Pub_Mag & Pub_Env --> Wire

        RxTask -->|"Twist 패킷 분기"| Chan_Cmd --> SubTask
        RxTask -->|"SetBool 요청 분기"| Chan_Srv --> SrvTask
        
        SubTask -->|"속도 파싱"| RTT_Log["RTT 콘솔 로그 출력"]
        SrvTask -->|"GPIO 토글"| LED
        SrvTask -->|"REPLY 프레임 송출"| Wire
    end

    subgraph Network ["물리 이더넷 링크 (LAN8742A RMII)"]
        Wire <===>|"Zenoh UDP 와이어 프로토콜 (Key: 0/nucleo/...)"| HostNet["호스트 물리 네트워크 (--net=host)"]
    end

    subgraph HostContainer ["Docker 격리 컨테이너 (ros:jazzy-ros-base + rmw_zenoh_cpp)"]
        HostNet <===> ZenohRMW["rmw_zenoh_cpp (ROS_DOMAIN_ID=0)"]
        ZenohRMW <===> Node["host_bringup: ahrs_verifier_node"]
        Node -->|"토픽 5종 수신 및 주기 지터 검증"| TestReport["통합 테스트 판정 (PASS/FAIL)"]
        Node -->|"/nucleo/cmd_vel 속도 명령 발행"| Sub_Vel
        Node -->|"/nucleo/set_led 서비스 호출"| Srv_LED
        ZenohRMW -.->|"sensor_msgs/msg/Imu"| RViz["RViz2 3D 디스플레이 (선택 실행)"]
    end
```

---

## 3. 핵심 구현 메커니즘 (Key Implementation Mechanisms)

### ① `crates/zenoh-ros2` 미들웨어 연동
본 예제는 저수준 직렬화 및 와이어 조작 코드를 직접 포함하지 않고, 전용 라이브러리인 `zenoh-ros2` 크레이트를 의존하여 통신을 수행한다:
- **`Publisher<T>`**: 16바이트 GID와 단조 증가 시퀀스 번호(`seq: i64`)를 내부 상태로 캡슐화하여, 멀티플렉싱 환경에서 시퀀스가 뒤엉켜 `A message was lost!` 경고가 발생하는 결함을 원천 방지한다.
- **`Subscriber<T>`**: 와이어 페이로드에서 토픽 키 매칭 및 CDR 역직렬화를 수행한다.
- **`ServiceServer<S>`**: Queryable 선언 및 클라이언트의 Request 파싱, Response 패킹, Zenoh Reply 프레임 합성을 전담한다.
- **`DiscoveryRegistry`**: OCP(개방-폐쇄 원칙) 기반으로 모든 엔드포인트의 ROS 2 Liveliness Token(`@ros2_lv/...`)을 단일 레지스트리에 등록하고 부팅 시 일괄 선언한다.

### ② ROS 2 Jazzy `rmw_zenoh_cpp` 와이어 키 및 토큰 규격
ROS 2 Jazzy 그래프에서 정식 엔드포인트로 인식되기 위해 아래와 같은 정식 와이어 키 및 토큰 형식을 준수한다:
- **토픽 키 형식**: `<DOMAIN_ID>/<TOPIC_NAME>/<TYPE_NAME>/<TYPE_HASH>`
  - 예: `0/nucleo/imu/data/sensor_msgs::msg::dds_::Imu_/RIHS01_...`
- **서비스 키 형식**: `<DOMAIN_ID>/<SERVICE_NAME>/<TYPE_NAME>/<TYPE_HASH>`
  - 예: `0/nucleo/set_led/example_interfaces::srv::dds_::SetBool_/RIHS01_...`
- **Liveliness 토큰 형식**: `@ros2_lv/<DOMAIN_ID>/<GID>/<SEQ>/<ENTITY_TYPE>/<NODE_NS>/<NODE_NAME>/<TOPIC_NAME>/<TYPE_NAME>/<TYPE_HASH>/<QOS>`
  - `ENTITY_TYPE`: `NN` (Node Name), `MP` (Message Publisher), `MS` (Message Subscriber), `SS` (Service Server)

### ③ 비동기 채널 기반 멀티태스크 동시성 아키텍처 (`src/main.rs`)
- **100 Hz RT 선점 인터럽트 (`Priority::P6`)**: InEKF 칼만 필터를 단 30 $\mu\text{s}$ 만에 연산하고 원자적 스냅샷(`IMU_SNAPSHOT`)을 갱신.
- **100 Hz Zenoh UDP 송신 태스크**: 스냅샷을 읽어 `Publisher<T>`로 CDR 패킷을 조립하고 UDP 브로드캐스트로 고속 송출.
- **수신 디스패처 및 독립 비동기 태스크 분리**:
  - UDP RX 루프는 수신 패킷의 키를 식별하여 `CMD_VEL_CHANNEL` 또는 `SERVICE_CHANNEL`로 라우팅한다.
  - **`subscriber_task`**: `CMD_VEL_CHANNEL`로부터 `Twist` 메시지를 수신하여 비동기로 파싱하고 RTT 콘솔로 출력.
  - **`service_task`**: `SERVICE_CHANNEL`로부터 `SetBool` 요청을 수신하여 온보드 LED(LD1 Green)를 제어한 뒤 즉시 Zenoh REPLY 프레임을 합성하여 클라이언트로 응답.
- **1 kHz TIM7 틱 프로파일러 (`Priority::P5`)**: 15ns 초경량 통계 카운터로 CPU 점유율을 실시간 계측.

### ④ Docker 격리형 정식 ROS 2 패키지 (`test_host/`)
- `host_bringup`: `package.xml`, `setup.py`, `ahrs_verifier_node.py`를 갖춘 정식 ROS 2 파이썬 패키지.
- `run_test.sh`: 호스트 네트워크(`--net=host`)를 공유하여 컨테이너 내부에서 `colcon build` 후 자동화 테스트를 원스톱으로 실행.

---

## 4. 엔지니어링 트레이드오프 및 인사이트 (Trade-offs & Insights)

| 구분 | 장점 (Pros) | 한계 및 주의점 (Cons & Constraints) |
| :--- | :--- | :--- |
| **Zenoh (rmw_zenoh)** | - 호스트 브리지 데몬 완전 불필요<br>- 와이어 오버헤드 5바이트 (DDS 대비 90% 이상 절감)<br>- 100~500Hz 초고속 제어 지원 | - ROS 2 Jazzy 이상 최신 배포판 권장<br>- Domain ID 불일치 시 패킷 자동 드롭 (일치 필수) |
| **zenoh-ros2 크레이트 분리** | - 프로토콜 직렬화와 애플리케이션 제어 로직의 완전한 관심사 분리<br>- 순수 no_std 기반으로 호스트 단위 테스트 가능 | - 엔티티 추가 시 Liveliness Token 및 Type Hash 메타데이터 동기화 필요 |
| **비동기 채널 분리 태스크** | - 수신 디스패처가 블로킹되지 않고 즉시 다음 패킷 수신 대기 가능<br>- 속도 명령 수신과 서비스 응답 처리의 독립적 주기 보장 | - 비동기 채널 큐 크기(`Channel<..., N>`) 초과 시 백프레셔 고려 필요 |
| **Docker 격리 하네스** | - 호스트 OS 설치 0, 빌드 부산물 유출 0<br>- 정식 ROS 2 노드와의 100% 호환성 보증 | - 도커 데몬 실행 권한 필요 (`docker run`) |

---

## 5. 빌드 및 자동 테스트 가이드  

### ① 타깃 펌웨어 빌드 및 플래시
```bash
# NUCLEO 보드 연결 후 빌드, 플래시 및 RTT 로깅 실행
cargo run -p ros2_node_06 --target thumbv7em-none-eabihf
# 또는 릴리스 최적화 모드로 플래시:
cargo run -p ros2_node_06 --target thumbv7em-none-eabihf --release
```

### ② 호스트 Docker 원스톱 자동화 검증
```bash
# 호스트 터미널에서 단 한 줄로 전체 ROS 2 통신 검증 (토픽 5종, cmd_vel, set_led)
./examples/06_ros2_node/test_host/run_test.sh
```

### ③ RViz2 3D 실시간 자세 시각화 (선택 실행)
```bash
# 호스트 X11 화면으로 RViz2 3D 디스플레이 구동
./examples/06_ros2_node/test_host/rviz2_view.sh
```

---

## 6. 수동 테스트 가이드 (Manual CLI Inspection)

자동화 검증 스크립트 외에 호스트 OS를 오염시키지 않고 Docker 대화형 셸 내부로 진입하여, 표준 `ros2` CLI 도구를 통해 토픽 스트림을 확인하고 서비스/토픽을 수동으로 조작하는 절차이다.

### ① 대화형 테스트 컨테이너 진입 및 Zenoh 라우터 구동
호스트 터미널에서 다음 명령을 실행하여 Docker 대화형 셸을 열고, 패키지 빌드 및 Zenoh 라우터 데몬(`rmw_zenohd`)을 백그라운드로 구동한다:

```bash
# 1. 호스트 머신에서 Docker 대화형 셸 진입 (네트워크 호스트 모드 공유)
docker run --rm -it --net=host \
  -v "$(pwd)/examples/06_ros2_node/test_host/src:/ros2_ws/src" \
  -w /ros2_ws \
  -e ROS_DOMAIN_ID=0 \
  -e RMW_IMPLEMENTATION=rmw_zenoh_cpp \
  ros2_node_test:latest bash

# 2. 컨테이너 내부 환경 설정 및 패키지 빌드
source /opt/ros/jazzy/setup.bash
colcon build
source /ros2_ws/install/setup.bash

# 3. Zenoh 라우터 데몬 백그라운드 구동
export ZENOH_ROUTER_CONFIG_URI=/ros2_ws/src/host_bringup/config/router_config.json5
ros2 run rmw_zenoh_cpp rmw_zenohd > /tmp/zenohd.log 2>&1 &
```

### ② 노드 및 토픽/서비스 목록 확인
라우터 데몬이 구동되면 NUCLEO-H743ZI2 보드가 발행/수신하는 ROS 2 엔드포인트가 즉시 식별된다:

```bash
# 1. 활성 노드 목록 확인
ros2 node list
# 기대 출력:
# /nucleo_h743zi2

# 2. 활성 토픽 목록 확인
ros2 topic list
# 기대 출력:
# /nucleo/cmd_vel
# /nucleo/humidity
# /nucleo/imu/data
# /nucleo/imu/mag
# /nucleo/pressure
# /nucleo/temperature

# 3. 서비스 목록 확인
ros2 service list
# 기대 출력:
# /nucleo/set_led
```

### ③ 센서 토픽 수신 및 주기 계측 (Echo & Hz)
```bash
# 1. 100 Hz 하드 실시간 IMU 쿼터니언/가속도/각속도 데이터 스트림 확인
ros2 topic echo /nucleo/imu/data

# 2. IMU 데이터 발행 주기 실측 (기대 주기: ~100 Hz, 약 10 ms)
ros2 topic hz /nucleo/imu/data

# 3. 10 Hz 지자기 센서 데이터 스트림 확인
ros2 topic echo /nucleo/imu/mag

# 4. 1 Hz 대기압, 온도, 습도 센서 데이터 확인
ros2 topic echo /nucleo/pressure
ros2 topic echo /nucleo/temperature
ros2 topic echo /nucleo/humidity
```

### ④ 온보드 액추에이터 제어 (Service Call & Pub)
수동으로 NUCLEO 보드의 온보드 LED를 제어하거나 주행 속도 명령을 전송하여 하드웨어 응답을 검증한다:

```bash
# 1. 온보드 Green LED (LD1) 점등 서비스 호출
ros2 service call /nucleo/set_led example_interfaces/srv/SetBool "{data: true}"
# 기대 응답:
# response: example_interfaces.srv.SetBool_Response(success=True, message='LED ON')

# 2. 온보드 Green LED (LD1) 소등 서비스 호출
ros2 service call /nucleo/set_led example_interfaces/srv/SetBool "{data: false}"
# 기대 응답:
# response: example_interfaces.srv.SetBool_Response(success=True, message='LED OFF')

# 3. 로봇 주행 속도 명령 발행 (NUCLEO RTT 콘솔에서 실시간 파싱 및 수신 로그 확인)
ros2 topic pub --once /nucleo/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"
```

---

## 7. 관련 문서 및 소스코드 참조 (References)

- [main.rs](src/main.rs): `06_ros2_node` 메인 펌웨어 및 Embassy 비동기 멀티태스크
- [msg.rs](src/msg.rs): NUCLEO 센서 전용 ROS 2 메시지 정의 및 토픽 메타데이터 SSOT
- [srv.rs](src/srv.rs): ROS 2 서비스 Request/Response 정의 및 서비스 메타데이터 SSOT
- [zenoh-ros2 크레이트](../../crates/zenoh-ros2/README.md): 임베디드 순수 Rust Zenoh 1.0 / ROS 2 미들웨어 명세서
- [so3-inekf 크레이트](../../crates/so3-inekf/README.md): $SO(3)$ 우불변 InEKF 수학 코어
- [nucleo-bsp 크레이트](../../crates/nucleo-bsp/README.md): 온보드 핀아웃 및 센서 레지스터 맵
- [Docker 테스트 하네스](test_host/run_test.sh): 원스톱 자동화 검증 스크립트
