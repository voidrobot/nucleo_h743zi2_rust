# NUCLEO-H743ZI2 Rust 임베디드 펌웨어 프로젝트

본 저장소는 STMicroelectronics의 **NUCLEO-H743ZI2** 개발 보드와 **X-NUCLEO-IKS01A3** 모션 MEMS 및 환경 센서 확장 보드를 기반으로, 현대적인 Rust 임베디드 생태계(`no_std`, `embassy`, `embedded-hal`)를 활용한 고성능 센서 제어 및 펌웨어 예제를 체계적으로 개발하고 검증하는 프로젝트이다.

---

## 1. 대상 하드웨어 구성 (Target Hardware)

### ① 메인보드: NUCLEO-H743ZI2
- **MCU**: STM32H743ZI (Arm® 32-bit Cortex®-M7 with double-precision FPU, L1 캐시, 최대 480 MHz 동작)
- **메모리**: 2 MB Dual-Bank Flash, 1 MB RAM (TCM RAM, AXI SRAM, SRAM1~4, Backup SRAM 분할 구조)
- **온보드 디버거**: ST-LINK/V3E (SWD/JTAG 디버깅, 가상 COM 포트, RTT 통신 지원)
- **사용자 I/O**: 사용자 LED 3개 (Green, Yellow, Red), 사용자 푸시 버튼 1개, Reset 버튼 1개
- **커넥터**: ST Zio 확장 커넥터(Arduino Uno V3 호환) 및 ST morpho 헤더

### ② 센서 쉴드: X-NUCLEO-IKS01A3
Arduino Uno V3 핀 호환 규격으로 NUCLEO-H743ZI2 상단에 직접 적층 결합(Stackable)되는 모션 MEMS 및 환경 센서 확장 보드이다.

| 센서 칩셋 | 분류 | 주요 측정 항목 및 사양 | 통신 인터페이스 |
| :--- | :--- | :--- | :--- |
| **LSM6DSO** | 6축 IMU | 3축 가속도계(±2/±4/±8/±16 g) + 3축 자이로스코프(±125/±250/±500/±1000/±2000 dps), 온칩 머신러닝 코어(MLC) | I2C (기본) / SPI |
| **LIS2MDL** | 지자기 센서 | 3축 디지털 자기 센서(±50 gauss), 초저전력 동작 | I2C (기본) / SPI |
| **LIS2DW12** | 가속도계 | 3축 초저전력 고성능 선형 가속도계(±2/±4/±8/±16 g) | I2C (기본) / SPI |
| **LPS22HH** | 기압 센서 | 260~1260 hPa 고정밀 대기압 및 온도 측정 | I2C (기본) / SPI |
| **HTS221** | 온습도 센서 | 정전용량식 상대 습도(0~100% rH) 및 온도(-40~120 °C) 측정 | I2C |
| **STTS751** | 온도 센서 | 고정밀 저전력 디지털 온도 센서(±0.5 °C 정확도, -40~125 °C) | I2C (SMBus 호환) |
| **DIL 24-pin** | 확장 소켓 | 추가 MEMS 어댑터(STEVAL-MKIxxxV 시리즈 등) 장착용 소켓 | I2C / SPI / GPIO |

---

## 2. 소프트웨어 아키텍처 및 기술 스택 (Software Stack)

- **언어 및 런타임**: Rust (`no_std`), Target: `thumbv7em-none-eabihf` (Cortex-M7 with Hardfloat)
- **임베디드 비동기 프레임워크**: `embassy` (`embassy-stm32`, `embassy-executor`, `embassy-time`, `embassy-sync`, `embassy-net`)
- **하드웨어 추상화 계층 (HAL)**: `embedded-hal` / `embedded-hal-async`
- **로깅 및 진단 프레임워크**: `defmt` + `defmt-rtt` (고속 바이너리 직렬화 로깅, 제로 UART 오버헤드)
- **디버깅 및 플래시 도구**: `probe-rs` (`cargo-embed`, `cargo-run`)
- **메모리 보호 및 안전성**: MPU(Memory Protection Unit) 활성화, DMA-코히런시 관리(L1 Cache 클린/무효화)
- **ROS 2 미들웨어**: 순수 Rust Zenoh 1.0 UDP 클라이언트 (`rmw_zenoh_cpp` 호환, 별도 중계 데몬 불필요)

---

## 3. 워크스페이스 구조 (Workspace Architecture)

본 워크스페이스는 Cargo Workspace 기반 멀티 크레이트 아키텍처로 구성되어 있다:

```text
nucleo_h743zi2_rust/
├── .cargo/
│   └── config.toml               # 빌드 타깃(thumbv7em-none-eabihf) 및 러너(probe-rs) 설정
├── Cargo.toml                    # Cargo Workspace 선언 및 공통 의존성 관리
├── crates/                       # 공통 라이브러리 크레이트 (순수 알고리즘, 미들웨어, BSP)
│   ├── nucleo-bsp/               # NUCLEO-H743ZI2 및 X-NUCLEO-IKS01A3 보드 지원 패키지
│   │   ├── Cargo.toml
│   │   └── src/lib.rs            # 온보드 핀 매핑(LED/버튼/이더넷 RMII) 및 6종 센서 레지스터 상수
│   ├── so3-inekf/                # 리 군(Lie Group) SO(3) 다양체 및 우불변 InEKF 수학 코어
│   │   ├── Cargo.toml
│   │   └── src/                  # Rodrigues Exp/Log 사상, Hat/Vee 연산자, 6D InEKF 필터
│   └── zenoh-ros2/               # [공용 미들웨어] no_std Zero-Allocation ROS 2 / Zenoh 클라이언트
│       ├── Cargo.toml
│       └── src/                  # wire(와이어 프레임), cdr(무할당 직렬화), Publisher/Subscriber/ServiceServer
├── examples/                     # 단계별 독립 실행 예제 바이너리 크레이트
│   ├── 01_blinky/                # [Step 0] 온보드 3색 LED 순차 점멸 예제
│   ├── 02_sensor_all_sampling/   # [Step 1] IKS01A3 6종 센서 이종 주기 비동기 샘플링 예제
│   ├── 03_sensor_web_dashboard/  # [Step 2] LAN8742A RMII 이더넷(DHCP) 및 내장 웹 대시보드 예제
│   ├── 04_ahrs_so3_inekf/        # [Step 3] SO(3) Right-Invariant InEKF AHRS 및 3D 웹 대시보드 예제
│   ├── 05_mixed_cpp_legacy/      # [Step 4] Rust + C++ 혼합 크로스 컴파일 및 Biquad LPF 브리지 예제
│   └── 06_ros2_node/             # [Step 5] Zenoh UDP 직통 ROS 2 노드 (100Hz IMU, cmd_vel, set_led 서비스)
├── history.md                    # 일자별 깃 커밋 히스토리 및 개발 기록
├── README.md                     # 본 프로젝트 기술 명세서
├── AGENTS.md                     # 에이전트 거버넌스 규칙
├── .agents -> void_agent_toolkit/.agents # 에이전트 도구 및 스킬 (심볼릭 링크)
└── void_agent_toolkit/           # 에이전트 툴킷 서브모듈
```

---

## 4. 예제 개발 로드맵 및 구성 (Examples)

| 예제 번호 | 패키지명 | 주요 기능 및 기술 스택 | 검증 방식 |
| :--- | :--- | :--- | :--- |
| **01_blinky** | `blinky_01` | GPIO 사용자 LED 순차 점멸, `defmt` RTT 초고속 로깅, Embassy 태스크 스케줄링 | 온보드 RTT 실측 |
| **02_sensor_all_sampling** | `sensor_all_sampling_02` | IKS01A3 6종 센서 허브, 416Hz 오버샘플링/LPF2, 선점형 `InterruptExecutor` 100Hz RT 제어 | 온보드 RTT 실측 (지터 < 1µs) |
| **03_sensor_web_dashboard** | `sensor_web_dashboard_03` | LAN8742A RMII 이더넷 드라이버, DHCPv4 자동 할당, 포트 80 비동기 웹서버 및 실시간 센서 JSON REST API | `curl` 및 브라우저 검증 |
| **04_ahrs_so3_inekf** | `ahrs_so3_inekf_04` | $SO(3)$ 우불변 InEKF 9축 자세 추정, ZARU 정지 감지, DWT CPU 사용률 계측, WebGL 3D 대시보드 | `curl` 및 WebGL 3D 뷰어 |
| **05_mixed_cpp_legacy** | `mixed_cpp_legacy_05` | Rust + C++ 혼합 빌드(`build.rs` + Clang++/G++), Biquad IIR LPF 필터, Safe RAII C-ABI 브리지 | 온보드 RTT 실측 |
| **06_ros2_node** | `ros2_node_06` | 순수 Rust Zenoh UDP 스택, 100Hz IMU 스트리밍, `cmd_vel` 수신, `set_led` 서비스 응답 (Zero-Loss) | Docker 격리 E2E 하네스 |

---

## 5. 개발 환경 준비 및 실행 가이드 (Getting Started)

### ① 하드웨어 연결 확인
1. NUCLEO-H743ZI2 보드의 ST Zio 커넥터에 X-NUCLEO-IKS01A3 쉴드를 적층 장착한다.
2. 유선 이더넷 기반 예제(`03`, `04`, `06`) 구동 시, 온보드 RJ45 포트에 LAN 케이블을 연결한다.
3. USB ST-LINK 커넥터(CN1)를 PC에 연결하고 장치를 확인한다:
   ```bash
   probe-rs list
   ```

### ② 예제 빌드 및 실행
모든 예제는 타깃 툴체인(`thumbv7em-none-eabihf`)을 지정하여 실행한다:

```bash
# 01_blinky: 기본 온보드 LED 순차 점멸 예제
cargo run -p blinky_01

# 02_sensor_all_sampling: 6종 센서 이종 주기(100Hz, 10Hz, 1Hz) 비동기 샘플링 예제
cargo run -p sensor_all_sampling_02

# 03_sensor_web_dashboard: LAN8742A 유선 이더넷(DHCP) 및 내장 웹 대시보드 모니터링 예제
cargo run -p sensor_web_dashboard_03

# 04_ahrs_so3_inekf: SO(3) 우불변 InEKF 자세 추정 및 GPU 가속 3D 웹 대시보드 예제
cargo run -p ahrs_so3_inekf_04

# 05_mixed_cpp_legacy: Rust + C++ 혼합 크로스 컴파일 및 Biquad LPF 필터링 예제
cargo run -p mixed_cpp_legacy_05

# 06_ros2_node: 임베디드 ROS 2 노드 펌웨어 플래시
cargo run -p ros2_node_06
```

### ③ 공용 크레이트 단위 테스트 및 Docker E2E 자동화 검증
- **`zenoh-ros2` 미들웨어 호스트 단위 테스트**:
  ```bash
  cargo test -p zenoh-ros2 --target x86_64-unknown-linux-gnu
  ```
- **`so3-inekf` 알고리즘 오라클 차등 테스트**:
  ```bash
  cargo test -p so3-inekf --target x86_64-unknown-linux-gnu
  ```
- **`06_ros2_node` Docker 격리 E2E 통합 하네스 검증**:
  ```bash
  ./examples/06_ros2_node/test_host/run_test.sh
  ```
  *(호스트 시스템에 ROS 2를 직접 설치할 필요 없이, Docker 컨테이너 내부에서 100Hz IMU 주기/지터, 매칭 구독자 식별, `set_led` 서비스 응답을 완전 무인으로 자동 검증합니다)*