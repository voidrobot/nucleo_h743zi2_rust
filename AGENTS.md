# 프로젝트 규칙: NUCLEO-H743ZI2 Rust 임베디드 펌웨어 (AGENTS.md)

본 문서는 **NUCLEO-H743ZI2** 개발 보드 및 **X-NUCLEO-IKS01A3** 센서 쉴드 기반의 Rust 임베디드 펌웨어 개발 시 에이전트가 준수해야 할 필수 도메인 지침 및 거버넌스 규칙이다.

---

## 1. 공통 거버넌스 및 에이전트 인프라 런타임 규칙

### ① 공통 거버넌스 상속 (Core Rules)
- 본 워크스페이스는 도메인 중립 공통 툴킷인 [CORE_RULES.md](void_agent_toolkit/CORE_RULES.md)의 거버넌스 규격 및 `.agents/rules/` 하위 8대 세부 운영 규칙을 100% 준용·상속한다.
- 터미널 4대 불변헌장([terminal-rules.md](.agents/rules/terminal-rules.md)), 계획 및 검증([planning-rules.md](.agents/rules/planning-rules.md)), 엔지니어링 원칙([engineering-rules.md](.agents/rules/engineering-rules.md)), 사고 원칙([reasoning-rules.md](.agents/rules/reasoning-rules.md)), Git 규칙([git-rules.md](.agents/rules/git-rules.md)), 커뮤니케이션/경어체([documentation-rules.md](.agents/rules/documentation-rules.md)), 지식 문서([knowledge-rules.md](.agents/rules/knowledge-rules.md))는 Antigravity 시스템 프롬프트에 의해 상시 강제된다.

### ② 워크스페이스 순수성 및 외부 런타임 강제 (`void_agent_env`)
- **순수 Rust 임베디드 도메인 유지**:
  - 본 워크스페이스 내부는 순수 Rust 환경으로만 유지하며, 워크스페이스 내 임의의 Python 가상환경(`.venv`) 생성을 엄격히 금지한다.
- **에이전트 인프라 격리 호출**:
  - 정적 분석, 코드 색인 등 에이전트 스킬 실행 시 [python-rules.md](.agents/rules/python-rules.md)에 따라 반드시 외부 격리 런타임(`$HOME/.local/share/void_agent_env/bin/python`)을 호출한다.

---

## 2. 대상 하드웨어 및 플랫폼 규격

- **메인보드 MCU**: STM32H743ZI (Arm® 32-bit Cortex®-M7 with DP-FPU, 480 MHz, L1 Cache)
- **센서 쉴드**: X-NUCLEO-IKS01A3 (LSM6DSO, LIS2MDL, LIS2DW12, LPS22HH, HTS221, STTS751)
- **온보드 디버거**: ST-LINK/V3E (SWD, VCP, RTT 지원)
- **타깃 툴체인**: `thumbv7em-none-eabihf` (`no_std`)
- **주요 프레임워크**: `embassy` (`embassy-stm32`, `embassy-executor`), `embedded-hal`, `defmt`

---

## 3. 임베디드 펌웨어 빌드, 플래시 및 테스트 안전 수칙

### ① 크로스 컴파일과 호스트 단위 테스트의 물리적 분리
- **임베디드 펌웨어 컴파일**:
  - 모든 펌웨어 바이너리 및 예제(`examples/*`)는 반드시 타깃 아키텍처(`thumbv7em-none-eabihf`)를 지정하여 빌드한다.
    ```bash
    cargo build --target thumbv7em-none-eabihf --example <EXAMPLE_NAME>
    ```
- **호스트 유닛 테스트 (`cargo test`)**:
  - `no_std` 및 하드웨어 레지스터 제어 코드는 호스트(`x86_64`)에서 실행할 수 없다.
  - 알고리즘(Madgwick AHRS, InEKF 계산), 프로토콜 패킷 파서, 순수 데이터 구조에 한해서만 호스트 타깃 라이브러리 단위 테스트(`cargo test --lib`)로 분리하여 실행한다.
  - 하드웨어 의존적인 예제 바이너리 전체를 대상으로 무차별적인 `cargo test`를 실행하는 행위를 엄격히 금지한다.

### ② 하드웨어 플래시 및 Probe-rs 안전 수칙 (Anti-Deadlock & Anti-Brick)
- **사전 하드웨어 감지 확인**:
  - `cargo run` 또는 `probe-rs run` 실행 전, 하드웨어 ST-LINK 연결 여부를 `probe-rs list` 명령으로 신속히 확인한다.
  - 타깃 보드가 연결되지 않은 상태에서 플래시 명령을 실행하여 프로세스가 무한 대기(Hang)에 빠지는 것을 방지한다.
- **RTT 로깅 및 세션 타임아웃 관리**:
  - `probe-rs run` 실행 시 RTT 로그 출력을 수신하는 동안 프로세스가 영구 블로킹되지 않도록, `terminal-rules.md`의 하트비트 자율 감시 프로토콜 및 타임아웃을 엄격히 적용한다.
- **칩 브릭(Brick) 방지 불변 규칙**:
  - STM32H7의 비휘발성 옵션 바이트(Option Bytes), 플래시 RDP(Readout Protection) 레벨, 보안 퓨즈(Security Fuses)를 수정하는 명령이나 코드를 절대 작성/실행하지 않는다.

### ③ STM32H7 메모리 도메인 및 DMA 코히런시 규칙
- **L1 데이터 캐시(D-Cache)와 DMA 충돌 방어**:
  - STM32H743은 Cortex-M7의 고속 L1 D-Cache를 탑재하고 있어, DMA(I2C/SPI DMA) 버퍼와 캐시 간 데이터 불일치(Cache Incoherency)가 발생할 수 있다.
  - DMA 전송에 사용되는 버퍼는 MPU(Memory Protection Unit)를 통해 Non-cacheable 영역(AXI SRAM 또는 SRAM4 등)에 배치하거나, 전송 전후로 캐시 클린(Clean) 및 무효화(Invalidate)를 명시적으로 수행하는 구조를 강제한다.
- **링커 스크립트(`Memory.x`) 무결성 유지**:
  - 플래시(2MB) 및 SRAM 분할 구조(ITCM, DTCM, AXI SRAM, SRAM1~4)의 주소 매핑을 임의로 훼손하지 않는다.
