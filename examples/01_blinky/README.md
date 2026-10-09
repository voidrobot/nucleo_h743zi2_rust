---
title: "NUCLEO-H743ZI2 Embassy 온보드 LED 순차 점멸 예제 (01_blinky)"
source: "examples/01_blinky"
created: "2026-10-09 19:00:20"
modified: "2026-10-09 19:05:15"
description: "NUCLEO-H743ZI2 온보드 3색 LED 순차 점멸 예제 및 RTT/defmt의 C/C++ 대비 아키텍처적 차별점 심층 분석서"
tags:
  - "embedded-rust"
  - "embassy"
  - "stm32h743"
  - "blinky"
  - "defmt-rtt"
  - "rtt-deep-dive"
  - "nucleo-bsp"
---

# NUCLEO-H743ZI2 Embassy 온보드 LED 순차 점멸 예제 (01_blinky)

## 1. 개요 및 설계 배경 (Overview & Context)
- **목적**: 본 예제는 NUCLEO-H743ZI2 개발 보드의 기본 하드웨어 동작 상태를 검증하는 Step 0 브링업(Bring-up) 펌웨어이다.
- **해결 과제**:
  - STM32H743ZI MCU의 기본 전원 공급, 클럭 트리 및 GPIO 출력 드라이버의 정상 작동 확인.
  - 별도의 UART 시리얼 배선 없이 ST-LINK/V3E 온보드 디버거를 통한 `defmt` + RTT 초고속 무간섭 로깅 파이프라인 검증.
  - 전용 하드웨어 추상화 계층인 `nucleo-bsp`의 `BoardLeds` API 정상 연동 확인.

---

## 2. 시스템 구조 및 데이터 흐름 (Architecture & Data Flow)

### ① 하드웨어 핀 매핑 (Hardware Pinout)
NUCLEO-H743ZI2 보드에 실장된 3개의 사용자 LED를 제어한다.

| 심볼 | LED 명칭 | 실장 색상 | MCU GPIO 핀 | 로직 레벨 | 초기 상태 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `leds.green` | LED1 | 녹색 (Green) | `PB0` | High = ON / Low = OFF | Low (OFF) |
| `leds.yellow` | LED2 | 노란색 (Yellow) | `PE1` | High = ON / Low = OFF | Low (OFF) |
| `leds.red` | LED3 | 적색 (Red) | `PB14` | High = ON / Low = OFF | Low (OFF) |

### ② 비동기 제어 루프 파이프라인 (Execution Flow)

```mermaid
sequenceDiagram
    autonumber
    participant App as 비동기 태스크 (main)
    participant Timer as Embassy Time Driver
    participant GPIO as STM32H7 GPIO 레지스터
    participant RTT as SRAM RTT 링버퍼

    Note over App: 1. 초기화 완료 (LED1, LED2, LED3)
    loop 순차 점멸 사이클 (1.4초 주기)
        App->>RTT: defmt::info!("Blink Cycle #{}: ...", counter)
        App->>GPIO: PB0 (Green) High
        App->>Timer: Timer::after_millis(300).await (WFE 슬립)
        Timer-->>App: 타이머 만료 인터럽트 복귀
        App->>GPIO: PB0 (Green) Low

        App->>GPIO: PE1 (Yellow) High
        App->>Timer: Timer::after_millis(300).await (WFE 슬립)
        Timer-->>App: 타이머 만료 인터럽트 복귀
        App->>GPIO: PE1 (Yellow) Low

        App->>GPIO: PB14 (Red) High
        App->>Timer: Timer::after_millis(300).await (WFE 슬립)
        Timer-->>App: 타이머 만료 인터럽트 복귀
        App->>GPIO: PB14 (Red) Low

        App->>Timer: Timer::after_millis(500).await (전체 소등 휴지기)
        Timer-->>App: 복귀
    end
```

---

## 3. 핵심 구현 메커니즘 (Key Implementation Mechanisms)

### ① `nucleo-bsp` 기반의 하드웨어 추상화 (`nucleo_bsp::BoardLeds`)
- 개별 예제에서 매번 원시 GPIO 핀 번호(`p.PB0`, `p.PE1`, `p.PB14`)를 하드코딩하지 않고, `BoardLeds::new(...)` 팩토리 메서드를 호출하여 구조체 단위로 제어권을 획득한다.
- 잘못된 핀 할당이나 중복 점유를 컴파일 타임에 Rust의 소유권(Ownership) 규칙으로 원천 방지한다.

### ② 논블로킹 비동기 타이머 (`Timer::after_millis(...).await`)
- `cortex_m::asm::delay()`와 같은 비지 웨이트(Busy-wait) 루프는 CPU 코어를 100% 점유하여 전력을 낭비하고 다른 작업을 방해한다.
- 본 예제는 `embassy-time` 드라이버를 사용하여 대기 시간 동안 Cortex-M7 코어를 저전력 슬립 모드(`WFI`/`WFE`)로 전환하고, 하드웨어 타이머 인터럽트로 복귀하는 협력적 멀티태스킹 방식을 채택한다.

### ③ Zero-UART 고속 RTT 로깅 (`defmt::info!`)
- 텍스트 포맷팅을 타깃 MCU에서 수행하지 않고, 정수 식별자와 바이트 인자만 SRAM의 RTT 버퍼에 고속 기록한다.
- 호스트의 `probe-rs`가 SWD 버스를 통해 이를 읽어와 역복원하므로 CPU 연산 지연이 거의 발생하지 않는다.

---

## 4. 심층 분석: RTT 메커니즘과 Rust `defmt`의 차별성 (Deep Dive: RTT & defmt)

### ① RTT (Real-Time Transfer)의 본질과 C/C++ 생태계
- **기원**: RTT는 SEGGER가 자사 J-Link 디버거를 위해 고안한 통신 규격으로, 언어에 종속되지 않는 하드웨어 디버그 기술이다.
- **C/C++에서의 사용**: C/C++ 임베디드 프로젝트에서도 `SEGGER_RTT.c` 소스를 포함하여 `SEGGER_RTT_printf(0, "val: %d\r\n", val)` 형태로 UART 대체재로 널리 사용해 왔다.
- **물리 계층 동작 원리**: 칩 내부 SRAM에 `_SEGGER_RTT` 제어 블록(링 버퍼)을 할당해 두고, ST-LINK나 J-Link 디버거가 SWD(Serial Wire Debug) 버스의 AHB-AP(Access Port)를 통해 CPU를 멈추지 않고(Non-intrusive) 메모리를 직접 읽어 호스트 PC로 스트리밍한다.

### ② C/C++ 일반 RTT vs Rust `defmt` 비교 분석

RTT 하드웨어 채널은 동일하지만, **"문자열 서식화(Formatting)를 어디에서 수행하는가"**에서 결정적 아키텍처 차이가 발생한다.

| 비교 항목 | C/C++ 전통적 RTT (`SEGGER_RTT_printf`) | Rust 생태계 (`defmt::info!`) |
| :--- | :--- | :--- |
| **통신 물리 채널** | SRAM RTT 링 버퍼 (SWD 읽기) | SRAM RTT 링 버퍼 (SWD 읽기) |
| **포맷 서식 문자열 저장소** | **MCU 플래시 메모리 (ROM)** | **호스트 PC ELF 심볼 테이블** (MCU 점유 0B) |
| **문자열 변환 연산 주체** | **MCU Cortex-M7 코어** | **호스트 PC CPU (`probe-rs`)** |
| **버퍼에 기록되는 페이로드** | 완성된 ASCII 문자열 (수십 바이트) | **정수 토큰 ID + 원시 바이트** (2~4바이트) |
| **CPU 실행 지연 시간** | 수 마이크로초 ~ 수십 마이크로초 | **수십 나노초 (수 CPU 사이클)** |
| **타이밍 왜곡 (Heisenbug)** | 포맷팅 부하로 실시간 루프 교란 가능 | 부하가 극소화되어 타이밍 왜곡 실질적 배제 |

### ③ C/C++ 환경과의 비교 및 Rust의 엔지니어링 혁신
- **C/C++에서의 포맷팅 지연 시도**: C/C++에서도 Google의 `Pigweed (pw_tokenizer)`나 일부 차량용 트레이서가 유사한 토큰화 방식을 지원하지만, 복잡한 매크로 트릭, 별도의 후처리 파이썬 스크립트, 독립 데몬 프로세스를 빌드 시스템(CMake/GN)에 수동 결합해야 하는 진입장벽이 존재한다.
- **Rust의 네이티브 통합**: Rust는 컴파일러 단계의 프로시저 매크로(Proc Macro)와 링커 섹션 제어 능력을 바탕으로, 개발자가 추가 툴체인 설정 없이 `Cargo.toml` 의존성과 `probe-rs` 러너만으로 이 고도화된 포맷팅 지연 파이프라인을 원클릭(`cargo run`)으로 사용할 수 있도록 완성도를 끌어올렸다.

---

## 5. 엔지니어링 트레이드오프 및 인사이트 (Trade-offs & Insights)

### ① 장점 (Pros)
- **보드 브링업의 결정론적 검증**: LED 3색이 정확히 녹색 → 노란색 → 빨간색 순으로 회전하는 시각적 피드백을 통해 칩의 정상 클럭 공급 여부를 즉시 판별할 수 있다.
- **센서 예제 확장의 기반 확보**: 본 비동기 이벤트 루프 구조를 그대로 유지하면서, 향후 I2C 센서 폴링 태스크나 백그라운드 DMA 전송 태스크를 Spawner를 통해 병렬로 추가할 수 있다.

### ② 한계 및 주의점 (Cons & Constraints)
- **디버거 의존성**: `defmt` RTT 로그를 수신하기 위해서는 반드시 ST-LINK/V3E 디버거와 호스트의 `probe-rs` 세션이 활성화되어 있어야 한다. (단독 배터리 구동 시에는 LED 동작만 육안 확인 가능).
- **소프트웨어 지터**: 단순 LED 점멸에서는 무시할 만하나, 정밀한 마이크로초(µs) 단위 타이밍 제어가 필요한 경우 하드웨어 PWM 타이머 주변장치를 활용해야 한다.

---

## 6. 실행 및 검증 방법 (Run & Verification)

### ① 실행 명령어
NUCLEO-H743ZI2 보드가 USB로 연결된 상태에서 워크스페이스 최상위 루트에서 아래 명령을 실행한다:

```bash
cargo run -p blinky_01
```

### ② 기대 RTT 터미널 출력
```text
      INFO  ==========================================
      INFO  NUCLEO-H743ZI2 Embassy Blinky (Workspace)
      INFO  ==========================================
      INFO  BSP BoardLeds 초기화 완료 (LED1: PB0, LED2: PE1, LED3: PB14)
      INFO  Blink Cycle #1: 순차 점멸 시작
      INFO  Blink Cycle #2: 순차 점멸 시작
      INFO  Blink Cycle #3: 순차 점멸 시작
```

---

## 7. 관련 문서 및 소스코드 참조 (References)
- [예제 메인 소스코드](src/main.rs): `01_blinky` 비동기 점멸 구현체
- [예제 패키지 설정](Cargo.toml): `blinky_01` 크레이트 의존성 정의
- [BSP 라이브러리](../../crates/nucleo-bsp/src/lib.rs): 온보드 핀아웃 및 `BoardLeds` 구조체
- [전체 프로젝트 README](../../README.md): 프로젝트 로드맵 및 개발 환경 설정
