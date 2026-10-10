---
title: "NUCLEO-H743ZI2 Mixed Language (Rust + Legacy C++) 예제 (05_mixed_cpp_legacy)"
source: "examples/05_mixed_cpp_legacy"
created: "2026-10-10 11:18:00"
modified: "2026-10-10 11:18:00"
description: "NUCLEO-H743ZI2 환경에서 레거시 C++ DSP 코드를 cc 크레이트 및 clang++-18로 크로스 컴파일하여 Rust Embassy 비동기 펌웨어와 링크/호출하는 Mixed Language 아키텍처 가이드"
tags:
  - "embedded-rust"
  - "mixed-language"
  - "cpp-ffi"
  - "biquad-filter"
  - "dsp"
  - "embassy"
  - "stm32h743"
  - "clang"
---

# NUCLEO-H743ZI2 Mixed Language (Rust + Legacy C++) 예제 (05_mixed_cpp_legacy)

## 1. 개요 및 설계 배경 (Overview & Context)

### ① 레거시 C++ 자산 재활용의 필요성
임베디드 소프트웨어 개발 현업에서는 이미 수년간의 검증을 거친 고성능 DSP 필터, 모터 제어 PID 알고리즘, 특수 통신 프로토콜 스택 등 방대한 **레거시 C/C++ 자산**이 존재한다. 이러한 자산을 Rust로 완전히 재작성(Rewrite)하는 것은 막대한 비용과 검증 리스크를 수반한다.

본 예제는 NUCLEO-H743ZI2 (Arm® Cortex®-M7 480 MHz) 개발 보드 및 X-NUCLEO-IKS01A3 센서 쉴드 환경에서, **기존 C++로 작성된 2차 IIR Biquad 저주파 통과 필터(LPF) 클래스를 수정 없이 Rust 빌드 파이프라인과 통합 크로스 컴파일**하고, Rust 비동기 펌웨어에서 무오버헤드로 안전하게 실시간 호출하는 표준 아키텍처를 제시한다.

### ② 왜 Makefile/CMake가 아닌 Rust `build.rs`인가? (단일 도구 원칙)
C/C++ 개발자에게 익숙한 `Makefile`이나 `CMakeLists.txt` 대신 Rust 소스 파일인 `build.rs` 안에서 C++ 컴파일러 플래그와 소스를 정의하는 구조는 낯설게 느껴질 수 있다. 그러나 이는 Rust 생태계의 **공식 표준(De facto standard)**이자 핵심 철학인 **단일 도구 원칙(Single Toolchain Principle)**에 기반한다:

1. **원클릭 빌드 보장**: 사용자는 `cmake .. && make && cargo build`와 같이 외부 빌드 도구를 사전에 설치·실행할 필요 없이, 오직 **`cargo build` 단 한 줄**만으로 C++ 컴파일부터 최종 MCU 바이너리 링크까지 원스톱 완결된다.
2. **크로스 컴파일 환경변수 완벽 상속**: Cargo가 관리하는 타깃 아키텍처(`TARGET=thumbv7em-none-eabihf`), 최적화 레벨(`OPT_LEVEL`), 출력 디렉터리(`OUT_DIR`)가 `build.rs`에 환경변수로 직접 주입되므로, Makefile과 Cargo 간 설정 불일치(Drift) 및 휴먼 에러가 원천 차단된다.
3. **Rust 오픈소스 표준 관행**: `ring`(암호학 라이브러리), `openssl-sys`, `libsqlite3-sys`, `zstd-sys`, 임베디드 벤더 BSP 등 C/C++ 코드를 포함하는 거의 모든 주요 Rust 크레이트가 이 `build.rs` + `cc` 방식을 채택하고 있다.

| 구분 | C/C++ 전통 방식 (Makefile / CMake) | Rust 방식 (`build.rs` + `cc` 크레이트) |
| :--- | :--- | :--- |
| **빌드 실행** | `cmake` ➔ `make` ➔ `cargo` (다단계 수동 실행) | **`cargo build` 단 한 줄로 C++까지 완전 자동화** |
| **호스트 도구 의존성** | `make`, `cmake`, `ninja` 등 별도 도구 설치 필수 | Rust 패키지 매니저(`cargo`) 자체 완결 구동 |
| **타깃 파라미터 동기화** | Makefile과 Cargo 간 아키텍처/FPU 플래그 수동 동기화 | Cargo 빌드 파라미터 자동 상속 (불일치 0%) |
| **운영체제 호환성** | Linux(`make`), Windows(`nmake`/`mingw`) 파편화 | 호스트 OS에 무관하게 동일한 Rust 스크립트 동작 |

### ③ 2단계(Two-Stage) 빌드 라이프사이클

Cargo는 프로젝트 루트에 `build.rs`가 존재하면 개발 PC(호스트)에서 먼저 `build.rs`를 실행한 후, 생성된 산출물을 MCU 타깃 링크 단계에 결합하는 2단계 빌드를 수행한다:

```mermaid
sequenceDiagram
    autonumber
    actor Dev as "개발자 (cargo build)"
    participant Cargo as "Cargo 빌드 엔진"
    participant BuildRs as "build.rs (PC 호스트 실행)"
    participant Clang as "Clang 18 크로스 컴파일러"
    participant Rustc as "Rust 링커 (rust-lld)"

    Dev->>Cargo: "cargo build --target thumbv7em-none-eabihf"
    Note over Cargo,BuildRs: [1단계: 사전 빌드 스크립트 실행 (호스트 PC)]
    Cargo->>BuildRs: "PC용으로 build.rs 컴파일 후 즉시 실행"
    BuildRs->>Clang: "biquad_filter.cpp 크로스 컴파일 (-mcpu=cortex-m7)"
    Clang-->>BuildRs: "liblegacy_dsp.a (정적 라이브러리) 생성"
    BuildRs->>Cargo: "cargo:rustc-link-search (라이브러리 위치 통보)"
    
    Note over Cargo,Rustc: [2단계: 실제 MCU 펌웨어 링크 (STM32H7)]
    Cargo->>Rustc: "main.rs + liblegacy_dsp.a 함께 최종 링크"
    Rustc-->>Dev: "STM32H7 펌웨어 바이너리(ELF) 빌드 완료!"
```

*(참고: 수백 개 소스 파일로 구성되고 기존 `CMakeLists.txt`가 이미 존재하는 대규모 레거시 C++ 프로젝트의 경우, `build.rs` 안에서 `cmake` 크레이트를 호출하여 기존 CMake 파이프라인을 그대로 감싸서 구동할 수도 있다.)*

---

## 2. 시스템 구조 및 데이터 흐름 (Architecture & Data Flow)

```mermaid
flowchart TD
    subgraph BuildTime["빌드 타임 파이프라인 (Cargo + build.rs)"]
        A["cargo build --target thumbv7em-none-eabihf"] --> B["build.rs 실행"]
        B --> C["cc::Build 크로스 컴파일러 호출"]
        C -->|"clang++-18 / -mcpu=cortex-m7 / -fno-exceptions / -fno-rtti"| D["cpp/src/biquad_filter.cpp"]
        D --> E["liblegacy_dsp.a (정적 아카이브)"]
        E -->|"rust-lld 정적 링크 (cpp_link_stdlib: None)"| F["mixed_cpp_legacy_05 ELF 바이너리"]
    end

    subgraph Runtime["런타임 실행 흐름 (100 Hz RT 센서 루프)"]
        G["IKS01A3 LSM6DSO 6축 IMU"] -->|"I2C1 DMA 400kHz"| H["Embassy 비동기 루프 (Rust main.rs)"]
        H -->|"원시 가속도 데이터 (mg)"| I["SafeBiquadFilter (Rust RAII 래퍼)"]
        I -->|"extern C FFI 무오버헤드 호출"| J["C++ BiquadFilter::process (Direct Form II Transposed)"]
        J -->|"평활화된 가속도 출력 (mg)"| K["defmt / RTT 실시간 로깅 및 LED 하트비트"]
    end
```

---

## 3. 핵심 구현 메커니즘 (Key Implementation Mechanisms)

### ① `cc` 크레이트와 Clang 18 기반 크로스 컴파일 파이프라인 (`build.rs`)
- **Clang 18 크로스 컴파일러 연동**: 호스트 시스템의 `clang++-18`을 직접 호출하여 타깃 아키텍처(`thumbv7em-none-eabihf`, Cortex-M7 Hard-float `fpv5-d16`)로 C++ 소스 코드를 오브젝트 및 아카이브(`liblegacy_dsp.a`)로 컴파일한다.
- **임베디드 C++ 런타임 제거 (`no_std` 호환)**:
  - `-fno-exceptions`: 예외 처리 테이블(`__gxx_personality_v0`) 제거.
  - `-fno-rtti`: 런타임 타입 정보(`typeid`) 및 가상 함수 테이블 오버헤드 제거.
  - `-fno-unwind-tables`: 스택 언와인딩 테이블 제거.
  - `build.cpp_link_stdlib(None)`: 호스트용 C++ 표준 라이브러리(`-lstdc++`) 링크를 원천 차단하여 순수 자립형(Freestanding) 임베디드 링크를 보장한다.

### ② 무할당(Zero-Allocation) C++ 클래스 및 C-ABI 브리지 (`cpp/`)
- **C++ 클래스 `BiquadFilter`**: Direct Form II Transposed 구조를 적용하여 부동소수점 오버플로우 저항성과 수치적 안정성을 극대화한다.
- **자립형 삼각함수 다항식 근사**: 외부 `libm` 누락 심볼을 방지하기 위해 7차 테일러/미니맥스 고정밀 다항식 근사(`local_sinf`, `local_cosf`)를 C++ 내부에 포함하여 외부 런타임 의존성을 0%로 달성했다.
- **Placement 초기화 C-ABI 브리지**:
  ```cpp
  uint32_t biquad_get_instance_size(void);
  void biquad_init_lpf(void* filter_mem, float sample_rate, float cutoff_freq, float q);
  float biquad_process(void* filter_mem, float input);
  void biquad_reset(void* filter_mem);
  ```
  동적 힙 할당(`new`/`malloc`)을 일절 배제하고, Rust에서 전달받은 스택/정적 버퍼에 C++ 객체를 직접 초기화한다.

### ③ Rust Safe RAII Newtype 래퍼 (`src/cpp_bridge.rs`)
- **스택 인라인 저장소**: C++ `sizeof(BiquadFilter)`가 28바이트(float 7개)임을 반영하여, Rust 측에서 4바이트 정렬된 32바이트 인라인 버퍼 `[u8; 32]`를 소유한다.
- **RAII 메모리 수명주기 보장**:
  ```rust
  #[repr(C, align(4))]
  pub struct SafeBiquadFilter {
      storage: [u8; 32],
  }
  ```
  `SafeBiquadFilter`는 포인터를 직접 다루지 않고 안전한 Rust Safe API(`new_lpf`, `process`, `reset`)를 제공하며, `Drop` 트레이트를 통해 소멸 시 상태를 안전하게 초기화한다.

---

## 4. 엔지니어링 트레이드오프 및 인사이트 (Trade-offs & Insights)

| 구분 | 장점 (Pros) | 한계 및 주의점 (Cons & Constraints) |
| :--- | :--- | :--- |
| **Mixed Language FFI** | - 수만 라인의 검증된 레거시 C++ 코드 재활용 가능<br>- Rust 생태계에 없는 C++ 독점 라이브러리 연동 가능<br>- FFI 함수 호출 오버헤드 0 (직접 CALL/BL 인스트럭션) | - `unsafe` 경계 존재 (포인터 크기/정렬 불일치 주의)<br>- C++ 예외(Exception) 전파 불가 (패닉 방어 필수) |
| **Zero-Allocation 스택 방식** | - 동적 힙(`malloc`/`free`) 누수 및 파편화 원천 차단<br>- 결정론적 실시간성(Deterministic Real-Time) 보장 | - C++ 클래스 크기 변경 시 Rust 저장소 크기 동기화 필요 |
| **Clang 18 크로스 컴파일** | - GCC 툴체인 별도 설치 없이 LLVM 단일 파이프라인 유지<br>- `-O3` 고도 벡터화 및 인라인 최적화 | - 호스트 Clang 버전 의존성 |

---

## 5. 빌드 및 실행 가이드 (Build & Run Guide)

### ① 타깃 릴리스 빌드 (Target Release Build)
STM32H743ZI Cortex-M7 타깃 아키텍처(`thumbv7em-none-eabihf`)를 지정하여 펌웨어를 컴파일한다:

```bash
cargo build --target thumbv7em-none-eabihf -p mixed_cpp_legacy_05 --release
```

### ② 메모리 풋프린트 점검 (Memory Footprint)
```bash
cargo size --target thumbv7em-none-eabihf -p mixed_cpp_legacy_05 --release -- -A
```
- **Flash (`.text` + `.rodata`)**: 약 **28.0 KB** (C++ 클래스 및 FFI 포함 초경량)
- **RAM (`.data` + `.bss`)**: 약 **33.4 KB**

### ③ ST-LINK/V3E 플래시 및 RTT 실시간 모니터링
`.cargo/config.toml`에 사전 설정된 Runner를 통해 빌드, 다운로드, RTT 수신을 원스톱으로 실행한다:

```bash
# Debug 바이너리 빌드 및 플래시 실행
cargo run -p mixed_cpp_legacy_05

# Release 최적화 바이너리 빌드 및 플래시 실행 (권장)
cargo run -p mixed_cpp_legacy_05 --release
```

**실측 출력 예시 (100 Hz 센서 루프)**:
```text
0.000021 [INFO ] I2C1 버스 400kHz 초기화 완료 (PB8/PB9)
0.000255 [INFO ] LSM6DSO WHO_AM_I: 0x6C (기대값: 0x6C)
0.000531 [INFO ] LSM6DSO 416Hz ODR 하드웨어 가속도계 가동 완료
0.000582 [INFO ] C++ BiquadFilter 3축(X, Y, Z) 인스턴스 초기화 완료 (Fs=100Hz, Fc=5Hz)
1.000958 [INFO ] [100Hz #100] RAW: [-158.112, -26.535, 983.32] mg | C++ LPF: [-157.77437, -28.64144, 984.4204] mg
2.000958 [INFO ] [100Hz #200] RAW: [-157.746, -27.206, 984.052] mg | C++ LPF: [-157.90004, -28.286167, 984.0669] mg
3.000958 [INFO ] [100Hz #300] RAW: [-156.709, -27.267, 984.357] mg | C++ LPF: [-158.18208, -28.432854, 983.98456] mg
```
원시 가속도계 데이터의 고주파 지터가 C++ 2차 저주파 통과 필터(LPF)에 의해 실시간으로 평활화(Filtering)됨을 확인할 수 있다.
