---
title: "NUCLEO-H743ZI2 Mixed Language (Rust + Legacy C++) 예제 (05_mixed_cpp_legacy)"
source: "examples/05_mixed_cpp_legacy"
created: "2026-10-10 11:18:00"
modified: "2026-10-10 11:55:00"
description: "NUCLEO-H743ZI2 환경에서 레거시 C++ DSP 코드를 build.rs 및 clang++-18로 크로스 컴파일하여 Rust Embassy 비동기 펌웨어와 링크/호출하는 Mixed Language 아키텍처 및 빌드 시스템 심층 분석서"
tags:
  - "embedded-rust"
  - "mixed-language"
  - "cpp-ffi"
  - "biquad-filter"
  - "dsp"
  - "embassy"
  - "stm32h743"
  - "clang"
  - "build-rs-deep-dive"
  - "memory-safety-deep-dive"
---

# NUCLEO-H743ZI2 Mixed Language (Rust + Legacy C++) 예제 (05_mixed_cpp_legacy)

## 1. 개요 및 설계 배경 (Overview & Context)

임베디드 소프트웨어 개발 현업에서는 이미 수년간의 필드 검증을 거친 고성능 DSP 필터, 모터 제어 PID 알고리즘, 특수 통신 프로토콜 스택 등 방대한 **레거시 C/C++ 코드베이스**가 존재한다. 이러한 검증된 자산을 Rust로 완전히 재작성(Full Rewrite)하는 것은 막대한 공수와 예기치 못한 수치적/동기적 버그 리스크를 수반한다.

본 예제는 NUCLEO-H743ZI2 (Arm® Cortex®-M7 480 MHz) 개발 보드 및 X-NUCLEO-IKS01A3 센서 쉴드 환경에서, **기존 C++로 작성된 2차 IIR Biquad 저주파 통과 필터(LPF) 클래스를 단 한 줄의 코드 수정 없이 Rust 빌드 파이프라인과 통합 크로스 컴파일**하고, Rust 비동기 펌웨어에서 무오버헤드로 안전하게 실시간 호출하는 표준 모범 아키텍처를 제시한다.

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

### ① Clang 18 기반 크로스 컴파일 파이프라인 (`build.rs`)
- **LLVM 통합 크로스 컴파일**: 호스트 시스템의 `clang++-18`을 직접 호출하여 타깃 아키텍처(`thumbv7em-none-eabihf`, Cortex-M7 Hard-float `fpv5-d16`)로 C++ 소스 코드를 오브젝트 및 아카이브(`liblegacy_dsp.a`)로 컴파일한다.
- **임베디드 C++ 런타임 제로화 (`no_std` 호환)**:
  - `-fno-exceptions`: 예외 처리 테이블(`__gxx_personality_v0`) 및 스택 언와인딩 제거.
  - `-fno-rtti`: 런타임 타입 정보(`typeid`) 및 가상 함수 테이블 오버헤드 제거.
  - `-fno-unwind-tables`: 불필요한 DWARF 언와인딩 프레임 제거.
  - `build.cpp_link_stdlib(None)`: 호스트용 C++ 표준 라이브러리(`-lstdc++`) 링크를 원천 차단하여 순수 자립형(Freestanding) 링크를 보장한다.

### ② 무할당(Zero-Allocation) C++ 클래스 및 C-ABI 브리지 (`cpp/`)
- **Direct Form II Transposed 구조**: 부동소수점 누적 오차 및 오버플로우 저항성이 뛰어난 2차 IIR Biquad LPF를 구현했다.
- **자립형 삼각함수 다항식 근사**: 외부 `libm` 심볼 누락을 방지하기 위해 7차 테일러/미니맥스 고정밀 다항식 근사(`local_sinf`, `local_cosf`)를 C++ 내부에 포함하여 외부 런타임 라이브러리 의존성을 0%로 달성했다.
- **Placement 초기화 C-ABI 브리지**:
  ```cpp
  uint32_t biquad_get_instance_size(void);
  void biquad_init_lpf(void* filter_mem, float sample_rate, float cutoff_freq, float q);
  float biquad_process(void* filter_mem, float input);
  void biquad_reset(void* filter_mem);
  ```
  동적 힙 할당(`malloc`/`new`)을 일절 배제하고, Rust에서 전달받은 스택/정적 버퍼에 C++ 객체를 직접 초기화한다.

### ③ Safe RAII Rust Newtype 래퍼 (`src/cpp_bridge.rs`)
- **스택 인라인 저장소**: C++ `sizeof(BiquadFilter)`가 28바이트(float 7개)임을 반영하여, Rust 측에서 4바이트 정렬된 32바이트 인라인 버퍼 `[u8; 32]`를 소유한다.
- **RAII 메모리 수명주기 보장**:
  ```rust
  #[repr(C, align(4))]
  pub struct SafeBiquadFilter {
      storage: [u8; 32],
  }
  ```
  `SafeBiquadFilter`는 포인터를 외부 비즈니스 로직에 노출하지 않고 안전한 Rust Safe API(`new_lpf`, `process`, `reset`)만을 제공하며, `Drop` 트레이트를 통해 소멸 시 상태를 안전하게 초기화한다.

---

## 4. 심층 분석: 빌드 시스템 패러다임 전환 — Makefile/CMake vs Rust `build.rs` (Deep Dive: Build Systems)

### ① 왜 C/C++ 개발자는 `build.rs`를 보고 당혹스러워하는가?
전통적 C/C++ 생태계에서는 **"빌드 로직(Makefile, CMakeLists.txt)"**과 **"소스 코드(*.c, *.cpp)"**의 물리적/개념적 분리가 철칙이었다. 따라서 C++ 컴파일러 플래그, 인클루드 경로, 컴파일 명령이 Rust 소스 코드 파일인 `build.rs` 내부에서 명령형(Imperative) Rust 문법으로 작성된 모습을 처음 접하면 큰 문화적 충격을 받기 쉽다.

그러나 이는 편법이 아니며, Rust 생태계의 절대적인 **공식 표준(De facto standard)**이자 핵심 설계 철학인 **단일 도구 원칙(Single Toolchain Principle)**에 기반한다.

### ② 단일 도구 원칙 (Single Toolchain Principle)
C/C++ 프로젝트에서는 외부 라이브러리를 결합할 때 `cmake` ➔ `make` ➔ 빌드 산출물 복사 ➔ 메인 빌드로 이어지는 파편화된 다단계 명령을 사용자가 직접 수동 관리해야 했다.
반면 Rust는 **"어떤 외부 언어나 레거시 자산이 포함되어 있더라도, 개발자는 오직 `cargo build` 단 한 줄만 치면 모든 의존성이 완결되어야 한다"**는 철학을 고수한다.

### ③ Makefile/CMake vs Rust `build.rs` 4대 비교 매트릭스

| 비교 관점 | C/C++ 전통 방식 (Makefile / CMake) | Rust 방식 (`build.rs` + `cc` 크레이트) |
| :--- | :--- | :--- |
| **빌드 실행 편의성** | `cmake .. && make && cargo build` (다단계 수동 체인) | **`cargo build` 단 한 줄로 C++ 컴파일부터 최종 링크까지 완결** |
| **호스트 도구 의존성** | `make`, `cmake`, `ninja`, `python` 등 별도 도구 설치 필수 | Rust 패키지 매니저(`cargo`)와 컴파일러만으로 자체 완결 |
| **타깃 파라미터 동기화** | Cargo 타깃과 Makefile 플래그를 **수동 동기화 (불일치/휴먼 에러 빈발)** | Cargo의 `TARGET`, `OPT_LEVEL`, `OUT_DIR` 환경변수를 **Rust 코드가 자동 상속** |
| **크로스 플랫폼 일관성** | Linux/macOS(`make`), Windows(`nmake`/`mingw`) 문법 파편화 | 호스트 OS에 무관하게 동일한 Rust 문법으로 일관된 빌드 제어 |

### ④ 2단계(Two-Stage) 크로스 컴파일 라이프사이클

Cargo는 프로젝트 루트에 `build.rs`가 존재하면 개발 PC(호스트)에서 먼저 `build.rs`를 실행한 후, 생성된 C++ 아카이브를 MCU 타깃 링크 단계에 결합하는 2단계 빌드를 수행한다:

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

### ⑤ Rust 오픈소스 생태계의 대표 사례 및 대규모 확장 전략
- **대표 크레이트 전원 채택**: `ring`(고성능 C/ASM 암호학 라이브러리), `openssl-sys`, `libsqlite3-sys`, `zstd-sys`, 임베디드 벤더 PAC 등 C/C++ 코드를 내장한 거의 모든 주요 Rust 라이브러리가 예외 없이 이 `build.rs` + `cc` 방식을 채택하고 있다.
- **수백 개 파일의 대규모 레거시 C++ 프로젝트 확장**:
  기존에 거대한 `CMakeLists.txt`가 이미 구축되어 있는 프로젝트의 경우, Rust 생태계의 **`cmake` 크레이트**를 활용할 수 있다. `build.rs` 안에서 `cmake::build("legacy_cpp_dir")`를 호출하면, Cargo가 기존 CMake 파이프라인을 그대로 감싸서 구동하고 생성된 산출물을 가져와 최종 펌웨어에 자동 링크한다.

---

## 5. 심층 분석: 임베디드 `no_std` 환경에서의 C++ FFI 메모리 안전성 (Deep Dive: Memory Safety)

### ① C++ 힙 할당(`malloc`/`new`)의 위험과 Zero-Allocation 스택 인라인 배치
- **임베디드 `no_std`의 링커 결함 방어**: 마이크로컨트롤러 환경에는 기본 C/C++ 런타임의 동적 힙 알로케이터(`malloc`, `free`, `operator new`)가 구비되어 있지 않거나 정적 할당 정책을 적용한다. 만약 C++ 코드에서 `new BiquadFilter()`를 호출하면 링커 단계에서 미정의 심볼(Undefined reference to `malloc`) 에러가 발생하거나 런타임 힙 단편화(Fragmentation)가 발생한다.
- **스택 인라인 저장소(Inline Storage) 기법**:
  C++ 클래스의 메모리 레이아웃(float 7개 = 28바이트)을 사전에 파악하여, Rust 측에서 4바이트 정렬된 고정 크기 스택 버퍼(`[u8; 32]`)를 생성하고 포인터를 전달하여 객체를 초기화(Placement 방식)함으로써 **동적 힙 할당을 0%로 배제**하고 결정론적 실시간성(Deterministic Real-Time)을 보장한다.

### ② C++ 예외(`-fno-exceptions`) 및 RTTI 제거의 필수성
- **예외 전파 불가(No Exception Crossing)**: C++의 예외 메커니즘(`throw`/`catch`)은 스택 언와인딩 테이블(`eh_frame`)과 런타임 지원을 필요로 한다. FFI 경계를 넘어 Rust로 C++ 예외가 누출되면 Rust는 이를 처리할 수 없어 즉각적인 미정의 동작(Undefined Behavior) 및 시스템 크래시를 유발한다.
- **플래그 강제**: 따라서 임베디드 C++ 컴파일 시 `-fno-exceptions`, `-fno-rtti`, `-fno-unwind-tables`를 필수 지정하여 예외 테이블 생성 자체를 원천 차단하고 바이너리 크기를 극한으로 경감시킨다.

### ③ `extern "C"` ABI와 심볼 맹글링(Name Mangling) 방어
- C++ 컴파일러는 함수 오버로딩과 네임스페이스를 지원하기 위해 심볼 이름을 복잡하게 변환(Name Mangling, 예: `_ZN12BiquadFilter7processEf`)한다.
- Rust 링커가 C++ 함수를 명확하게 찾을 수 있도록, 모든 브리지 함수는 반드시 `extern "C"` 블록으로 선언하여 순수 C-ABI 심볼(`biquad_process`)로 노출해야 한다.

---

## 6. 엔지니어링 트레이드오프 및 인사이트 (Trade-offs & Insights)

| 구분 | 장점 (Pros) | 한계 및 주의점 (Cons & Constraints) |
| :--- | :--- | :--- |
| **Mixed Language FFI** | - 수만 라인의 검증된 레거시 C++ 코드 재활용 가능<br>- Rust 생태계에 없는 C++ 독점 알고리즘 연동 가능<br>- FFI 함수 호출 오버헤드 0 (직접 CALL/BL 인스트럭션) | - `unsafe` 경계 존재 (포인터 크기/정렬 불일치 주의)<br>- C++ 예외(Exception) 전파 불가 (패닉 방어 필수) |
| **Zero-Allocation 스택 방식** | - 동적 힙(`malloc`/`free`) 누수 및 파편화 원천 차단<br>- 결정론적 실시간성(Deterministic Real-Time) 보장 | - C++ 클래스 크기 변경 시 Rust 저장소 크기 동기화 필요 |
| **Clang 18 크로스 컴파일** | - GCC 툴체인 별도 설치 없이 LLVM 단일 파이프라인 유지<br>- `-O3` 고도 벡터화 및 인라인 최적화 | - 호스트 Clang 버전 의존성 |

---

## 7. 빌드 및 실행 가이드 (Build & Run Guide)

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
