---
title: "NUCLEO-H743ZI2 SO(3) Right-Invariant InEKF 자세 추정 및 3D 웹 대시보드 (04_ahrs_so3_inekf)"
source: "examples/04_ahrs_so3_inekf"
created: "2026-10-09 22:30:00"
modified: "2026-10-09 22:30:00"
description: "리 군 SO(3) 다양체 및 우불변(Right-Invariant) 오차 역학을 적용하여 선형화 오차 없는 상수 야코비 기반 자세 추정을 구현하고, 온보드 이더넷(DHCP)과 내장 3D 웹 대시보드로 실시간 시각화하는 임베디드 AHRS 예제"
tags:
  - "embedded-rust"
  - "lie-group"
  - "so3"
  - "inekf"
  - "ahrs"
  - "iks01a3"
  - "web-dashboard"
  - "3d-visualization"
---

# NUCLEO-H743ZI2 SO(3) Right-Invariant InEKF 자세 추정 및 3D 웹 대시보드 (04_ahrs_so3_inekf)

## 1. 개요 및 설계 배경 (Overview & Context)

### ① 개발 배경 및 목적
본 예제는 NUCLEO-H743ZI2 및 X-NUCLEO-IKS01A3 센서 쉴드 기반으로, 현대 로보틱스와 항공우주 항법 분야의 최전선 이론인 **리 군(Lie Group) $SO(3)$ 다양체 기하학**과 **우불변(Right-Invariant) 확장 칼만 필터(InEKF)**를 결합한 고성능 임베디드 AHRS(Attitude and Heading Reference System) 펌웨어이다.

단순한 오일러 각이나 쿼터니언 기반 필터의 한계(짐벌 락, 이중 커버링, 국소 선형화 왜곡)를 극복하고, 온보드 유선 이더넷(DHCP)을 통해 **브라우저에서 3D 보드 모델의 실시간 물리 회전 및 텔레메트리를 시각화하는 완전 독립형(Zero-CDN) 웹 대시보드**를 제공한다.

### ② 해결 과제
1. **야코비(Jacobian)의 상태 독립 상수화 (Invariance)**:
   - 일반 ESKF는 바디 프레임 오차를 취하여 관측 야코비가 현재 추정 자세 $\hat{R}$에 종속된다.
   - 우불변(Right-Invariant) 오차 $\eta = \hat{R} R^{-1} \in SO(3)$를 정의함으로써, 중력 가속도 및 지자기 관측 야코비를 상태와 완전히 무관한 **절대 상수 행렬($-[\mathbf{g}]_\times$, $-[\mathbf{m}]_\times$)**로 변환하여 전역 수렴성(Global Convergence)을 달성한다.
2. **동적 힙 할당 제로 (Zero-Heap Invariant)**:
   - $SO(3)$ 지수/로그 사상, 6차원 InEKF 상태 전파, $3 \times 3$ 여인수 역행렬, 조셉 형태(Joseph Form) 공분산 갱신을 순수 스택 기반 고정 크기 배열로 완결한다.
3. **독립형 3D 가상 대시보드 (Zero-CDN)**:
   - 외부 Three.js나 인터넷 연결 없이 폐쇄망에서도 동작하도록 브라우저 하드웨어 가속 CSS3 3D Transform 기반 3D NUCLEO 보드 모델을 Flash에 내장한다.

---

## 2. 시스템 구조 및 데이터 흐름 (Architecture & Data Flow)

```mermaid
flowchart TD
    subgraph Hardware ["NUCLEO-H743ZI2 + X-NUCLEO-IKS01A3"]
        LSM["LSM6DSO (6축 IMU)"] -->|I2C1 400kHz DMA| RT_Loop["100Hz RT-IMU 선점 루프 (NVIC CEC P6)"]
        LIS["LIS2MDL (3축 지자기)"] -->|I2C1 400kHz| Mag_Loop["10Hz 지자기 관측 태스크"]
        ETH["LAN8742A RMII PHY"] <-->|DHCPv4 / TCP Port 80| Net_Stack["embassy-net 스택"]
    end

    subgraph MathCore ["crates/so3-inekf (수학 엔진)"]
        RT_Loop -->|각속도 w| Pred["100Hz 관성 적분: R = R * exp(w_unb * dt)"]
        RT_Loop -->|가속도 a| Up_Acc["중력 관측 갱신: H = -[g]x (상수 야코비)"]
        Mag_Loop -->|지자기 m| Up_Mag["지자기 관측 갱신: H = -[m]x (상수 야코비)"]
        Pred --> InEKF["RightInvariantInEKF"]
        Up_Acc --> InEKF
        Up_Mag --> InEKF
    end

    subgraph Output ["텔레메트리 및 시각화"]
        InEKF -->|Roll, Pitch, Yaw, Quat, Bias| Snapshot["AHRS_SNAPSHOT (원자적 동기화)"]
        Snapshot --> Web["내장 HTTP 웹서버"]
        Snapshot --> RTT["1Hz RTT 디버그 콘솔"]
        Web -->|GET /api/ahrs (JSON)| RestAPI["REST API 텔레메트리"]
        Web -->|GET / (HTML/CSS/JS)| Dashboard3D["브라우저 3D 자세 동기화 (GPU 가속)"]
    end
```

---

## 3. 핵심 구현 메커니즘 (Key Implementation Mechanisms)

### ① 리 군 $SO(3)$ Rodrigues 지수 사상 및 특이점 테일러 전개 (`crates/so3-inekf/src/so3.rs`)
회전 벡터 $\boldsymbol{\phi} \in \mathbb{R}^3$로부터 $3 \times 3$ 직교 회전 행렬 $R \in SO(3)$를 유도한다:
$$\exp(\boldsymbol{\phi}^\wedge) = I + a \boldsymbol{\phi}^\wedge + b (\boldsymbol{\phi}^\wedge)^2$$
- $\|\boldsymbol{\phi}\| < 10^{-4}$ 근방에서는 테일러 급수($a \approx 1 - \frac{\theta^2}{6}$, $b \approx \frac{1}{2} - \frac{\theta^2}{24}$)를 적용하여 부동소수점 0 나누기(Division-by-Zero)를 원천 차단한다.

### ② 우불변(Right-Invariant) 관측 갱신 및 상수 야코비 (`crates/so3-inekf/src/inekf.rs`)
- **공간 투영 혁신**: $\mathbf{z} = \hat{R} y_{sensor} - \mathbf{v}_{ref}$
- **상수 야코비**: $H = \begin{bmatrix} -[\mathbf{v}_{ref}]_\times & 0_{3 \times 3} \end{bmatrix}$ (상태 $\hat{R}$과 무관)
- **우불변 매니폴드 상태 복귀 (Manifold Retraction)**:
  $$\hat{R} \leftarrow \exp(-\boldsymbol{\xi}^\wedge) \cdot \hat{R}, \quad \hat{\mathbf{b}}_\omega \leftarrow \hat{\mathbf{b}}_\omega + \delta \mathbf{b}$$
- **조셉 형태(Joseph Form) 공분산 갱신**:
  $$P \leftarrow (I - K H) P (I - K H)^T + K R_{cov} K^T$$
  수치적 비대칭 및 음수 고윳값 전파를 방어하여 장기 가동 안정성을 보장한다.

### ③ 내장 3D 글래스모피즘 웹 대시보드 (`examples/04_ahrs_so3_inekf/src/main.rs`)
- 외부 인터넷망이나 CDN 없이 순수 CSS3 3D Transform(`transform-style: preserve-3d`)을 통해 브라우저 하드웨어 GPU 가속으로 가상 NUCLEO-144 보드를 렌더링한다.
- 100 Hz 루프에서 추정된 Roll, Pitch, Yaw 및 회전 행렬을 `/api/ahrs`를 통해 실시간 폴링하여 보드의 물리적 기울임과 지연 없이 1:1 회전 동기화한다.

---

## 4. 엔지니어링 트레이드오프 및 인사이트 (Trade-offs & Insights)

| 구분 | 장점 (Pros) | 한계 및 주의점 (Cons & Constraints) |
| :--- | :--- | :--- |
| **$SO(3)$ InEKF** | - 재선형화 불필요(상수 야코비로 연산 극대화)<br>- 짐벌 락 및 쿼터니언 이중 커버링 부재<br>- 전역 점근 수렴성 보장 | - $3 \times 3$ 역행렬 연산 필요 (여인수 방식으로 1µs 내 해결)<br>- 급격한 동적 가속도 외란 시 중력 벡터 왜곡 방어 로직 필요 |
| **임베디드 웹 3D 대시보드** | - 외부 설치 프로그램(ROS, Unity 등) 없이 브라우저로 즉시 3D 확인<br>- 폐쇄망 자립 구동 (Flash 내장 19 KB) | - 단일 TCP 소켓 기반이므로 다수 클라이언트 동시 접속 시 순차 서빙 처리 필요 |

---

## 5. 실행 및 검증 가이드 (Verification)

### ① 호스트 유닛 테스트 (수학 불변성 검증)
```bash
cargo test --target x86_64-unknown-linux-gnu -p so3-inekf --lib
```
- $SO(3)$ 지수/로그 왕복 오차, 특이점 테일러 전개, 중력 수렴 및 조셉 공분산 정상 검증.

### ② 타깃 보드 플래시 및 실행
```bash
cargo run -p ahrs_so3_inekf_04
```
- NUCLEO 보드가 실행되면 DHCP 서버로부터 IP(예: `192.168.50.93`)를 할당받는다.
- 웹 브라우저 접속: `http://192.168.50.93/`
- REST API 데이터 확인: `curl -s http://192.168.50.93/api/ahrs`

---

## 6. 관련 문서 및 소스코드 참조 (References)
- [so3.rs](../../crates/so3-inekf/src/so3.rs): Lie Group $SO(3)$ 및 $\mathfrak{so}(3)$ 연산자 구현체
- [inekf.rs](../../crates/so3-inekf/src/inekf.rs): 6D Right-Invariant InEKF 엔진
- [main.rs](src/main.rs): 100 Hz RT-IMU 선점 루프 및 3D 웹서버 펌웨어
