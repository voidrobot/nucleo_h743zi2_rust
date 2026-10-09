# 프로젝트 개발 히스토리 (History)

본 문서는 NUCLEO-H743ZI2 및 X-NUCLEO-IKS01A3 기반 Rust 임베디드 펌웨어 프로젝트의 일자별 주요 마일스톤 및 커밋 내역을 기록한다.

---

## 2026.10.10

- **so3-inekf 오라클 차등 테스트(Differential Testing) 전수 검증**: `[dev-dependencies]`에 `nalgebra` 격리 도입, $SO(3)$ 리 군(Exp/Log/Adjoint), 여인수 역행렬, InEKF 조셉 공분산 갱신, 2,000스텝 촐레스키 양정치성($\lambda_i > 0$), 3D 궤적 시뮬레이션(RMSE $1.85^\circ$) 19개 테스트 100% 통과 (타깃 MCU 105 KB 오염 0%).
- **Embassy Zero-Fork 하드웨어 DWT 유휴 역산(Idle Inversion) CPU 프로파일링 구현**:
  - `#[embassy_executor::main]` 제거 및 `raw::Executor::new(core::ptr::null_mut())` 기반 커스텀 메인 루프 도입 (Embassy 라이브러리 포크/수정 0%).
  - 메인 루프 `cortex_m::asm::wfe()` 전후 DWT 사이클 카운터 계측으로 슬립 사이클 구간 $C_{\text{wfe\_span}}$ 누적.
  - 고우선순위 선점 인터럽트(`InterruptExecutor`) 도메인 InEKF 순수 연산 사이클 $C_{\text{rt}}$ 분리 차감 및 1초 단위 실제 DWT 총 사이클 기반 동적 자가 보정(Self-Calibrating) 알고리즘 적용 (매직 넘버 0개, 클럭 주파수 오차 0%).
  - 물리 보드 실측 검증: 대기 시 시스템 순수 부하 7.57% (InEKF 5.99% + 백그라운드 1.58%), 12.5 Hz 웹 폴링 시 10.76% 정밀 동적 반영 확인.


## 2026.10.09

- **프로젝트 초기화 & 멀티 크레이트 아키텍처 수립**: NUCLEO-H743ZI2 및 X-NUCLEO-IKS01A3 기반 임베디드 Rust 툴체인(`thumbv7em-none-eabihf`), `nucleo-bsp` 및 거버넌스 규칙(`AGENTS.md`) 구축.
- **01_blinky**: 온보드 3색 LED 순차 점멸, `defmt` + RTT 무간섭 초고속 로깅 파이프라인 및 Embassy 스케줄링 검증.
- **02_sensor_all_sampling**: IKS01A3 6종 센서 허브, LSM6DSO 416Hz 오버샘플링/LPF2 안티-에일리어싱, 선점형 `InterruptExecutor` 기반 100Hz RT 제어(지터 0 µs) 달성.
- **03_sensor_web_dashboard**: LAN8742A RMII 이더넷 드라이버 및 DHCPv4 자동 연동, 포트 80 비동기 웹서버 및 실시간 텔레메트리 REST API 서빙.
- **so3-inekf 수학 코어 개발**: 리 군 $SO(3)$ Rodrigues 지수 사상(Exp/Log) 및 6차원 우불변 InEKF 상수 야코비/Joseph 공분산 엔진 구현 (호스트 단위 테스트 통과).
- **04_ahrs_so3_inekf**: 100Hz NVIC 선점 RT-IMU 루프와 지자기 관측 보정 결합 고정밀 AHRS 구현.
- **정지 감지(ZARU) & 1D 지자기 디커플링**: 가속도 바이어스 추정 배제, 정지 시 각속도 적분 동결(Yaw 드리프트 제거), 수평각(Roll/Pitch) 지자기 왜곡 간섭 100% 차단.
- **3D 자세 시각화 엔진 개편**: Body Frame RGB 3축(빨강/초록/파랑) 슬림 직육면체 3D 모델 및 $\pm 180^\circ$ 연속 각도 언래핑 적용.
- **비침습적 CPU 부하 계측**: `.await` I/O 대기 배제, InEKF 순수 연산 지연(~600 µs) 및 실질 CPU 사용률(~7%) 정밀 산출, DWT 카운터 활성화 및 대시보드 네온 배지 연동.
- **예제 빌드 가이드 표준화**: `examples/*` 전체 README에 크로스 컴파일, 릴리스 최적화, 메모리 풋프린트(`cargo size`) 가이드 체계화.

