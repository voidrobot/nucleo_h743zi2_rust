//! # X-NUCLEO-IKS01A3 센서 메트롤로지 및 잡음 사양 (Noise Metrology)
//!
//! IEEE Std 952/1293 및 STMicroelectronics 센서 공식 데이터시트에 명시된
//! 각도 랜덤 워크(ARW), 속도 랜덤 워크(VRW), RMS 잡음 사양을 상수로 정의한다.

use core::f32::consts::PI;
use super::sensitivity::constants::STANDARD_GRAVITY;

/// LSM6DSO 6축 IMU 메트롤로지 잡음 사양
pub mod lsm6dso {
    use super::*;

    /// 각속도계 각도 랜덤 워크 (Angle Random Walk, ARW / Rate Noise Density): 3.8 mdps/√Hz
    pub const GYRO_ARW_MDPS_RT_HZ: f32 = 3.8;

    /// 각속도계 각도 랜덤 워크 SI 단위 환산: rad/s/√Hz (≈ 6.63225e-5)
    pub const GYRO_ARW_RAD_PER_SEC_RT_HZ: f32 =
        GYRO_ARW_MDPS_RT_HZ * (PI / 180.0) / 1000.0;

    /// 각속도계 연속시간 백색 잡음 전력 스펙트럼 밀도 (PSD, (rad/s)^2/Hz)
    pub const GYRO_WHITE_NOISE_PSD: f32 =
        GYRO_ARW_RAD_PER_SEC_RT_HZ * GYRO_ARW_RAD_PER_SEC_RT_HZ;

    /// 가속도계 속도 랜덤 워크 (Velocity Random Walk, VRW / Accel Noise Density): 60 µg/√Hz
    pub const ACCEL_VRW_UG_RT_HZ: f32 = 60.0;

    /// 가속도계 속도 랜덤 워크 SI 단위 환산: m/s^2/√Hz (≈ 5.88399e-4)
    pub const ACCEL_VRW_MPS2_RT_HZ: f32 =
        ACCEL_VRW_UG_RT_HZ * 1e-6 * STANDARD_GRAVITY;

    /// 가속도계 연속시간 백색 잡음 전력 스펙트럼 밀도 (PSD, (m/s^2)^2/Hz)
    pub const ACCEL_WHITE_NOISE_PSD: f32 =
        ACCEL_VRW_MPS2_RT_HZ * ACCEL_VRW_MPS2_RT_HZ;

    /// 100 Hz 샘플링 환경 및 기동 외란 마진을 반영한 권장 연속시간 프로세스 노이즈 분산 Q_gyro ((rad/s)^2/Hz)
    pub const RECOMMENDED_Q_GYRO: f32 = 1e-3;

    /// 100 Hz 샘플링 환경 바이어스 레이트 랜덤 워크 (RRW) 드리프트 분산 Q_bias ((rad/s^2)^2/Hz)
    pub const RECOMMENDED_Q_BIAS: f32 = 1e-5;

    /// 100 Hz 동적 기동 상태 가속도계 관측 노이즈 분산 R_accel ((m/s^2)^2)
    pub const RECOMMENDED_R_ACCEL_DYNAMIC: f32 = 0.2;

    /// 100 Hz 정지 상태 (ZARU) 가속도계 관측 노이즈 분산 R_accel ((m/s^2)^2, 무외란 신속 수렴용)
    pub const RECOMMENDED_R_ACCEL_STATIONARY: f32 = 0.04;
}

/// LIS2MDL 3축 지자기 센서 메트롤로지 잡음 사양
pub mod lis2mdl {
    /// 지자기 자속 밀도 유효 노이즈 (Magnetic Flux Density RMS Noise): 3.0 mgauss
    pub const MAG_RMS_NOISE_MGAUSS: f32 = 3.0;

    /// 지자기 자속 밀도 유효 노이즈 (Gauss 단위): 0.003 gauss
    pub const MAG_RMS_NOISE_GAUSS: f32 = MAG_RMS_NOISE_MGAUSS * 1e-3;

    /// 지자기 자속 밀도 유효 노이즈 (Tesla 단위): 0.3 µT = 3.0e-7 T
    pub const MAG_RMS_NOISE_TESLA: f32 = MAG_RMS_NOISE_GAUSS * 1e-4;

    /// 100 Hz 필터 환경 권장 정규화 지자기 관측 노이즈 분산 R_mag
    pub const RECOMMENDED_R_MAG: f32 = 0.1;
}
