#![no_std]

pub mod so3;
pub mod inekf;

pub use so3::So3;
pub use inekf::RightInvariantInEKF;

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::PI;

    #[test]
    fn test_so3_identity_and_mul() {
        let eye = So3::identity();
        assert_eq!(eye.data[0][0], 1.0);
        assert_eq!(eye.data[1][1], 1.0);
        assert_eq!(eye.data[2][2], 1.0);

        let v = [1.0, 2.0, 3.0];
        let v_rot = eye.rotate_vec(v);
        assert!((v_rot[0] - v[0]).abs() < 1e-6);
        assert!((v_rot[1] - v[1]).abs() < 1e-6);
        assert!((v_rot[2] - v[2]).abs() < 1e-6);
    }

    #[test]
    fn test_so3_exp_log_roundtrip() {
        // 45도 Z축 회전
        let phi = [0.0, 0.0, PI / 4.0];
        let r = So3::exp(phi);
        let phi_rec = r.log();

        assert!((phi_rec[0] - phi[0]).abs() < 1e-4);
        assert!((phi_rec[1] - phi[1]).abs() < 1e-4);
        assert!((phi_rec[2] - phi[2]).abs() < 1e-4);

        let (roll, pitch, yaw) = r.to_euler_deg();
        assert!(roll.abs() < 1e-3);
        assert!(pitch.abs() < 1e-3);
        assert!((yaw - 45.0).abs() < 1e-3);
    }

    #[test]
    fn test_so3_exp_near_singularity() {
        // 1e-5 수준의 아주 작은 회전 (특이점 테일러 전개 검증)
        let phi = [1e-5, -2e-5, 3e-5];
        let r = So3::exp(phi);
        let phi_rec = r.log();

        assert!((phi_rec[0] - phi[0]).abs() < 1e-7);
        assert!((phi_rec[1] - phi[1]).abs() < 1e-7);
        assert!((phi_rec[2] - phi[2]).abs() < 1e-7);
    }

    #[test]
    fn test_inekf_gravity_convergence() {
        let mut filter = RightInvariantInEKF::new();

        // 초기 상태: 수평 (Identity)
        let (r0, p0, _y0) = filter.rot.to_euler_deg();
        assert!(r0.abs() < 1e-3);
        assert!(p0.abs() < 1e-3);

        // 보드가 20도 Roll 기울어진 상태를 시뮬레이션
        // Roll 20도일 때 가속도계가 측정하는 중력 벡터 (Body frame)
        let angle_rad = 20.0 * PI / 180.0;
        let roll_rot = So3::exp([angle_rad, 0.0, 0.0]);
        // y_acc = R_body_to_world^T * [0, 0, 9.80665]
        let true_acc = roll_rot.transpose().rotate_vec([0.0, 0.0, 9.80665]);

        let initial_trace = filter.cov_trace();

        // 10회 관측 갱신 수행
        for _ in 0..10 {
            filter.predict([0.0, 0.0, 0.0], 0.01);
            let updated = filter.update_accel(true_acc);
            assert!(updated);
        }

        let (roll_est, pitch_est, _) = filter.rot.to_euler_deg();
        // 추정치가 20도로 점진 수렴하는지 검증
        assert!(roll_est > 10.0 && roll_est <= 21.0);
        assert!(pitch_est.abs() < 1.0);

        // 공분산 대각합이 수렴하며 감소했는지 검증
        let final_trace = filter.cov_trace();
        assert!(final_trace < initial_trace);
    }

    #[test]
    fn test_inekf_stillness_detection_and_zaru() {
        let mut filter = RightInvariantInEKF::new();
        let stationary_accel = [0.0, 0.0, 9.80665];
        // 작은 하드웨어 자이로 오프셋 모사 (0.02 rad/s = 약 1.1 dps)
        let raw_gyro_bias = [0.02, -0.01, 0.03];

        // 35회 연속 정지 입력 공급 (30회 이상 지속 시 is_stationary 활성화)
        for _ in 0..35 {
            filter.update_stillness(raw_gyro_bias, stationary_accel);
            filter.predict(raw_gyro_bias, 0.01);
            filter.update_accel(stationary_accel);
        }

        assert!(filter.is_stationary, "35회 연속 정지 입력 후 is_stationary는 true여야 함");
        // 정지 상태 확정 시점의 Yaw 각도 기록
        let (_r, _p, yaw_freeze) = filter.rot.to_euler_deg();

        // 정지 상태를 유지하며 추가 50회(0.5초) 동안 지속적으로 바이어스가 있는 자이로 원시 입력 공급
        for _ in 0..50 {
            filter.update_stillness(raw_gyro_bias, stationary_accel);
            filter.predict(raw_gyro_bias, 0.01);
            filter.update_accel(stationary_accel);
        }

        // 자이로 바이어스가 ZARU를 통해 raw_gyro_bias 방향으로 수렴했는지 확인 (총 55회 * 0.002 수렴)
        assert!(filter.bias_gyro[0] > 0.001);
        assert!(filter.bias_gyro[1] < -0.0005);
        assert!(filter.bias_gyro[2] > 0.001);

        // ZARU로 인해 정지 중에는 각속도 적분이 완전 동결되어 Yaw 드리프트가 0이어야 함
        let (_r, _p, yaw_after) = filter.rot.to_euler_deg();
        assert!(
            (yaw_after - yaw_freeze).abs() < 1e-4,
            "정지 상태(ZARU) 중에는 추가 자이로 바이어스 입력에도 불구하고 Yaw 드리프트가 0이어야 함: diff={}",
            yaw_after - yaw_freeze
        );
    }

    #[test]
    fn test_inekf_decoupled_mag_convergence() {
        let mut filter = RightInvariantInEKF::new();
        // 보드가 수평에서 30도 Yaw 회전된 상태를 시뮬레이션
        // 바디 프레임 센서가 계측하는 지자기 벡터: y_mag = R_body_to_world^T * [1.0, 0.0, 0.5]
        let angle_rad = 30.0 * PI / 180.0;
        let true_rot = So3::exp([0.0, 0.0, angle_rad]);
        let mag_measured = true_rot.transpose().rotate_vec([1.0, 0.0, 0.5]);

        for _ in 0..20 {
            filter.predict([0.0, 0.0, 0.0], 0.01);
            filter.update_mag(mag_measured);
        }

        let (roll, pitch, yaw) = filter.rot.to_euler_deg();
        // Roll과 Pitch는 지자기 갱신에 의해 절대 왜곡되지 않아야 함 (0도 유지)
        assert!(roll.abs() < 0.01, "지자기 센서 갱신 후 Roll 왜곡 방지 검증: roll={}", roll);
        assert!(pitch.abs() < 0.01, "지자기 센서 갱신 후 Pitch 왜곡 방지 검증: pitch={}", pitch);
        // Yaw는 지자기 측정 방향인 30도로 수렴해야 함
        assert!(yaw > 15.0 && yaw <= 31.0, "지자기 센서 갱신 후 Yaw 수렴 검증: yaw={}", yaw);
    }
}
