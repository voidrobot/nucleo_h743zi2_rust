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
}
