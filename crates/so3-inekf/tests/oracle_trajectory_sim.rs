//! # Oracle Differential Tests: 3D Trajectory Simulation Benchmark
//!
//! Generates ground truth 3D angular trajectories using nalgebra, simulates noisy IMU & Mag sensors,
//! and verifies that `RightInvariantInEKF` converges to ground truth with low RMSE.

use nalgebra::{Rotation3, Vector3};
use rand::{rngs::StdRng, Rng, SeedableRng};
use so3_inekf::{RightInvariantInEKF, So3};

#[test]
fn test_oracle_trajectory_simulation_convergence() {
    let mut filter = RightInvariantInEKF::new();
    let mut rng = StdRng::seed_from_u64(2026);

    const DT: f32 = 0.01; // 100 Hz
    const TOTAL_STEPS: usize = 1000; // 10 seconds
    let g_world = Vector3::new(0.0f32, 0.0, 9.80665);
    let m_world = Vector3::new(0.35f32, 0.0, 0.45).normalize();

    let true_bias = Vector3::new(0.015f32, -0.01, 0.02);

    // Ground truth orientation integrated via nalgebra
    let mut r_true = Rotation3::identity();

    let mut attitude_errors_rad: Vec<f32> = Vec::with_capacity(TOTAL_STEPS);

    for step in 0..TOTAL_STEPS {
        let t = step as f32 * DT;

        // Ground truth angular velocity (multi-axis sinusoidal motion)
        let w_true = Vector3::new(
            0.15 * (0.8 * t).sin(),
            0.12 * (0.5 * t).cos(),
            0.08 * (0.3 * t).sin(),
        );

        // Advance ground truth
        let delta_r_true = Rotation3::from_scaled_axis(w_true * DT);
        r_true = r_true * delta_r_true;

        // Synthesize noisy IMU measurements
        // Body frame sensor readings: R_true rotates from Body to World, so y_body = R_true^T * v_world
        let gyro_noise = Vector3::new(
            rng.gen_range(-0.005f32..0.005),
            rng.gen_range(-0.005f32..0.005),
            rng.gen_range(-0.005f32..0.005),
        );
        let gyro_meas = w_true + true_bias + gyro_noise;

        let accel_noise = Vector3::new(
            rng.gen_range(-0.05f32..0.05),
            rng.gen_range(-0.05f32..0.05),
            rng.gen_range(-0.05f32..0.05),
        );
        let accel_meas = r_true.transpose() * g_world + accel_noise;

        let mag_noise = Vector3::new(
            rng.gen_range(-0.02f32..0.02),
            rng.gen_range(-0.02f32..0.02),
            rng.gen_range(-0.02f32..0.02),
        );
        let mag_meas = (r_true.transpose() * m_world + mag_noise).normalize();

        // Feed to RightInvariantInEKF
        let gyro_arr = [gyro_meas.x, gyro_meas.y, gyro_meas.z];
        let accel_arr = [accel_meas.x, accel_meas.y, accel_meas.z];
        let mag_arr = [mag_meas.x, mag_meas.y, mag_meas.z];

        filter.predict(gyro_arr, DT);
        filter.update_accel(accel_arr);
        if step % 10 == 0 {
            filter.update_mag(mag_arr);
        }

        // Compute rotation error: Delta R = R_hat * R_true^T
        let r_hat_data = filter.rot.data;
        let r_hat_na = nalgebra::Matrix3::new(
            r_hat_data[0][0], r_hat_data[0][1], r_hat_data[0][2],
            r_hat_data[1][0], r_hat_data[1][1], r_hat_data[1][2],
            r_hat_data[2][0], r_hat_data[2][1], r_hat_data[2][2],
        );
        let r_err_mat = r_hat_na * r_true.matrix().transpose();
        let r_err_so3 = So3::from_matrix([
            [r_err_mat[(0, 0)], r_err_mat[(0, 1)], r_err_mat[(0, 2)]],
            [r_err_mat[(1, 0)], r_err_mat[(1, 1)], r_err_mat[(1, 2)]],
            [r_err_mat[(2, 0)], r_err_mat[(2, 1)], r_err_mat[(2, 2)]],
        ]);
        let err_vec = r_err_so3.log();
        let err_angle = (err_vec[0] * err_vec[0] + err_vec[1] * err_vec[1] + err_vec[2] * err_vec[2]).sqrt();

        // Record error after initial convergence period (t > 2.0s / step > 200)
        if step > 200 {
            attitude_errors_rad.push(err_angle);
        }
    }

    // Compute Root Mean Square Error (RMSE)
    let sum_sq: f32 = attitude_errors_rad.iter().map(|e| e * e).sum();
    let rmse_rad = (sum_sq / attitude_errors_rad.len() as f32).sqrt();
    let rmse_deg = rmse_rad * 180.0 / std::f32::consts::PI;

    println!("Simulation 10s Complete: Attitude RMSE = {:.3} deg ({:.4} rad)", rmse_deg, rmse_rad);

    // Verify RMSE is well within acceptable AHRS filter bounds (< 3.0 degrees)
    assert!(
        rmse_deg < 3.0,
        "Attitude RMSE too high: {:.2} deg (expected < 3.0 deg)", rmse_deg
    );
}
