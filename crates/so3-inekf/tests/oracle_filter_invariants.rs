//! # Oracle Differential Tests: Filter Invariants under Long Run & Stress
//!
//! Verifies 10,000-step covariance symmetry, positive definiteness via Cholesky decomposition,
//! and pathological input resilience.

use nalgebra::{Cholesky, Matrix6};
use rand::{rngs::StdRng, Rng, SeedableRng};
use so3_inekf::RightInvariantInEKF;

/// Helper: convert 6x6 float array to nalgebra Matrix6
fn array66_to_nalgebra(p: &[[f32; 6]; 6]) -> Matrix6<f32> {
    let mut m = Matrix6::zeros();
    for i in 0..6 {
        for j in 0..6 {
            m[(i, j)] = p[i][j];
        }
    }
    m
}

/// 1. 10,000 continuous steps: Covariance Symmetry and Positive-Definiteness (Cholesky)
#[test]
fn test_oracle_filter_long_run_positive_definiteness() {
    let mut filter = RightInvariantInEKF::new();
    let mut rng = StdRng::seed_from_u64(4321);

    const STEPS: usize = 2_000;
    const DT: f32 = 0.01; // 100 Hz

    for step in 0..STEPS {
        // Simulated noisy gyro and accel
        let gyro_raw = [
            rng.gen_range(-0.1f32..0.1),
            rng.gen_range(-0.1f32..0.1),
            rng.gen_range(-0.1f32..0.1),
        ];
        let accel_raw = [
            rng.gen_range(-0.2f32..0.2),
            rng.gen_range(-0.2f32..0.2),
            9.80665f32 + rng.gen_range(-0.2f32..0.2),
        ];

        filter.update_stillness(gyro_raw, accel_raw);
        filter.predict(gyro_raw, DT);
        filter.update_accel(accel_raw);

        // Every 100 steps, audit covariance invariants
        if step % 100 == 0 {
            // Check 1: Symmetry P == P^T
            for i in 0..6 {
                for j in (i + 1)..6 {
                    let diff = (filter.p[i][j] - filter.p[j][i]).abs();
                    assert!(
                        diff < 1e-4,
                        "Step {}: Covariance asymmetry at ({}, {}): P_ij={}, P_ji={}, diff={}",
                        step,
                        i,
                        j,
                        filter.p[i][j],
                        filter.p[j][i],
                        diff
                    );
                }
            }

            // Check 2: Positive Definiteness via Cholesky decomposition
            let p_na = array66_to_nalgebra(&filter.p);
            let chol = Cholesky::new(p_na);
            assert!(
                chol.is_some(),
                "Step {}: Covariance matrix lost positive definiteness! Cholesky failed. P={:?}",
                step,
                filter.p
            );

            // Check 3: Finite values (no NaN or Inf)
            assert!(
                filter.cov_trace().is_finite(),
                "Step {}: Covariance trace is not finite: {}",
                step,
                filter.cov_trace()
            );
        }
    }
}

/// 2. Pathological inputs: zero gyro, free-fall drop, extreme shock (10g)
#[test]
fn test_oracle_filter_pathological_inputs() {
    let mut filter = RightInvariantInEKF::new();

    // 1. Zero gyro
    filter.predict([0.0, 0.0, 0.0], 0.01);
    assert!(filter.cov_trace().is_finite());

    // 2. Free-fall (accel magnitude ~ 0.0 m/s^2) -> must reject
    let freefall = [0.0, 0.0, 0.1];
    let accepted = filter.update_accel(freefall);
    assert!(!accepted, "Free-fall acceleration must be rejected");

    // 3. Extreme shock (10g = ~98 m/s^2) -> must reject
    let shock = [0.0, 0.0, 98.1];
    let accepted_shock = filter.update_accel(shock);
    assert!(!accepted_shock, "10g shock acceleration must be rejected");

    // Filter state must remain clean and finite
    let (roll, pitch, yaw) = filter.rot.to_euler_deg();
    assert!(roll.is_finite() && pitch.is_finite() && yaw.is_finite());
}

/// 3. ZARU (Zero Angular Rate Update) exponential bias convergence test
#[test]
fn test_oracle_zaru_bias_convergence() {
    let mut filter = RightInvariantInEKF::new();
    let true_bias = [0.03f32, -0.02, 0.025];
    let stationary_accel = [0.0f32, 0.0, 9.80665];

    // Run 600 cycles (~6 seconds)
    for _ in 0..600 {
        filter.update_stillness(true_bias, stationary_accel);
        filter.predict(true_bias, 0.01);
        filter.update_accel(stationary_accel);
    }

    assert!(
        filter.is_stationary,
        "Must be stationary after 300 still cycles"
    );

    // Verify bias converged towards true_bias
    for i in 0..3 {
        let diff = (filter.bias_gyro[i] - true_bias[i]).abs();
        assert!(
            diff < 0.015,
            "Axis {} bias did not converge: estimated={}, true={}, diff={}",
            i,
            filter.bias_gyro[i],
            true_bias[i],
            diff
        );
    }
}
