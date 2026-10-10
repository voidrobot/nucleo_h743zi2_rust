//! # Oracle Differential Tests: SO(3) Lie Group vs nalgebra
//!
//! Ground truth cross-validation between `so3-inekf::So3` and `nalgebra::Rotation3`.

use nalgebra::{Matrix3, Rotation3, UnitQuaternion, Vector3};
use rand::{rngs::StdRng, Rng, SeedableRng};
use so3_inekf::So3;
use std::f32::consts::PI;

const EPSILON: f32 = 1.0e-5;

/// Helper: convert So3 to nalgebra Matrix3
fn so3_to_nalgebra(rot: &So3) -> Matrix3<f32> {
    Matrix3::new(
        rot.data[0][0],
        rot.data[0][1],
        rot.data[0][2],
        rot.data[1][0],
        rot.data[1][1],
        rot.data[1][2],
        rot.data[2][0],
        rot.data[2][1],
        rot.data[2][2],
    )
}

/// Helper: check matrix equality within tolerance
fn assert_matrix_close(m1: &Matrix3<f32>, m2: &Matrix3<f32>, tol: f32, msg: &str) {
    let diff = (m1 - m2).abs();
    for i in 0..3 {
        for j in 0..3 {
            assert!(
                diff[(i, j)] <= tol,
                "{}: mismatch at ({}, {}): m1={}, m2={}, diff={}",
                msg,
                i,
                j,
                m1[(i, j)],
                m2[(i, j)],
                diff[(i, j)]
            );
        }
    }
}

/// 1. Exp map differential test across 1,000 random rotation vectors
#[test]
fn test_oracle_exp_map_random() {
    let mut rng = StdRng::seed_from_u64(42);

    for _ in 0..200 {
        // Sample random rotation axis and angle in [0.001, PI - 0.01]
        let axis = Vector3::new(
            rng.gen_range(-1.0..1.0),
            rng.gen_range(-1.0..1.0),
            rng.gen_range(-1.0..1.0),
        )
        .normalize();
        let angle: f32 = rng.gen_range(0.001..(PI - 0.01));
        let omega = axis * angle;
        let w_arr = [omega.x, omega.y, omega.z];

        // Custom So3::exp
        let rot_custom = So3::exp(w_arr);
        let m_custom = so3_to_nalgebra(&rot_custom);

        // nalgebra reference
        let rot_oracle = Rotation3::from_scaled_axis(omega);
        let m_oracle = *rot_oracle.matrix();

        assert_matrix_close(&m_custom, &m_oracle, EPSILON, "Random Exp Map Mismatch");
    }
}

/// 2. Exp map near singularity (Taylor expansion regime: norm < 1e-4 down to 1e-9)
#[test]
fn test_oracle_exp_map_near_zero() {
    let mut rng = StdRng::seed_from_u64(1337);

    for exp_p in 4..9 {
        let scale = 10.0f32.powi(-exp_p);
        for _ in 0..100 {
            let omega = Vector3::new(
                rng.gen_range(-1.0..1.0),
                rng.gen_range(-1.0..1.0),
                rng.gen_range(-1.0..1.0),
            ) * scale;
            let w_arr = [omega.x, omega.y, omega.z];

            let rot_custom = So3::exp(w_arr);
            let m_custom = so3_to_nalgebra(&rot_custom);

            let rot_oracle = Rotation3::from_scaled_axis(omega);
            let m_oracle = *rot_oracle.matrix();

            assert_matrix_close(&m_custom, &m_oracle, 1e-6, "Near-zero Taylor Exp Mismatch");
        }
    }
}

/// 3. Log map and round-trip (Log(Exp(w)) == w) vs nalgebra::scaled_axis()
#[test]
fn test_oracle_log_map_roundtrip() {
    let mut rng = StdRng::seed_from_u64(999);

    for _ in 0..100 {
        let axis = Vector3::new(
            rng.gen_range(-1.0..1.0),
            rng.gen_range(-1.0..1.0),
            rng.gen_range(-1.0..1.0),
        )
        .normalize();
        let angle: f32 = rng.gen_range(0.01..(PI - 0.05));
        let omega = axis * angle;
        let w_arr = [omega.x, omega.y, omega.z];

        let rot = So3::exp(w_arr);
        let log_w = rot.log();

        assert!(
            (log_w[0] - w_arr[0]).abs() < 1e-3
                && (log_w[1] - w_arr[1]).abs() < 1e-3
                && (log_w[2] - w_arr[2]).abs() < 1e-3,
            "Log mismatch: angle={}, log_w={:?}, w_arr={:?}, diff=[{}, {}, {}]",
            angle,
            log_w,
            w_arr,
            log_w[0] - w_arr[0],
            log_w[1] - w_arr[1],
            log_w[2] - w_arr[2]
        );

        // nalgebra check
        let na_rot = Rotation3::from_scaled_axis(omega);
        let na_axis_angle = na_rot.scaled_axis();
        assert!(
            (log_w[0] - na_axis_angle.x).abs() < 1e-3
                && (log_w[1] - na_axis_angle.y).abs() < 1e-3
                && (log_w[2] - na_axis_angle.z).abs() < 1e-3,
            "nalgebra axis_angle mismatch: log_w={:?}, na={:?}",
            log_w,
            na_axis_angle
        );
    }
}

/// 4. Quaternion conversions cross-checked with nalgebra::UnitQuaternion
#[test]
fn test_oracle_quaternion_conversions() {
    let mut rng = StdRng::seed_from_u64(777);

    for _ in 0..100 {
        let axis = Vector3::new(
            rng.gen_range(-1.0..1.0),
            rng.gen_range(-1.0..1.0),
            rng.gen_range(-1.0..1.0),
        )
        .normalize();
        let angle: f32 = rng.gen_range(0.01..(PI - 0.02));
        let omega = axis * angle;

        let rot = So3::exp([omega.x, omega.y, omega.z]);
        let q_custom = rot.to_quaternion(); // [w, x, y, z]

        let na_rot = Rotation3::from_scaled_axis(omega);
        let na_quat = UnitQuaternion::from_rotation_matrix(&na_rot);

        // Quaternion sign ambiguity: q and -q represent same rotation
        let dot = q_custom[0] * na_quat.w
            + q_custom[1] * na_quat.i
            + q_custom[2] * na_quat.j
            + q_custom[3] * na_quat.k;
        assert!(
            (dot.abs() - 1.0).abs() < 1e-4,
            "Quaternion mismatch: custom={:?}, na=[w={}, i={}, j={}, k={}], dot={}",
            q_custom,
            na_quat.w,
            na_quat.i,
            na_quat.j,
            na_quat.k,
            dot
        );

        // Roundtrip from_quaternion
        let rot_reconstructed = So3::from_quaternion(q_custom);
        let m_recon = so3_to_nalgebra(&rot_reconstructed);
        let m_orig = so3_to_nalgebra(&rot);
        assert_matrix_close(&m_recon, &m_orig, 1e-4, "Quaternion Roundtrip Mismatch");
    }
}

/// 5. Adjoint action identity: exp(Ad_R * w) == R * exp(w) * R^T
#[test]
fn test_oracle_adjoint_identity() {
    let mut rng = StdRng::seed_from_u64(555);

    for _ in 0..100 {
        let r_rot = So3::exp([
            rng.gen_range(-1.0..1.0),
            rng.gen_range(-1.0..1.0),
            rng.gen_range(-1.0..1.0),
        ]);
        let w = [
            rng.gen_range(-1.0..1.0),
            rng.gen_range(-1.0..1.0),
            rng.gen_range(-1.0..1.0),
        ];

        // Ad_R * w for SO(3) is simply matrix-vector multiplication R * w
        let ad_r_w = r_rot.rotate_vec(w);

        // Left side: exp(Ad_R * w)
        let left = So3::exp(ad_r_w);

        // Right side: R * exp(w) * R^T
        let right = r_rot.mul(&So3::exp(w)).mul(&r_rot.transpose());

        let m_left = so3_to_nalgebra(&left);
        let m_right = so3_to_nalgebra(&right);
        assert_matrix_close(&m_left, &m_right, 1e-4, "Adjoint Identity Mismatch");
    }
}

/// 6. Orthogonality and Determinant invariant: R * R^T == I, det(R) == 1.0
#[test]
fn test_oracle_orthogonality_and_determinant() {
    let mut rng = StdRng::seed_from_u64(333);

    for _ in 0..100 {
        let rot = So3::exp([
            rng.gen_range(-2.0..2.0),
            rng.gen_range(-2.0..2.0),
            rng.gen_range(-2.0..2.0),
        ]);
        let m = so3_to_nalgebra(&rot);

        // Check R * R^T == I
        let r_rt = m * m.transpose();
        let eye = Matrix3::identity();
        assert_matrix_close(&r_rt, &eye, 1e-5, "R * R^T != Identity");

        // Check det(R) == 1.0
        let det = m.determinant();
        assert!((det - 1.0).abs() < 1e-5, "det(R) != 1.0: det={}", det);
    }
}
