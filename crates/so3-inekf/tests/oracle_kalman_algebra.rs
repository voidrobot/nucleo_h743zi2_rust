//! # Oracle Differential Tests: Kalman Algebra vs nalgebra
//!
//! Numerical equivalence testing of 3x3 matrix inversion, Kalman Gain derivation,
//! and Joseph Form covariance updates.

use nalgebra::{Matrix3, Matrix6, SMatrix};
use rand::{rngs::StdRng, Rng, SeedableRng};
use so3_inekf::inekf::invert_3x3;
use so3_inekf::So3;

/// 1. 3x3 Inversion: invert_3x3 vs nalgebra::Matrix3::try_inverse
#[test]
fn test_oracle_invert_3x3_positive_definite() {
    let mut rng = StdRng::seed_from_u64(1234);

    for _ in 0..100 {
        // Generate random symmetric positive-definite 3x3 matrix: M = A * A^T + 0.5 * I
        let a: Matrix3<f32> = Matrix3::new(
            rng.gen_range(-2.0f32..2.0), rng.gen_range(-2.0f32..2.0), rng.gen_range(-2.0f32..2.0),
            rng.gen_range(-2.0f32..2.0), rng.gen_range(-2.0f32..2.0), rng.gen_range(-2.0f32..2.0),
            rng.gen_range(-2.0f32..2.0), rng.gen_range(-2.0f32..2.0), rng.gen_range(-2.0f32..2.0),
        );
        let m: Matrix3<f32> = a * a.transpose() + 0.5f32 * Matrix3::identity();

        let m_arr = [
            [m[(0, 0)], m[(0, 1)], m[(0, 2)]],
            [m[(1, 0)], m[(1, 1)], m[(1, 2)]],
            [m[(2, 0)], m[(2, 1)], m[(2, 2)]],
        ];

        // Custom invert_3x3
        let inv_custom = invert_3x3(m_arr).expect("Positive definite matrix must be invertible");

        // nalgebra reference
        let inv_oracle = m.try_inverse().expect("nalgebra inverse must succeed");

        for i in 0..3 {
            for j in 0..3 {
                let diff = (inv_custom[i][j] - inv_oracle[(i, j)]).abs();
                assert!(
                    diff < 1e-4,
                    "Inverse mismatch at ({}, {}): custom={}, oracle={}, diff={}",
                    i, j, inv_custom[i][j], inv_oracle[(i, j)], diff
                );
            }
        }
    }
}

/// 2. Singular 3x3 Inversion: ensure None is safely returned without panic or NaN
#[test]
fn test_oracle_invert_3x3_singular_matrices() {
    // Rank 1 matrix: all rows identical
    let rank1 = [
        [1.0, 2.0, 3.0],
        [1.0, 2.0, 3.0],
        [1.0, 2.0, 3.0],
    ];
    assert!(invert_3x3(rank1).is_none(), "Rank 1 matrix must return None");

    // All zeros
    let zeros = [[0.0; 3]; 3];
    assert!(invert_3x3(zeros).is_none(), "Zero matrix must return None");

    // Rank 2: third row is sum of first two
    let rank2 = [
        [1.0, 0.0, 2.0],
        [0.0, 1.0, 3.0],
        [1.0, 1.0, 5.0],
    ];
    assert!(invert_3x3(rank2).is_none(), "Rank 2 matrix must return None");
}

/// 3. Joseph Form Covariance Update: compare hand-written 3-nested loops with nalgebra
#[test]
fn test_oracle_joseph_form_covariance_update() {
    let mut rng = StdRng::seed_from_u64(8888);

    for _ in 0..100 {
        // 1. Generate random 6x6 positive-definite covariance P: P = B * B^T + 0.01 * I
        let mut b: Matrix6<f32> = Matrix6::zeros();
        for i in 0..6 {
            for j in 0..6 {
                b[(i, j)] = rng.gen_range(-1.0f32..1.0);
            }
        }
        let p_na: Matrix6<f32> = b * b.transpose() + 0.01f32 * Matrix6::identity();

        let mut p_arr = [[0.0f32; 6]; 6];
        for i in 0..6 {
            for j in 0..6 {
                p_arr[i][j] = p_na[(i, j)];
            }
        }

        // 2. Set up observation vector and constant Jacobian H = [-v_ref]_x (3x6)
        let v_ref = [0.0f32, 0.0, 9.80665];
        let neg_v_hat = So3::hat([-v_ref[0], -v_ref[1], -v_ref[2]]);
        let mut h_na: SMatrix<f32, 3, 6> = SMatrix::zeros();
        for i in 0..3 {
            for j in 0..3 {
                h_na[(i, j)] = neg_v_hat[i][j];
            }
        }
        let r_noise = 0.2f32;
        let r_mat = r_noise * Matrix3::identity();

        // 3. Innovation covariance S = H * P * H^T + R * I
        let s_na = h_na * p_na * h_na.transpose() + r_mat;
        let s_inv_na = s_na.try_inverse().expect("S must be invertible");

        // 4. Kalman Gain K = P * H^T * S^-1 (6x3)
        let k_na: SMatrix<f32, 6, 3> = p_na * h_na.transpose() * s_inv_na;

        // 5. Reference Joseph Form via nalgebra:
        // P_oracle = (I - K*H) * P * (I - K*H)^T + K * (R*I) * K^T
        let eye6 = Matrix6::identity();
        let a_na = eye6 - k_na * h_na;
        let p_oracle = a_na * p_na * a_na.transpose() + k_na * r_mat * k_na.transpose();

        // 6. Custom manual calculation (as implemented in RightInvariantInEKF::update_vector_observation)
        let mut k_gain = [[0.0f32; 3]; 6];
        for i in 0..6 {
            for j in 0..3 {
                k_gain[i][j] = k_na[(i, j)];
            }
        }

        let mut a = [[0.0f32; 6]; 6];
        for i in 0..6 {
            for j in 0..6 {
                let delta = if i == j { 1.0 } else { 0.0 };
                let mut kh = 0.0;
                for m in 0..3 {
                    kh += k_gain[i][m] * h_na[(m, j)];
                }
                a[i][j] = delta - kh;
            }
        }

        let mut ap = [[0.0f32; 6]; 6];
        for i in 0..6 {
            for j in 0..6 {
                let mut sum = 0.0;
                for m in 0..6 {
                    sum += a[i][m] * p_arr[m][j];
                }
                ap[i][j] = sum;
            }
        }

        let mut p_custom = [[0.0f32; 6]; 6];
        for i in 0..6 {
            for j in 0..6 {
                let mut sum = 0.0;
                for m in 0..6 {
                    sum += ap[i][m] * a[j][m];
                }
                let mut krk = 0.0;
                for m in 0..3 {
                    krk += k_gain[i][m] * k_gain[j][m] * r_noise;
                }
                p_custom[i][j] = sum + krk;
            }
        }

        // 7. Verify custom Joseph form matches nalgebra reference within 1e-4
        for i in 0..6 {
            for j in 0..6 {
                let diff = (p_custom[i][j] - p_oracle[(i, j)]).abs();
                assert!(
                    diff < 1e-4,
                    "Joseph form mismatch at ({}, {}): custom={}, oracle={}, diff={}",
                    i, j, p_custom[i][j], p_oracle[(i, j)], diff
                );
            }
        }
    }
}
