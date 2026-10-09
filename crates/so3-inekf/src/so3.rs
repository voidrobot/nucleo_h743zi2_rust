//! # SO(3) Lie Group and so(3) Lie Algebra Module
//!
//! Provides mathematically rigorous 3D rotation group operations for embedded systems:
//! - Rodrigues' Exponential Map with Taylor expansion near singularity (norm < 1e-4)
//! - Matrix Logarithm Map
//! - Hat ([\cdot]_\times) and Vee (\cdot^\vee) Lie algebra operators
//! - Conversions to/from Unit Quaternions and Euler Angles (Roll, Pitch, Yaw)

use libm::{acosf, asinf, atan2f, cosf, sinf, sqrtf};

/// 3x3 회전 행렬 기반 SO(3) 리 군(Lie Group) 구조체 (Row-Major)
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct So3 {
    pub data: [[f32; 3]; 3],
}

impl Default for So3 {
    fn default() -> Self {
        Self::identity()
    }
}

impl So3 {
    /// 단위 회전 행렬 생성 (Identity Matrix)
    pub const fn identity() -> Self {
        Self {
            data: [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
        }
    }

    /// 원시 3x3 행렬로부터 생성
    pub const fn from_matrix(m: [[f32; 3]; 3]) -> Self {
        Self { data: m }
    }

    /// 전치 행렬 (SO(3)에서는 전치 행렬이 곧 역행렬: R^T = R^-1)
    pub fn transpose(&self) -> Self {
        Self {
            data: [
                [self.data[0][0], self.data[1][0], self.data[2][0]],
                [self.data[0][1], self.data[1][1], self.data[2][1]],
                [self.data[0][2], self.data[1][2], self.data[2][2]],
            ],
        }
    }

    /// 두 SO(3) 원소 간의 행렬 곱셈: R_out = self * other
    pub fn mul(&self, other: &Self) -> Self {
        let mut out = [[0.0f32; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                out[i][j] = self.data[i][0] * other.data[0][j]
                    + self.data[i][1] * other.data[1][j]
                    + self.data[i][2] * other.data[2][j];
            }
        }
        Self { data: out }
    }

    /// 3차원 벡터 회전: v_out = R * v
    pub fn rotate_vec(&self, v: [f32; 3]) -> [f32; 3] {
        [
            self.data[0][0] * v[0] + self.data[0][1] * v[1] + self.data[0][2] * v[2],
            self.data[1][0] * v[0] + self.data[1][1] * v[1] + self.data[1][2] * v[2],
            self.data[2][0] * v[0] + self.data[2][1] * v[1] + self.data[2][2] * v[2],
        ]
    }

    /// 리 대수 so(3) Hat 연산자: w -> [w]_\times
    pub fn hat(w: [f32; 3]) -> [[f32; 3]; 3] {
        [
            [0.0, -w[2], w[1]],
            [w[2], 0.0, -w[0]],
            [-w[1], w[0], 0.0],
        ]
    }

    /// 리 대수 so(3) Vee 연산자: [w]_\times -> w
    pub fn vee(m: [[f32; 3]; 3]) -> [f32; 3] {
        [
            0.5 * (m[2][1] - m[1][2]),
            0.5 * (m[0][2] - m[2][0]),
            0.5 * (m[1][0] - m[0][1]),
        ]
    }

    /// Rodrigues 지수 사상 (Exponential Map): so(3) -> SO(3)
    /// phi 벡터를 회전 벡터(회전축 * 각도)로 받아 3x3 회전 행렬로 변환
    /// theta < 1e-4 근방에서는 테일러 급수 전개를 적용하여 0 나누기를 원천 차단
    pub fn exp(phi: [f32; 3]) -> Self {
        let theta_sq = phi[0] * phi[0] + phi[1] * phi[1] + phi[2] * phi[2];
        let theta = sqrtf(theta_sq);

        let w_hat = Self::hat(phi);
        // w_hat^2 계산
        let mut w_hat2 = [[0.0f32; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                w_hat2[i][j] = w_hat[i][0] * w_hat[0][j]
                    + w_hat[i][1] * w_hat[1][j]
                    + w_hat[i][2] * w_hat[2][j];
            }
        }

        let (a, b) = if theta < 1e-4 {
            // Taylor expansion: sin(t)/t ~ 1 - t^2/6, (1-cos(t))/t^2 ~ 0.5 - t^2/24
            let a = 1.0 - theta_sq / 6.0;
            let b = 0.5 - theta_sq / 24.0;
            (a, b)
        } else {
            let a = sinf(theta) / theta;
            let b = (1.0 - cosf(theta)) / theta_sq;
            (a, b)
        };

        let mut r = [[0.0f32; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                let delta = if i == j { 1.0 } else { 0.0 };
                r[i][j] = delta + a * w_hat[i][j] + b * w_hat2[i][j];
            }
        }

        Self { data: r }
    }

    /// 로그 사상 (Logarithm Map): SO(3) -> so(3)
    /// 3x3 회전 행렬을 회전 벡터 phi in R^3 로 변환
    pub fn log(&self) -> [f32; 3] {
        let trace = self.data[0][0] + self.data[1][1] + self.data[2][2];
        let cos_theta = 0.5 * (trace - 1.0);
        let cos_theta_clamped = if cos_theta > 1.0 {
            1.0
        } else if cos_theta < -1.0 {
            -1.0
        } else {
            cos_theta
        };

        let theta = acosf(cos_theta_clamped);
        let sin_theta = sinf(theta);

        if theta < 1e-4 || sin_theta.abs() < 1e-6 {
            // Taylor approximation near identity
            let factor = 0.5 * (1.0 + theta * theta / 6.0);
            [
                factor * (self.data[2][1] - self.data[1][2]),
                factor * (self.data[0][2] - self.data[2][0]),
                factor * (self.data[1][0] - self.data[0][1]),
            ]
        } else {
            let factor = theta / (2.0 * sin_theta);
            [
                factor * (self.data[2][1] - self.data[1][2]),
                factor * (self.data[0][2] - self.data[2][0]),
                factor * (self.data[1][0] - self.data[0][1]),
            ]
        }
    }

    /// 단위 쿼터니언 변환: [qw, qx, qy, qz]
    /// Shepperd 최적 안정화 알고리즘 기반
    pub fn to_quaternion(&self) -> [f32; 4] {
        let r = &self.data;
        let trace = r[0][0] + r[1][1] + r[2][2];

        if trace > 0.0 {
            let s = 0.5 / sqrtf(trace + 1.0);
            [
                0.25 / s,
                (r[2][1] - r[1][2]) * s,
                (r[0][2] - r[2][0]) * s,
                (r[1][0] - r[0][1]) * s,
            ]
        } else if r[0][0] > r[1][1] && r[0][0] > r[2][2] {
            let s = 2.0 * sqrtf(1.0 + r[0][0] - r[1][1] - r[2][2]);
            [
                (r[2][1] - r[1][2]) / s,
                0.25 * s,
                (r[0][1] + r[1][0]) / s,
                (r[0][2] + r[2][0]) / s,
            ]
        } else if r[1][1] > r[2][2] {
            let s = 2.0 * sqrtf(1.0 + r[1][1] - r[0][0] - r[2][2]);
            [
                (r[0][2] - r[2][0]) / s,
                (r[0][1] + r[1][0]) / s,
                0.25 * s,
                (r[1][2] + r[2][1]) / s,
            ]
        } else {
            let s = 2.0 * sqrtf(1.0 + r[2][2] - r[0][0] - r[1][1]);
            [
                (r[1][0] - r[0][1]) / s,
                (r[0][2] + r[2][0]) / s,
                (r[1][2] + r[2][1]) / s,
                0.25 * s,
            ]
        }
    }

    /// ZYX 오일러 각 변환: (Roll, Pitch, Yaw) in Degrees
    pub fn to_euler_deg(&self) -> (f32, f32, f32) {
        let r = &self.data;
        // Pitch: arcsin(-R[2][0])
        let sin_pitch = -r[2][0];
        let sin_pitch_clamped = if sin_pitch > 1.0 {
            1.0
        } else if sin_pitch < -1.0 {
            -1.0
        } else {
            sin_pitch
        };
        let pitch_rad = asinf(sin_pitch_clamped);

        let (roll_rad, yaw_rad) = if sin_pitch_clamped.abs() < 0.9999 {
            let roll = atan2f(r[2][1], r[2][2]);
            let yaw = atan2f(r[1][0], r[0][0]);
            (roll, yaw)
        } else {
            // Gimbal lock singularity fallback
            let roll = 0.0;
            let yaw = atan2f(-r[0][1], r[1][1]);
            (roll, yaw)
        };

        const RAD_TO_DEG: f32 = 180.0 / core::f32::consts::PI;
        (roll_rad * RAD_TO_DEG, pitch_rad * RAD_TO_DEG, yaw_rad * RAD_TO_DEG)
    }
}
