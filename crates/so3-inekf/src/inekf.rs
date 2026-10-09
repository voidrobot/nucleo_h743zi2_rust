//! # 6-DOF Right-Invariant InEKF (Invariant Extended Kalman Filter)
//!
//! Implements orientation and gyro bias estimation on the SO(3) Lie Group.
//!
//! Mathematical Properties:
//! - State: Attitude R in SO(3), Gyroscope Bias b_w in R^3
//! - Error Definition (Right-Invariant): \eta = \hat{R} R^-1 \approx I + [\xi]_\times
//! - Constant Measurement Jacobians: H_acc = [-g]_\times, H_mag = [-m]_\times
//! - Joseph Form Covariance Update for absolute numerical symmetry and positive-definiteness

use crate::so3::So3;
use libm::fabsf;

/// 3x3 대칭 양의 정부호 행렬의 역행렬 계산 (여인수 행렬식 방식)
pub fn invert_3x3(m: [[f32; 3]; 3]) -> Option<[[f32; 3]; 3]> {
    let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

    if fabsf(det) < 1e-9 {
        return None;
    }

    let inv_det = 1.0 / det;
    let mut inv = [[0.0f32; 3]; 3];

    inv[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * inv_det;
    inv[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inv_det;
    inv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv_det;

    inv[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * inv_det;
    inv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv_det;
    inv[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv_det;

    inv[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * inv_det;
    inv[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv_det;
    inv[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv_det;

    Some(inv)
}

/// 6차원 우불변 칼만 필터 구조체
pub struct RightInvariantInEKF {
    /// 공칭 자세 상태 \hat{R} \in SO(3)
    pub rot: So3,
    /// 공칭 3축 자이로스코프 바이어스 (rad/s)
    pub bias_gyro: [f32; 3],
    /// 6x6 오차 공분산 행렬 P (0..3: 자세 오차 \xi, 3..6: 바이어스 오차 \delta b)
    pub p: [[f32; 6]; 6],
    /// 자이로 노이즈 분산
    pub q_gyro: f32,
    /// 자이로 바이어스 랜덤워크 분산
    pub q_bias: f32,
    /// 가속도계 관측 노이즈 분산
    pub r_accel: f32,
    /// 지자기 센서 관측 노이즈 분산
    pub r_mag: f32,
    /// 기준 중력 벡터 (NED/ENU 좌표계 기준, 기본값 [0, 0, 9.80665])
    pub g_ref: [f32; 3],
    /// 기준 지구 자기장 벡터
    pub m_ref: [f32; 3],
}

impl RightInvariantInEKF {
    /// 초기 파라미터로 InEKF 인스턴스 생성
    pub fn new() -> Self {
        let mut p = [[0.0f32; 6]; 6];
        // 초기 공분산 설정 (자세 0.1 rad^2, 바이어스 0.01 (rad/s)^2)
        for i in 0..3 {
            p[i][i] = 0.1;
            p[i + 3][i + 3] = 0.01;
        }

        Self {
            rot: So3::identity(),
            bias_gyro: [0.0; 3],
            p,
            q_gyro: 1e-3,       // 0.001 (rad/s)^2/Hz
            q_bias: 1e-5,       // 0.00001 (rad/s^2)^2/Hz
            r_accel: 0.2,       // 가속도계 노이즈
            r_mag: 0.1,         // 지자기 센서 노이즈
            g_ref: [0.0, 0.0, 9.80665],
            m_ref: [0.35, 0.0, 0.45], // 표준 지구 자기장 정규화 벡터 (북향/하향 성분)
        }
    }

    /// 공분산 대각합(Trace) 계산 (수렴도 지표)
    pub fn cov_trace(&self) -> f32 {
        let mut trace = 0.0;
        for i in 0..6 {
            trace += self.p[i][i];
        }
        trace
    }

    /// [1단계: 100 Hz 전파 (Propagation / Predict)]
    /// 자이로스코프 측정값(rad/s)과 샘플 주기 dt(s)를 이용한 상태 및 공분산 전파
    pub fn predict(&mut self, gyro_raw: [f32; 3], dt: f32) {
        // 1. 바이어스 보정된 순수 각속도
        let w_unb = [
            gyro_raw[0] - self.bias_gyro[0],
            gyro_raw[1] - self.bias_gyro[1],
            gyro_raw[2] - self.bias_gyro[2],
        ];

        // 2. 공칭 자세 갱신: \hat{R}_{k+1} = \hat{R}_k * exp(w_unb * dt)
        let phi = [w_unb[0] * dt, w_unb[1] * dt, w_unb[2] * dt];
        let delta_rot = So3::exp(phi);
        self.rot = self.rot.mul(&delta_rot);

        // 3. 상태 전이 행렬 F (6x6) 구성
        //    \dot{\xi} = -\hat{R} * \delta b_w  ==> F_12 = -\hat{R} * dt
        let r_dt = [
            [self.rot.data[0][0] * dt, self.rot.data[0][1] * dt, self.rot.data[0][2] * dt],
            [self.rot.data[1][0] * dt, self.rot.data[1][1] * dt, self.rot.data[1][2] * dt],
            [self.rot.data[2][0] * dt, self.rot.data[2][1] * dt, self.rot.data[2][2] * dt],
        ];

        let mut f = [[0.0f32; 6]; 6];
        for i in 0..6 {
            f[i][i] = 1.0;
        }
        for i in 0..3 {
            for j in 0..3 {
                f[i][j + 3] = -r_dt[i][j];
            }
        }

        // 4. 공분산 전파: P_new = F * P * F^T + Q * dt
        let mut fp = [[0.0f32; 6]; 6];
        for i in 0..6 {
            for j in 0..6 {
                let mut sum = 0.0;
                for k in 0..6 {
                    sum += f[i][k] * self.p[k][j];
                }
                fp[i][j] = sum;
            }
        }

        let mut p_new = [[0.0f32; 6]; 6];
        for i in 0..6 {
            for j in 0..6 {
                let mut sum = 0.0;
                for k in 0..6 {
                    sum += fp[i][k] * f[j][k]; // F^T[k][j] = F[j][k]
                }
                p_new[i][j] = sum;
            }
        }

        // Q * dt 더하기
        for i in 0..3 {
            p_new[i][i] += self.q_gyro * dt;
            p_new[i + 3][i + 3] += self.q_bias * dt;
        }

        self.p = p_new;
    }

    /// [2단계: 가속도계 중력 관측 갱신 (Accelerometer Update)]
    /// Right-Invariant 오차 모델: 혁신 z = \hat{R} * y_acc - g_ref, 상수 야코비 H = [-g_ref]_\times
    pub fn update_accel(&mut self, accel_raw_mps2: [f32; 3]) -> bool {
        // 동적 외란 제거 (0.85g ~ 1.15g 범위 밖은 중력 방향 왜곡으로 간주하여 스킵)
        let acc_mag_sq = accel_raw_mps2[0] * accel_raw_mps2[0]
            + accel_raw_mps2[1] * accel_raw_mps2[1]
            + accel_raw_mps2[2] * accel_raw_mps2[2];
        let g_sq = self.g_ref[0] * self.g_ref[0] + self.g_ref[1] * self.g_ref[1] + self.g_ref[2] * self.g_ref[2];
        let ratio = acc_mag_sq / g_sq;
        if ratio < 0.7 || ratio > 1.3 {
            return false;
        }

        self.update_vector_observation(accel_raw_mps2, self.g_ref, self.r_accel)
    }

    /// [3단계: 지자기 센서 관측 갱신 (Magnetometer Update)]
    /// Right-Invariant 오차 모델: 혁신 z = \hat{R} * y_mag - m_ref, 상수 야코비 H = [-m_ref]_\times
    pub fn update_mag(&mut self, mag_raw_norm: [f32; 3]) -> bool {
        self.update_vector_observation(mag_raw_norm, self.m_ref, self.r_mag)
    }

    /// 우불변 벡터 관측 갱신 내부 공통 엔진 (상수 야코비 + Joseph Form 갱신)
    fn update_vector_observation(&mut self, y_body: [f32; 3], v_ref: [f32; 3], r_noise: f32) -> bool {
        // 1. 공간 투영 혁신: z = \hat{R} * y_body - v_ref (3x1)
        let y_spatial = self.rot.rotate_vec(y_body);
        let z = [
            y_spatial[0] - v_ref[0],
            y_spatial[1] - v_ref[1],
            y_spatial[2] - v_ref[2],
        ];

        // 2. 상수 야코비 H = [-v_ref]_\times  (3x6, 바이어스 부분은 0)
        let neg_v_hat = So3::hat([-v_ref[0], -v_ref[1], -v_ref[2]]);
        let mut h = [[0.0f32; 6]; 3];
        for i in 0..3 {
            for j in 0..3 {
                h[i][j] = neg_v_hat[i][j];
            }
        }

        // 3. 혁신 공분산 S = H * P * H^T + R * I (3x3)
        let mut hp = [[0.0f32; 6]; 3];
        for i in 0..3 {
            for j in 0..6 {
                let mut sum = 0.0;
                for k in 0..6 {
                    sum += h[i][k] * self.p[k][j];
                }
                hp[i][j] = sum;
            }
        }

        let mut s = [[0.0f32; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                let mut sum = 0.0;
                for k in 0..6 {
                    sum += hp[i][k] * h[j][k];
                }
                s[i][j] = sum;
            }
            s[i][i] += r_noise;
        }

        // 4. S의 역행렬 계산
        let s_inv = match invert_3x3(s) {
            Some(inv) => inv,
            None => return false,
        };

        // 5. 칼만 게인 K = P * H^T * S^-1 (6x3)
        let mut p_ht = [[0.0f32; 3]; 6];
        for i in 0..6 {
            for j in 0..3 {
                let mut sum = 0.0;
                for k in 0..6 {
                    sum += self.p[i][k] * h[j][k]; // H^T[k][j] = H[j][k]
                }
                p_ht[i][j] = sum;
            }
        }

        let mut k_gain = [[0.0f32; 3]; 6];
        for i in 0..6 {
            for j in 0..3 {
                let mut sum = 0.0;
                for l in 0..3 {
                    sum += p_ht[i][l] * s_inv[l][j];
                }
                k_gain[i][j] = sum;
            }
        }

        // 6. 상태 보정량 계산: \Delta x = K * z (6x1)
        let mut delta_x = [0.0f32; 6];
        for i in 0..6 {
            delta_x[i] = k_gain[i][0] * z[0] + k_gain[i][1] * z[1] + k_gain[i][2] * z[2];
        }

        // 7. 우불변 매니폴드 상태 복귀 (Manifold Retraction)
        //    \eta = \hat{R} R^-1 \implies R = \eta^-1 \hat{R} \approx exp(-\xi) * \hat{R}
        let xi = [delta_x[0], delta_x[1], delta_x[2]];
        let exp_neg_xi = So3::exp([-xi[0], -xi[1], -xi[2]]);
        self.rot = exp_neg_xi.mul(&self.rot);

        //    \hat{b} \leftarrow \hat{b} + \delta b
        self.bias_gyro[0] += delta_x[3];
        self.bias_gyro[1] += delta_x[4];
        self.bias_gyro[2] += delta_x[5];

        // 8. 조셉 형태(Joseph Form) 공분산 갱신: P = (I - K*H) * P * (I - K*H)^T + K * R * K^T
        let mut kh = [[0.0f32; 6]; 6];
        for i in 0..6 {
            for j in 0..6 {
                let mut sum = 0.0;
                for l in 0..3 {
                    sum += k_gain[i][l] * h[l][j];
                }
                kh[i][j] = sum;
            }
        }

        let mut i_kh = [[0.0f32; 6]; 6];
        for i in 0..6 {
            for j in 0..6 {
                let delta = if i == j { 1.0 } else { 0.0 };
                i_kh[i][j] = delta - kh[i][j];
            }
        }

        let mut ikh_p = [[0.0f32; 6]; 6];
        for i in 0..6 {
            for j in 0..6 {
                let mut sum = 0.0;
                for l in 0..6 {
                    sum += i_kh[i][l] * self.p[l][j];
                }
                ikh_p[i][j] = sum;
            }
        }

        let mut p_updated = [[0.0f32; 6]; 6];
        for i in 0..6 {
            for j in 0..6 {
                let mut sum = 0.0;
                for l in 0..6 {
                    sum += ikh_p[i][l] * i_kh[j][l]; // (I-KH)^T[l][j] = (I-KH)[j][l]
                }
                p_updated[i][j] = sum;
            }
        }

        // K * R * K^T 더하기
        for i in 0..6 {
            for j in 0..6 {
                let mut sum = 0.0;
                for l in 0..3 {
                    sum += k_gain[i][l] * k_gain[j][l];
                }
                p_updated[i][j] += sum * r_noise;
            }
        }

        self.p = p_updated;
        true
    }
}
