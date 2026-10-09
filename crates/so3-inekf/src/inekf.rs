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

    // [정지 상태(Stillness / ZARU) 감지기 필드]
    pub is_stationary: bool,
    pub stationary_counter: u32,
    pub gyro_mean: [f32; 3],
    pub gyro_var: f32,
    pub accel_mean: [f32; 3],
    pub accel_var: f32,
}

impl Default for RightInvariantInEKF {
    fn default() -> Self {
        Self::new()
    }
}

impl RightInvariantInEKF {
    /// 초기 파라미터로 InEKF 인스턴스 생성 (const fn 지원)
    pub const fn new() -> Self {
        let mut p = [[0.0f32; 6]; 6];
        // 초기 공분산 설정 (자세 0.1 rad^2, 바이어스 0.01 (rad/s)^2)
        p[0][0] = 0.1;
        p[1][1] = 0.1;
        p[2][2] = 0.1;
        p[3][3] = 0.01;
        p[4][4] = 0.01;
        p[5][5] = 0.01;

        Self {
            rot: So3::identity(),
            bias_gyro: [0.0; 3],
            p,
            q_gyro: 1e-3,       // 0.001 (rad/s)^2/Hz
            q_bias: 1e-5,       // 0.00001 (rad/s^2)^2/Hz
            r_accel: 0.2,       // 기본 가속도계 노이즈
            r_mag: 0.1,         // 기본 지자기 센서 노이즈
            g_ref: [0.0, 0.0, 9.80665],
            m_ref: [0.35, 0.0, 0.45], // 표준 지구 자기장 정규화 벡터 (북향/하향 성분)
            is_stationary: false,
            stationary_counter: 0,
            gyro_mean: [0.0; 3],
            gyro_var: 0.0,
            accel_mean: [0.0, 0.0, 9.80665],
            accel_var: 0.0,
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

    /// [정지 상태(Stillness / ZARU) 감지 및 바이어스 적응 갱신]
    /// 가속도 및 자이로스코프의 1차 이동평균과 분산 추이를 관측하여 정지 상태를 판별.
    /// 정지 상태 시 자이로 바이어스를 실측값으로 정밀 흡수(ZARU)하고 Yaw 드리프트를 동결한다.
    pub fn update_stillness(&mut self, gyro_raw: [f32; 3], accel_mps2: [f32; 3]) -> bool {
        const ALPHA: f32 = 0.05; // 100Hz 기준 약 20~50 샘플(0.2~0.5초) 시정수 지수이동평균

        let mut g_diff_sq = 0.0f32;
        let mut a_diff_sq = 0.0f32;
        for i in 0..3 {
            let diff_g = gyro_raw[i] - self.gyro_mean[i];
            g_diff_sq += diff_g * diff_g;
            self.gyro_mean[i] += ALPHA * diff_g;

            let diff_a = accel_mps2[i] - self.accel_mean[i];
            a_diff_sq += diff_a * diff_a;
            self.accel_mean[i] += ALPHA * diff_a;
        }

        self.gyro_var = (1.0 - ALPHA) * self.gyro_var + ALPHA * g_diff_sq;
        self.accel_var = (1.0 - ALPHA) * self.accel_var + ALPHA * a_diff_sq;

        let a_norm = libm::sqrtf(
            self.accel_mean[0] * self.accel_mean[0]
                + self.accel_mean[1] * self.accel_mean[1]
                + self.accel_mean[2] * self.accel_mean[2],
        );
        let g_diff = libm::fabsf(a_norm - 9.80665);

        // 3대 물리적 정지 조건:
        // 1. 각속도 분산 < 0.0005 (rad/s)^2 (약 1.3 dps 이하 진동)
        // 2. 가속도 분산 < 0.05 (m/s^2)^2 (선형 가속/충격 없음)
        // 3. 중력 크기 오차 < 0.5 m/s^2 (순수 1.0g 중력장)
        let is_still_instant = self.gyro_var < 0.0005 && self.accel_var < 0.05 && g_diff < 0.5;

        if is_still_instant {
            if self.stationary_counter < 30 {
                self.stationary_counter += 1;
            } else {
                self.is_stationary = true;
            }
        } else {
            self.stationary_counter = 0;
            self.is_stationary = false;
        }

        // 정지 상태일 때: ZARU(Zero Angular Rate Update)
        // 정지 중 계측되는 자이로 값은 100% 바이어스이므로 자이로 바이어스를 점진 흡수
        if self.is_stationary {
            const BIAS_LEARN_RATE: f32 = 0.002;
            for i in 0..3 {
                self.bias_gyro[i] += BIAS_LEARN_RATE * (gyro_raw[i] - self.bias_gyro[i]);
            }
        }

        self.is_stationary
    }

    /// [1단계: 100 Hz 전파 (Propagation / Predict)]
    /// 자이로스코프 측정값(rad/s)과 샘플 주기 dt(s)를 이용한 상태 및 공분산 전파
    pub fn predict(&mut self, gyro_raw: [f32; 3], dt: f32) {
        // 1. 바이어스 보정된 순수 각속도
        let mut w_unb = [
            gyro_raw[0] - self.bias_gyro[0],
            gyro_raw[1] - self.bias_gyro[1],
            gyro_raw[2] - self.bias_gyro[2],
        ];

        // 정지 상태 시 물리적 진실에 따라 모든 각속도 적분을 완전히 동결(0.0)하여 Yaw 드리프트를 영구 소거
        if self.is_stationary {
            w_unb[0] = 0.0;
            w_unb[1] = 0.0;
            w_unb[2] = 0.0;
        }

        // 2. 공칭 자세 갱신: \hat{R}_{k+1} = \hat{R}_k * exp(w_unb * dt)
        let phi = [w_unb[0] * dt, w_unb[1] * dt, w_unb[2] * dt];
        let delta_rot = So3::exp(phi);
        self.rot = self.rot.mul(&delta_rot);

        // 3. 상태 전이 행렬 F (6x6) 구성
        //    \dot{\xi} = +\hat{R} * \delta b_w  ==> F_12 = +\hat{R} * dt
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
                // Right-Invariant 오차 역학: \dot{\xi} = +\hat{R} * \delta b_w
                f[i][j + 3] = r_dt[i][j];
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
    /// 가속도 바이어스는 기동 가속도와의 역학적 커플링 오동작을 방지하기 위해 추정하지 않음 (사용자 요구사항 반영).
    /// 정지 상태에서는 중력 정렬을 강화(R=0.04)하고, 동적 기동 시에는 적응형 노이즈 스케일링으로 모션 왜곡을 차단한다.
    pub fn update_accel(&mut self, accel_raw_mps2: [f32; 3]) -> bool {
        let acc_mag_sq = accel_raw_mps2[0] * accel_raw_mps2[0]
            + accel_raw_mps2[1] * accel_raw_mps2[1]
            + accel_raw_mps2[2] * accel_raw_mps2[2];
        let acc_mag = libm::sqrtf(acc_mag_sq);
        let g_norm = 9.80665f32;
        let g_diff = libm::fabsf(acc_mag - g_norm);

        // 1. 극단적 외란(자유낙하, 강한 충격) 기각 (0.5g ~ 1.8g 범위)
        if acc_mag < 0.5 * g_norm || acc_mag > 1.8 * g_norm {
            return false;
        }

        // 2. 적응형 측정 공분산(Adaptive Noise Covariance Scaling)
        let r_noise = if self.is_stationary {
            // 정지 상태: 순수 중력장이므로 측정 노이즈를 0.04로 축소하여 Roll/Pitch 신속 정렬
            0.04
        } else {
            // 동적 기동 상태: 선형 가속도 오차와 가속도 분산에 비례하여 노이즈를 수십 배 확대, 자이로 적분에 주도권 위임
            self.r_accel * (1.0 + 8.0 * g_diff + 20.0 * self.accel_var)
        };

        self.update_vector_observation(accel_raw_mps2, self.g_ref, r_noise)
    }

    /// [3단계: 지자기 센서 관측 갱신 (Decoupled 1D Yaw Update)]
    /// 수평면 투영(Tilt-compensated Decoupled Yaw) 기법을 적용하여
    /// 지자기 센서의 왜곡이 가속도계의 수평 자세(Roll/Pitch)를 절대 교란하지 못하도록
    /// 순수 방위각(Yaw, \xi_z) 성분만 1D 칼만 갱신으로 독립 보정한다.
    pub fn update_mag(&mut self, mag_raw_norm: [f32; 3]) -> bool {
        // 1. 공간 좌표계로 지자기 벡터 투영: h = \hat{R} * y_mag
        let h = self.rot.rotate_vec(mag_raw_norm);

        // 2. 수평면 투영 크기 계산
        let h_horiz_sq = h[0] * h[0] + h[1] * h[1];
        if h_horiz_sq < 1e-4 {
            return false; // 특이점(수직 복각 극지방) 방어
        }

        // 3. 자북 헤딩 오차각 (공간 좌표계 X축 북향 기준 Yaw 각도 오차)
        //    h_x > 0, h_y = 0 일 때 헤딩 오차가 0
        let delta_yaw = libm::atan2f(h[1], h[0]);

        // 4. Decoupled 1D 스칼라 칼만 갱신 (오직 Yaw 오차 \xi_z 만 관측)
        //    관측 야코비 H = [0, 0, 1, 0, 0, 0] (1x6)
        //    혁신 공분산 S = H * P * H^T + R = P[2][2] + R_mag
        let s = self.p[2][2] + self.r_mag;
        if s < 1e-6 {
            return false;
        }
        let s_inv = 1.0 / s;

        // 칼만 게인 K = P * H^T * S^-1 (6x1)
        // H^T는 index 2만 1.0이므로 P * H^T는 P의 2번째 열(column 2)
        let mut k_gain = [0.0f32; 6];
        for i in 0..6 {
            k_gain[i] = self.p[i][2] * s_inv;
        }

        // 상태 오차 보정량: Yaw 회전 오차 \Delta \xi_z
        let delta_xi_z = k_gain[2] * delta_yaw;
        let exp_neg_yaw = So3::exp([0.0, 0.0, -delta_xi_z]);
        self.rot = exp_neg_yaw.mul(&self.rot);

        // Z축 자이로 바이어스 완만 보정 (정지 상태가 아닐 때만 칼만 게인 반영)
        if !self.is_stationary {
            self.bias_gyro[2] += k_gain[5] * delta_yaw;
        }

        // 조셉 형태(Joseph Form) 공분산 갱신: P = (I - K*H) * P * (I - K*H)^T + K * R * K^T
        let mut a = [[0.0f32; 6]; 6];
        for i in 0..6 {
            for j in 0..6 {
                let delta = if i == j { 1.0 } else { 0.0 };
                let kh = if j == 2 { k_gain[i] } else { 0.0 };
                a[i][j] = delta - kh;
            }
        }

        let mut ap = [[0.0f32; 6]; 6];
        for i in 0..6 {
            for j in 0..6 {
                let mut sum = 0.0;
                for k in 0..6 {
                    sum += a[i][k] * self.p[k][j];
                }
                ap[i][j] = sum;
            }
        }

        let mut p_new = [[0.0f32; 6]; 6];
        for i in 0..6 {
            for j in 0..6 {
                let mut sum = 0.0;
                for k in 0..6 {
                    sum += ap[i][k] * a[j][k]; // A^T[k][j] = A[j][k]
                }
                // K * R * K^T 성분 더하기
                p_new[i][j] = sum + k_gain[i] * k_gain[j] * self.r_mag;
            }
        }

        self.p = p_new;
        true
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
