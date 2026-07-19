// 페달의 밟기에 따라 3상 교류 전류의 PWM 듀티 비 계산 파일

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>

#include "shared_types.h"

#define DT 0.0001f          // 10kHz 제어 주기 (0.1ms)
#define PI 3.14159265f

// 제어용 글로벌 변수 및 게인 설정
static float error_d_sum = 0.0f;
static float error_q_sum = 0.0f;
const float Kp_d = 1.5f, Ki_d = 200.0f; // d축 PI 게인
const float Kp_q = 1.5f, Ki_q = 200.0f; // q축 PI 게인

// ====================================================================
// 3단계: 전류 지령 계산 (MTPA 효율 표 기반 분리)
// ====================================================================
void mtpa_current_mapping(float torque_cmd, float* id_ref, float* iq_ref) {
    // d축 : 자석 N극의 정면, 자석의 힘
    // q축 : 자석과 자석 사이 공간, 토크
    // 의도 : 회전하는 3차원 => 회전하는 자석 위에서 바라보자.
    // 회전문에서 기둥이 아닌 문을 앞 뒤로, 좌 우로 민다고 상상해보기. => 전기 모터도 이와 유사한 구조를 지니고 있음.

    // MTPA(최대 토크당 최소 전류)를 단순화한 수학적 모델링
    // 효율을 위해 q축 전류는 토크에 비례하고, d축 전류는 약하게 음수 방향으로 제어함
    const float KT = 2.0f; // 토크 상수 상수값 대입
    
    *iq_ref = torque_cmd / KT;             // 토크를 직접 만드는 힘 전류
    *id_ref = -0.1f * (*iq_ref);           // 전기를 아끼기 위해 자속을 약간 깎음 (MTPA 특성)
    
    // 물리적인 배터리 최대 전류 한계치로 제한 (클램핑)
    const float MAX_CURRENT = 250.0f; // A
    if (*iq_ref > MAX_CURRENT) *iq_ref = MAX_CURRENT;
    if (*id_ref < -MAX_CURRENT) *id_ref = -MAX_CURRENT;
}

// ====================================================================
// 4단계: 전압 제어 (PI 제어기로 전류 오차 추종)
// ====================================================================
void current_pi_control(float id_ref, float iq_ref, float id_actual, float iq_actual, float* vd_cmd, float* vq_cmd) {
    // 안티 윈드업
    // 사용자가 페달을 계속 밟아서 iq_ref이 계속 늘어나서 MAX_VOLTAGE 이상의 값을 가졌다고 가정.
    // 페달에 발을 떼거나 브레이크를 눌러도 모터 전압은 한동안 400V로 유지될 가능성이 있음.
    // 그 과정에서 부품이 타버리거나 교통사고가 날 수 있으니 I 제어는 최대전압 이하일 때만 누적합을 하도록 수정
    
    const float MAX_VOLTAGE = 400.0f;

    float error_d = id_ref - id_actual;
    
    // 임시로 PI 제어 전압 계산
    float vd_tentative = (Kp_d * error_d) + (Ki_d * (error_d_sum + error_d * DT));

    // 제한 범위 안에 있을 때만 오차를 누적 (포화 방지)
    if (vd_tentative <= MAX_VOLTAGE && vd_tentative >= -MAX_VOLTAGE) {
        error_d_sum += error_d * DT;
        *vd_cmd = vd_tentative;
    } else {
        // 한계를 벗어났다면 전압을 한계값으로 고정
        *vd_cmd = (vd_tentative > MAX_VOLTAGE) ? MAX_VOLTAGE : -MAX_VOLTAGE;
    }

    float error_q = iq_ref - iq_actual;
    
    // 임시로 q축 전압 계산
    float vq_tentative = (Kp_q * error_q) + (Ki_q * (error_q_sum + error_q * DT));

    // 제한 범위 안에 있을 때만 오차를 누적
    if (vq_tentative <= MAX_VOLTAGE && vq_tentative >= -MAX_VOLTAGE) {
        error_q_sum += error_q * DT;
        *vq_cmd = vq_tentative;
    } else {
        *vq_cmd = (vq_tentative > MAX_VOLTAGE) ? MAX_VOLTAGE : -MAX_VOLTAGE;
    }
}

// ====================================================================
// 5단계: 역 변환 및 PWM 출력 (가상 dq축 전압 ➡️ 현실 3상 듀티 제어)
// ====================================================================
PWMOutputs inverse_dq_and_pwm(float vd, float vq, float theta) {
    PWMOutputs outputs;
    const float DC_LINK_VOLTAGE = 400.0f; // 배터리 전압 값
    
    // 1. 역 박 변환 (Inverse Park Transformation): dq축 ➡️ 정지 고정 좌표계 αβ축
    float v_alpha = vd * cosf(theta) - vq * sinf(theta);
    float v_beta  = vd * sinf(theta) + vq * cosf(theta);
    
    // 2. 역 클라크 변환 (Inverse Clarke Transformation): αβ축 ➡️ 현실의 3상 전압 Va, Vb, Vc
    float va = v_alpha;
    float vb = -0.5f * v_alpha + (sqrtf(3.0f) / 2.0f) * v_beta;
    float vc = -0.5f * v_alpha - (sqrtf(3.0f) / 2.0f) * v_beta;
    
    // 3. PWM 듀티 비 계산 (전압을 타이머 스위칭 시간인 0.0 ~ 1.0 비율로 변환)
    // 고정된 배터리팩 전압을 우리가 원하는 전압으로 바꿔야 함.
    // 엄청 빠른 속도로 껐다 켰다를 반복하면 원하는 전압을 맞출 수 있음.
    // 0.5f는 전압의 특성 때문임.
    // +, -로 보내야되기에 0.5를 중간 지점으로 설정
    outputs.duty_A = (va / DC_LINK_VOLTAGE) + 0.5f;
    outputs.duty_B = (vb / DC_LINK_VOLTAGE) + 0.5f;
    outputs.duty_C = (vc / DC_LINK_VOLTAGE) + 0.5f;
    
    // 하드웨어 타이머 범위를 벗어나지 않게 클램핑 (0% ~ 100%)
    if (outputs.duty_A < 0.0f) outputs.duty_A = 0.0f; if (outputs.duty_A > 1.0f) outputs.duty_A = 1.0f;
    if (outputs.duty_B < 0.0f) outputs.duty_B = 0.0f; if (outputs.duty_B > 1.0f) outputs.duty_B = 1.0f;
    if (outputs.duty_C < 0.0f) outputs.duty_C = 0.0f; if (outputs.duty_C > 1.0f) outputs.duty_C = 1.0f;
    
    return outputs;
}