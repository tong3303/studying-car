// 페달의 밟기에 따라 3상 교류 전류의 PWM 듀티 비 계산 파일

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>

#define DT 0.0001f          // 10kHz 제어 주기 (0.1ms)
#define PI 3.14159265f

// 제어용 글로벌 변수 및 게인 설정
float error_d_sum = 0.0f;
float error_q_sum = 0.0f;
const float Kp_d = 1.5f, Ki_d = 200.0f; // d축 PI 게인
const float Kp_q = 1.5f, Ki_q = 200.0f; // q축 PI 게인

// 가상의 모터 센서 및 하드웨어 인터페이스를 위한 구조체
typedef struct {
    uint16_t adc_sensor1;   // 가속 페달 센서 1
    uint16_t adc_sensor2;   // 가속 페달 센서 2
    float vehicle_speed;    // 현재 차량 속도 (km/h)
    float id_actual;        // 전류 센서가 읽은 실제 d축 전류
    float iq_actual;        // 전류 센서가 읽은 실제 q축 전류
    float motor_angle;      // 모터 회전 로터의 전기적 각도 (라디안)
} VehicleHardwareInputs;

// 파이프라인 출력 데이터 구조체
typedef struct {
    float duty_A;
    float duty_B;
    float duty_C;
} PWMOutputs;


// ====================================================================
// 1단계: 페달 입력 센싱 (각도 ➡️ 디지털 숫자 ➡️ 0 ~ 100%)
// ====================================================================
float step1_pedal_sensing(uint16_t adc1, uint16_t adc2) {
    uint16_t adc_min = 500;   // 0.5V에 대응하는 디지털 값
    uint16_t adc_max = 4000;  // 4.5V에 대응하는 디지털 값
    
    // 센서 1 매핑
    float pedal1 = 0.0f;
    if (adc1 >= adc_min) {
        pedal1 = ((float)(adc1 - adc_min) / (adc_max - adc_min)) * 100.0f;
    }
    if (pedal1 > 100.0f) pedal1 = 100.0f;

    // 센서 2 매핑
    float pedal2 = 0.0f;
    if (adc2 >= adc_min) {
        pedal2 = ((float)(adc2 - adc_min) / (adc_max - adc_min)) * 100.0f;
    }
    if (pedal2 > 100.0f) pedal2 = 100.0f;

    // 🔒 페일 세이프 (이중화 검증): 두 센서 오차가 5% 이상이면 안전을 위해 0% 반환
    if (fabs(pedal1 - pedal2) > 5.0f) {
        // 실제 차량이라면 여기서 Fault 인터럽트를 발생시키고 모터를 끕니다.
        return 0.0f; 
    }

    return (pedal1 + pedal2) / 2.0f; // 안전할 경우 평균값 적용
}

// ====================================================================
// 2단계: 토크 지령 계산 (속도와 페달을 고려한 2D 매핑 변형)
// ====================================================================
float step2_calculate_torque(float pedal_percent, float speed) {
    // 최대 토크 한계 정의
    const float MAX_TORQUE = 350.0f; // Nm
    
    // 단순화된 룩업 테이블 대용 수식 (기본 토크 생성)
    float base_torque = (pedal_percent / 100.0f) * MAX_TORQUE;
    
    // 🏎️ 차량 속도가 너무 높으면 모터 특성상 낼 수 있는 토크가 줄어듭니다 (감쇄 로직)
    float speed_factor = 1.0f;
    if (speed > 80.0f) {
        speed_factor = 1.0f - ((speed - 80.0f) / 120.0f); // 200km/h에서 0이 됨
        if (speed_factor < 0.0f) speed_factor = 0.0f;
    }

    return base_torque * speed_factor;
}

// ====================================================================
// 3단계: 전류 지령 계산 (MTPA 효율 표 기반 분리)
// ====================================================================
void step3_mtpa_current_mapping(float torque_cmd, float* id_ref, float* iq_ref) {
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
void step4_current_pi_control(float id_ref, float iq_ref, float id_actual, float iq_actual, float* vd_cmd, float* vq_cmd) {
    // d축 PI 제어 계산
    float error_d = id_ref - id_actual;
    error_d_sum += error_d * DT;
    *vd_cmd = (Kp_d * error_d) + (Ki_d * error_d_sum);

    // q축 PI 제어 계산
    float error_q = iq_ref - iq_actual;
    error_q_sum += error_q * DT;
    *vq_cmd = (Kp_q * error_q) + (Ki_q * error_q_sum);
    
    // 전압 제한: 배터리 전압(예: 400V)을 넘을 수 없도록 제한
    const float MAX_VOLTAGE = 400.0f;
    if (*vd_cmd > MAX_VOLTAGE)  *vd_cmd = MAX_VOLTAGE;
    if (*vd_cmd < -MAX_VOLTAGE) *vd_cmd = -MAX_VOLTAGE;
    if (*vq_cmd > MAX_VOLTAGE)  *vq_cmd = MAX_VOLTAGE;
    if (*vq_cmd < -MAX_VOLTAGE) *vq_cmd = -MAX_VOLTAGE;
}

// ====================================================================
// 5단계: 역 변환 및 PWM 출력 (가상 dq축 전압 ➡️ 현실 3상 듀티 제어)
// ====================================================================
PWMOutputs step5_inverse_dq_and_pwm(float vd, float vq, float theta) {
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


// ====================================================================
// 🚀 실시간 모터 제어 파이프라인 스케줄러 (10kHz 루프)
// ====================================================================
void motor_control_pipeline_tick(VehicleHardwareInputs hw_inputs) {
    printf("=== [Control Loop Tick] ===\n");

    // Step 1: 페달 입력 센싱
    float pedal_pct = step1_pedal_sensing(hw_inputs.adc_sensor1, hw_inputs.adc_sensor2);
    printf("[Step 1] Pedal Input     : %.1f %%\n", pedal_pct);

    // Step 2: 토크 지령 계산
    float torque_cmd = step2_calculate_torque(pedal_pct, hw_inputs.vehicle_speed);
    printf("[Step 2] Target Torque   : %.2f Nm (Speed: %.1f km/h)\n", torque_cmd, hw_inputs.vehicle_speed);

    // Step 3: 전류 지령 계산
    float id_ref = 0.0f, iq_ref = 0.0f;
    step3_mtpa_current_mapping(torque_cmd, &id_ref, &iq_ref);
    printf("[Step 3] Target Current  : Id_ref = %.2f A, Iq_ref = %.2f A\n", id_ref, iq_ref);

    // Step 4: 전압 제어 (PI)
    float vd_cmd = 0.0f, vq_cmd = 0.0f;
    step4_current_pi_control(id_ref, iq_ref, hw_inputs.id_actual, hw_inputs.iq_actual, &vd_cmd, &vq_cmd);
    printf("[Step 4] Controlled Volt : Vd_cmd = %.2f V, Vq_cmd = %.2f V\n", vd_cmd, vq_cmd);

    // Step 5: 역 변환 및 PWM 출력 매핑
    PWMOutputs pwm = step5_inverse_dq_and_pwm(vd_cmd, vq_cmd, hw_inputs.motor_angle);
    printf("[Step 5] Inverter PWM    : U상 = %.2f%%, V상 = %.2f%%, W상 = %.2f%%\n\n", 
           pwm.duty_A * 100.0f, pwm.duty_B * 100.0f, pwm.duty_C * 100.0f);
}

// 시스템 작동 테스트 메인 함수
int main() {
    // 임의의 정상 주행 하드웨어 입력 상황 가정
    VehicleHardwareInputs normal_drive = {
        .adc_sensor1 = 2250,   // 대략 반쯤 밟음 (2.5V 수준)
        .adc_sensor2 = 2250,   // 정상 상황 (두 센서 일치)
        .vehicle_speed = 50.0f, // 50 km/h 주행 중
        .id_actual = -10.5f,   // 현재 센서가 측정한 실제 모터 내부 전류 상태
        .iq_actual = 95.0f,
        .motor_angle = 0.785f  // 현재 모터 회전 각도 (45도 상황)
    };

    // 1 사이클 구동
    motor_control_pipeline_tick(normal_drive);

    // 임의의 센서 고장(페일 세이프 작동) 상황 가정
    VehicleHardwareInputs fault_drive = {
        .adc_sensor1 = 3500,   // 1번 센서는 가속 신호 전송
        .adc_sensor2 = 600,    // 2번 센서는 단선 에러 상태
        .vehicle_speed = 50.0f,
        .id_actual = 0.0f,
        .iq_actual = 0.0f,
        .motor_angle = 0.0f
    };

    // 고장 상황 구동 테스트
    printf("--- 센서 에러 발생 시나리오 ---\n");
    motor_control_pipeline_tick(fault_drive);

    return 0;
}