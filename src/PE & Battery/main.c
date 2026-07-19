#include "shared_types.h"
#include "vehicle_strategy.h"
#include "electric_motor_control.h"

#define DT 0.0001f          // 10kHz 제어 주기 (0.1ms)
#define PI 3.14159265f

// 제어용 글로벌 변수 및 게인 설정
static float error_d_sum = 0.0f;
static float error_q_sum = 0.0f;
const float Kp_d = 1.5f, Ki_d = 200.0f; // d축 PI 게인
const float Kp_q = 1.5f, Ki_q = 200.0f; // q축 PI 게인

// ====================================================================
// 🚀 실시간 모터 제어 파이프라인 스케줄러 (10kHz 루프)
// ====================================================================
void motor_control_pipeline_tick(VehicleHardwareInputs hw_inputs, BatteryInformations battery) {
    printf("=== [Control Loop Tick] ===\n");

    // Step 1: 페달 입력 센싱
    float pedal_pct = pedal_sensing(hw_inputs.adc_sensor1, hw_inputs.adc_sensor2);
    printf("[Step 1] Pedal Input     : %.1f %%\n", pedal_pct);

    // Step 2: 토크 지령 계산
    float torque_cmd = calculate_torque(pedal_pct, hw_inputs.vehicle_speed, battery);
    printf("[Step 2] Target Torque   : %.2f Nm (Speed: %.1f km/h)\n", torque_cmd, hw_inputs.vehicle_speed);

    // Step 3: 전류 지령 계산
    float id_ref = 0.0f, iq_ref = 0.0f;
    mtpa_current_mapping(torque_cmd, &id_ref, &iq_ref);
    printf("[Step 3] Target Current  : Id_ref = %.2f A, Iq_ref = %.2f A\n", id_ref, iq_ref);

    // Step 4: 전압 제어 (PI)
    float vd_cmd = 0.0f, vq_cmd = 0.0f;
    current_pi_control(id_ref, iq_ref, hw_inputs.id_actual, hw_inputs.iq_actual, &vd_cmd, &vq_cmd);
    printf("[Step 4] Controlled Volt : Vd_cmd = %.2f V, Vq_cmd = %.2f V\n", vd_cmd, vq_cmd);

    // Step 5: 역 변환 및 PWM 출력 매핑
    PWMOutputs pwm = inverse_dq_and_pwm(vd_cmd, vq_cmd, hw_inputs.motor_angle);
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

    BatteryInformations battery = {
        .soc = 30.0f,
        .temp = 30.0f
    };

    // 1 사이클 구동
    motor_control_pipeline_tick(normal_drive, battery);

    // 임의의 센서 고장(페일 세이프 작동) 상황 가정
    VehicleHardwareInputs fault_drive = {
        .adc_sensor1 = 3500,   // 1번 센서는 가속 신호 전송
        .adc_sensor2 = 600,    // 2번 센서는 단선 에러 상태
        .vehicle_speed = 50.0f,
        .id_actual = 0.0f,
        .iq_actual = 0.0f,
        .motor_angle = 0.0f
    };

    battery.soc = 5.0f;
    battery.temp = 60.0f;

    // 고장 상황 구동 테스트
    printf("--- 센서 에러 발생 시나리오 ---\n");
    motor_control_pipeline_tick(fault_drive, battery);

    return 0;
}