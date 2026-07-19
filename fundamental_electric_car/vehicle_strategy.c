#include <stdint.h>
#include "shared_types.h"

// ====================================================================
// 1단계: 페달 입력 센싱 (각도 -> 디지털 숫자 -> 0 ~ 100%)
// ====================================================================
float pedal_sensing(uint16_t adc1, uint16_t adc2) {
    uint16_t adc_min = 500;   // 0.5V에 대응하는 디지털 값
    uint16_t adc_max = 4500;  // 4.5V에 대응하는 디지털 값
    
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
float calculate_torque(float pedal_percent, float speed, BatteryInformations battery) {
     const float MAX_TORQUE = 350.0f;
     const float MAX_REGEN_TORQUE = 150.0f;
    
    if (pedal_percent > 0.01f) {
        // 단순화된 룩업 테이블 대용 수식 (기본 토크 생성)
        float base_torque = (pedal_percent / 100.0f) * MAX_TORQUE;
        
        // 차량 속도가 너무 높으면 토크가 줄어들도록 유도
        float speed_factor = 1.0f;
        if (speed > 80.0f) {
            speed_factor = 1.0f - ((speed - 80.0f) / 120.0f); // 200km/h에서 0이 됨
            if (speed_factor < 0.0f) speed_factor = 0.0f;
        }

        // 배터리 온도와 잔량에 따른 감쇄 계수(Factor) 계산
        float battery_factor = 1.0f;

        // 배터리 온도가 너무 높으면 보호를 위해 출력 제한
        if (battery.temp > 60.0f) {
            battery_factor = 0.3f; // 평소의 30% 힘만 내도록 제한
        } else if (battery.temp > 50.0f) {
            // 선형적으로 힘을 줄임 (50도: 100% -> 60도: 30%)
            battery_factor = 1.0f - ((battery.temp - 50.0f) / 10.0f) * 0.7f;
        }

        // 배터리 잔량이 거의 없으면 차가 멈추는 것을 막기 위해 제한
        float soc_factor = 1.0f;
        if (battery.soc < 5.0f) {
            soc_factor = 0.2f; // 잔량 5% 미만이면 20% 힘만 냄
        } else if (battery.soc < 15.0f) {
            soc_factor = 0.2f + ((battery.soc - 5.0f) / 10.0f) * 0.8f;  // 잔량 15%부터 5%까지 서서히 힘을 줄임
        }

        // 두 배터리 제한 요인 중 더 작은 값을 최종 배터리 계수로 채택
        if (soc_factor < battery_factor) {
            battery_factor = soc_factor;
        }

        return base_torque * speed_factor * battery_factor;
    }

    // [안전 장치 1] 배터리가 가득 찼거나 차가 멈춰가면 회생 제동 금지 (관성 주행 모드)
    if (battery.soc >= 95.0f || speed < 5.0f) {
        return 0.0f; 
    }
    
    // [안전 장치 2] 배터리 고온 보호 (배터리가 너무 뜨거우면 회생제동 충전 금지)
    if (battery.temp > 55.0f) {
        return 0.0f;
    }

    // [회생 제동 실행] 페달을 떼면 최대 제동 토크의 20%로 엔진 브레이크 효과 유도
    float regen_torque = -1.0f * (MAX_REGEN_TORQUE * 0.2f); 

    return regen_torque; // 최종 제동 토크 반환 (음수)
    
}