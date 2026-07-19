#ifndef SHARED_TYPES_H  // 중복 포함 방지 (이 헤더파일이 여러번 읽히는 걸 막아줌)
#define SHARED_TYPES_H

#include <stdint.h>

// 가상의 모터 센서 및 하드웨어 인터페이스를 위한 구조체 설계도
typedef struct {
    uint16_t adc_sensor1;   // 가속 페달 센서 1
    uint16_t adc_sensor2;   // 가속 페달 센서 2
    float vehicle_speed;    // 현재 차량 속도 (km/h)
    float id_actual;        // 실제 d축 전류
    float iq_actual;        // 실제 q축 전류
    float motor_angle;      // 모터 전기적 각도
    float battery_soc;      // 배터리 잔량
} VehicleHardwareInputs;


// 파이프라인 출력 데이터 구조체
typedef struct {
    float duty_A;
    float duty_B;
    float duty_C;
} PWMOutputs;

// 배터리
typedef struct {
    float soc;
    float temp;
} BatteryInformations;


#endif // SHARED_TYPES_H