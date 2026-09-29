
#include "FOCDriver.h"
#include "main.h"
#include "math.h"
#include "../hardware/AS5600lib/AS5600.h"

//记得定义PWM引脚



//电角度求解
int pole_pairs = 7; //电机极对数，根据实际电机修改
int direction = 1; //旋转方向，1或-1
float Electrical_Angle( int pole_pairs)
{
    return Angle_Normalize((float)(direction * pole_pairs) * getAngle_Without_track()-Zero_Electric_Angle);
} 
//角度归一化
float Angle_Normalize (float angle)
{
    float a = fmod(angle, 2*PI);
    return (a >= 0) ? a : (a + 2*PI);
}

//写入占空比
void SetPWM(float Ua, float Ub, float Uc)
{
    //限制电压幅值
    Ua = _constrain(Ua, 0.0f, Voltage_Power_Supply);
    Ub = _constrain(Ub, 0.0f, Voltage_Power_Supply);
    Uc = _constrain(Uc, 0.0f, Voltage_Power_Supply);


    // 限制占空比在0到1之间
    dc_a = _constrain(Ua / Voltage_Power_Supply, 0.0f, 1.0f);
    dc_b = _constrain(Ub / Voltage_Power_Supply, 0.0f, 1.0f);
    dc_c = _constrain(Uc / Voltage_Power_Supply, 0.0f, 1.0f);

    //写入PWM占空比
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, (uint32_t)(dc_a * __HAL_TIM_GET_AUTORELOAD(&htim1)));
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, (uint32_t)(dc_b * __HAL_TIM_GET_AUTORELOAD(&htim1)));
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, (uint32_t)(dc_c * __HAL_TIM_GET_AUTORELOAD(&htim1)));

}

void SetphaseVoltage(float Uq, float Angle_el)
{

    Angle_el = Angle_Normalize(Angle_el + Zero_Electric_Angle);
    //clark变换
    Ualpha = -Uq * sinf(Angle_el);
    Ubeta = Uq * cosf(Angle_el);

    //park变换
    // 克拉克逆变换
    Ua = Ualpha + Voltage_Power_Supply/2;
    Ub = (sqrt(3)*Ubeta-Ualpha)/2 + Voltage_Power_Supply/2;
    Uc = (-Ualpha-sqrt(3)*Ubeta)/2 + Voltage_Power_Supply/2;

    SetPWM(Ua, Ub, Uc);
    
}

