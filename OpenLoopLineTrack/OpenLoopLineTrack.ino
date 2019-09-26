
/**
 * @file LineTrack.ino 速度开环状态机循线示例程序
 * @author igcxl (igcxl@qq.com)
 * @brief 电机开环状态机示例程序
 * @note 请根据实际情况添加状态和修改电机速度
 * @version 0.3
 * @date 2019-09-26
 * @copyright Copyright © igcxl.com 2019
 * 
 */

#include <FlexiTimer2.h> ///<定时中断
#include "uart.h"
#include <SoftwareSerial.h>
SoftwareSerial mySerial(A14, A15); // (RX, TX)
//******************PWM引脚和电机驱动引脚******************//
const int AIN1 = 5;  ///<A路电机控制PWM波引脚1
const int AIN2 = 6;  ///<A路电机控制PWM波引脚2
const int BIN1 = 8;  ///<B路电机控制PWM波引脚1
const int BIN2 = 7;  ///<B路电机控制PWM波引脚2
const int CIN1 = 11; ///<C路电机控制PWM波引脚1
const int CIN2 = 12; ///<C路电机控制PWM波引脚2
const int DIN1 = 44; ///<D路电机控制PWM波引脚1
const int DIN2 = 46; ///<D路电机控制PWM波引脚2

//******************电机启动初始值**********************//
int motorDeadZone = 30; ///<高频时电机启动初始值高约为130，低频时电机启动初始值低约为30,和电池电压、电机特性、PWM频率等有关，需要单独测试

//******************定时中断有关参数 **********************//
#define TIMER_PERIOD 10            ///<定时中断周期10ms
int Encoder_Pulses_Constrain = 75; ///<定时中断周期内最高转速对应编码器计数值，用于滤波,电机转速500rpm,每转输出390个脉冲，12V额定电压空载10ms二倍频的计数是500/60/1000*10*390*2=65个

//******************编码器引脚***************************//
#define ENCODER_A 2    ///<A路电机编码器引脚AA，外部中断，中断号0
#define ENCODER_B 3    ///<B路电机编码器引脚BA，外部中断，中断号1
#define ENCODER_C 18   ///<C路电机编码器引脚CA，外部中断，中断号5
#define ENCODER_D 19   ///<D路电机编码器引脚DA，外部中断，中断号4
#define DIRECTION_A 51 ///<A路电机编码器引脚AB
#define DIRECTION_B 53 ///<B路电机编码器引脚BB
#define DIRECTION_C 52 ///<C路电机编码器引脚CB
#define DIRECTION_D 50 ///<D路电机编码器引脚DB

//******************电压检测引脚***************************//
#define VOLTAGE A0 ///<电压检测引脚A0

//******************串口调试输出开关***************************//
//#define DEBUG_INFO ///<调试输出开关，注释后关闭调试信息串口输出
//******************TTS调试输出开关***************************//
#define DEBUG_TTS ///<调试输出开关，注释后关闭调试信息串口输出

//******************循线传感器***************************//
unsigned int Temp_Data[2] = {0}; //循线传感器数据缓存区
//**********************全局变量***********************//
//int MotorR, MotorL;    ///<电机变量
int g_Battery_Voltage; ///<电池电压采样变量,实际电压*100

/**
 * @brief 赋值给2路PWM寄存器
 * @note 重载函数，适用于2或4定向轮车
 * @param motorLeft  左路电机控制PWM-->  motorb 和 motora(取值范围：-255至255，负值为向后转，正值为向前转)
 * @param motorRight 右路电机控制PWM-->  motorc 和 motord(取值范围：-255至255，负值为向后转，正值为向前转)
 */
void Set_PWM(int motorLeft, int motorRight)
{
    if (motorLeft > 0)
        analogWrite(AIN2, motorLeft + motorDeadZone), analogWrite(AIN1, 0); ///<赋值给PWM寄存器根据电机响应速度与机械误差微调,
    else if (motorLeft == 0)
        analogWrite(AIN2, 0), analogWrite(AIN1, 0);
    else if (motorLeft < 0)
        analogWrite(AIN1, -motorLeft + motorDeadZone), analogWrite(AIN2, 0); ///<高频时电机启动初始值高约为130，低频时电机启动初始值低约为30

    if (motorLeft > 0)
        analogWrite(BIN2, motorLeft + motorDeadZone), analogWrite(BIN1, 0); ///<赋值给PWM寄存器根据电机响应速度与机械误差微调,
    else if (motorLeft == 0)
        analogWrite(BIN2, 0), analogWrite(BIN1, 0);
    else if (motorLeft < 0)
        analogWrite(BIN1, -motorLeft + motorDeadZone), analogWrite(BIN2, 0); ///<高频时电机启动初始值高约为130，低频时电机启动初始值低约为30

    if (motorRight > 0)
        analogWrite(CIN1, motorRight + motorDeadZone), analogWrite(CIN2, 0); ///<赋值给PWM寄存器根据电机响应速度与机械误差微调,
    else if (motorRight == 0)
        analogWrite(CIN2, 0), analogWrite(CIN1, 0);
    else if (motorRight < 0)
        analogWrite(CIN2, -motorRight + motorDeadZone), analogWrite(CIN1, 0); ///<高频时电机启动初始值高约为130，低频时电机启动初始值低约为30

    if (motorRight > 0)
        analogWrite(DIN1, motorRight + motorDeadZone), analogWrite(DIN2, 0); ///<赋值给PWM寄存器根据电机响应速度与机械误差微调,
    else if (motorRight == 0)
        analogWrite(DIN1, 0), analogWrite(DIN2, 0);
    else if (motorRight < 0)
        analogWrite(DIN2, -motorRight + motorDeadZone), analogWrite(DIN1, 0); ///<高频时电机启动初始值高约为130，低频时电机启动初始值低约为30
}

/**
 * @brief 异常关闭电机
 * 
 * @return unsigned char 返回断电状态 1=断电 0=通电 值：1：异常  0：正常
 */
unsigned char Turn_Off()
{
    byte temp;
    if (g_Battery_Voltage < 1100)
    {
        temp = 1;
        digitalWrite(AIN1, LOW); ///<电机A驱动的电平控制
        digitalWrite(AIN2, LOW); ///<电机A驱动的电平控制
        digitalWrite(BIN1, LOW); //电机B驱动的电平控制
        digitalWrite(BIN2, LOW); //电机B驱动的电平控制
        digitalWrite(CIN1, LOW); //电机C驱动的电平控制
        digitalWrite(CIN2, LOW); //电机C驱动的电平控制
        digitalWrite(DIN1, LOW); //电机D驱动的电平控制
        digitalWrite(DIN2, LOW); //电机D驱动的电平控制
    }
    else
        temp = 0;
    return temp;
}

/**
 * @brief 定时中断执行函数
 * @note 核心函数，采集电压，获取速度，速度PI控制
 * 
 */
void Control()
{
    int voltageTemp;             ///<电压采样临时变量
    static float s_Voltage_Sum;  ///<电压采样周期累加值
    static byte s_Voltage_Count; ///<电压采样周期
    sei();                       ///<全局中断开启

    voltageTemp = analogRead(VOLTAGE); ///<采集电池电压
    s_Voltage_Count++;                 ///<平均值计数器
    s_Voltage_Sum += voltageTemp;      ///<多次采样累积
    if (s_Voltage_Count == 100)
    {
        g_Battery_Voltage = s_Voltage_Sum * 0.05371; ///<求电压平均值
        s_Voltage_Sum = 0;
        s_Voltage_Count = 0;
    }
    Turn_Off();
}

void track_line()
{
    unsigned int Temp[2] = {0}; //数据缓存区
    Read_Data(Temp);
#ifdef DEBUG_INFO
    Serial.println(Temp[0], BIN);
#endif
    switch (Temp[0])
    {                      //765 4321
    case 0x00:             //000 0000 全白
        Set_PWM(-40, -40); //后退
        break;
    case 0x7F:           //111 1111 全黑
        Set_PWM(40, 40); //前进
        break;
    case 0x1C:           //0011100 居中
        Set_PWM(40, 40); //前进
        break;
    case 0x70:            //1110000 大偏左
        Set_PWM(60, -60); //大右转
        break;
    case 0x38:            //0111000 小偏左
        Set_PWM(60, -20); //小右转
        break;
    case 0x0E:            //0001110 小偏右
        Set_PWM(-20, 60); //小左转
        break;
    case 0x07:            //0000111 大偏右
        Set_PWM(-60, 60); //大左转
        break;
    case 0x7C: //1111100 T路口
               //Set_PWM(0, 0); //停
#ifdef DEBUG_TTS
        mySerial.println("STOP");
#endif
        //delay(5000);
        break;
    case 0x0C:            //0001100 小偏右
        Set_PWM(-20, 60); //小左转
        break;
    case 0x18:            //0011000 小偏左
        Set_PWM(60, -20); //小右转
        break;
    case 0x40:            // 1000000 大偏左
        Set_PWM(60, -60); //大右转
        break;
    case 0x60:            // 1100000 大偏左
        Set_PWM(60, -60); //大右转
        break;
    case 0x01:             // 1000000 大偏右
        Set_PWM(-100, 60); //大左转
        break;
    case 0x03:            // 1100000 大偏右
        Set_PWM(-60, 60); //大左转
        break;
    default:
        Set_PWM(40, 40);
        break;
    }
    delay(10);
}

/**
 * @brief 初始化函数
 * 
 */
void setup()
{

    pinMode(AIN1, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(CIN1, OUTPUT);
    pinMode(DIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(CIN2, OUTPUT);
    pinMode(DIN2, OUTPUT);
    pinMode(ENCODER_A, INPUT);
    pinMode(ENCODER_B, INPUT);
    pinMode(ENCODER_C, INPUT);
    pinMode(ENCODER_D, INPUT);
    pinMode(DIRECTION_A, INPUT);
    pinMode(DIRECTION_B, INPUT);
    pinMode(DIRECTION_C, INPUT);
    pinMode(DIRECTION_D, INPUT);
    Serial.begin(9600);                      ///<调试用串口
    uart_Init();                             ///<循线传感器串口
    delay(300);                              ///<延时等待初始化完成
    FlexiTimer2::set(TIMER_PERIOD, Control); ///<10毫秒定时中断函数
    FlexiTimer2::start();                    ///<中断使能
#ifdef DEBUG_TTS
    mySerial.begin(9600);
    delay(50);            //等待TTS上电启动
    mySerial.write(0xB3); //初
    mySerial.write(0xF5);
    mySerial.write(0xCA); //始
    mySerial.write(0xBC);
    mySerial.write(0xBB); //化
    mySerial.write(0xAF);
    mySerial.println("OK");
#endif
}

/**
 * @brief 主函数
 * 
 */
void loop()
{
    track_line();
#ifdef DEBUG_INFO
    Read_Data(Temp_Data);
    Serial.println(Temp_Data[0], BIN);
    delay(1000);
#endif
}
