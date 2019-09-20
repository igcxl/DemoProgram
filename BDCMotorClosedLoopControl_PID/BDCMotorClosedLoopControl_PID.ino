/*
Name:    BDCMotorClosedLoopControl_Encoder.ino 有刷直流电机闭环控制编码器测速部分
原理：
Created:  09/04/2019 8:39:28 PM
Author: igcxl
*/

//#include <SSD1306.h>                 //OLED显示器库文件
#include <FlexiTimer2.h>        //定时中断


//******************PWM引脚和电机驱动引脚***************************//
const int AIN1 = 5;//A电机控制PWM波
const int AIN2 = 6;//A电机控制PWM波
const int BIN1 = 8;//B电机控制PWM波
const int BIN2 = 7;//B电机控制PWM波
const int CIN1 = 11;//C电机控制PWM波
const int CIN2 = 12;//C电机控制PWM波
const int DIN1 = 44;//D电机控制PWM波
const int DIN2 = 46;//D电机控制PWM波

//******************电机启动初始值  作者：igcxl**********************//
int motorDeadZone = 30;//高频时电机启动初始值高约为130，低频时电机启动初始值低约为30,和电池电压、电机特性、PWM频率等有关，需要单独测试
int PWMConstrain = 255-motorDeadZone;//电机PWM限幅值

//******************定时中断有关参数  作者：igcxl**********************//
#define TimerPeriod  10 //定时中断的周期10ms
int EncoderConstrain = 100;//定时中断内周期最高转速对应编码器计数值，用于滤波

//******************编码器引脚***************************//
#define ENCODER_A 2//A路电机编码器引脚AA，外部中断，中断号0
#define ENCODER_B 3//B路电机编码器引脚BA，外部中断，中断号1
#define ENCODER_C 18//C路电机编码器引脚CA，外部中断，中断号5
#define ENCODER_D 19//D路电机编码器引脚DA，外部中断，中断号4
#define DIRECTION_A 51//A路电机编码器引脚AB
#define DIRECTION_B 53//B路电机编码器引脚BB
#define DIRECTION_C 52//C路电机编码器引脚CB
#define DIRECTION_D 50//D路电机编码器引脚DB

//**********************全局变量***********************//
volatile long Velocity_1, Velocity_2, Velocity_3 , Velocity_4 ;   //编码器数据
long Velocity_A, Velocity_B, Velocity_C, Velocity_D ;//各轮速度
int iConstrain;
float Velocity_KP =3.0, Velocity_KI =0.2;//PI参数
float Target_A, Target_B,Target_C, Target_D;   //速度控制器倍率参数，目标参数
int Motora, Motorb,Motorc, Motord;//变量

/**************************************************************************
函数功能：赋值给PWM寄存器 ，重载函数，适用于4轮麦轮车
入口参数：PWM
**************************************************************************/
void Set_PWM(int motora, int motorb, int motorc,int motord) { 
  if (motora > 0)        analogWrite(AIN2, motora+motorDeadZone), analogWrite(AIN1, 0); //赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if(motora == 0)   analogWrite(AIN2, 0), analogWrite(AIN1,0);   
  else if (motora < 0)   analogWrite(AIN1, -motora+motorDeadZone), analogWrite(AIN2, 0);//高频时电机启动初始值高约为130，低频时电机启动初始值低约为30

  if (motorb > 0)        analogWrite(BIN2,motorb+motorDeadZone),analogWrite(BIN1, 0); //赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if(motorb == 0)   analogWrite(BIN2, 0),analogWrite(BIN1, 0);
  else if (motorb < 0)   analogWrite(BIN1,-motorb+motorDeadZone),analogWrite(BIN2, 0);//高频时电机启动初始值高约为130，低频时电机启动初始值低约为30

  if (motorc > 0)        analogWrite(CIN1,motorc+motorDeadZone),analogWrite(CIN2,0); //赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if(motorc == 0)   analogWrite(CIN2, 0), analogWrite(CIN1, 0);
  else if (motorc < 0)   analogWrite(CIN2, -motorc+motorDeadZone), analogWrite(CIN1, 0);//高频时电机启动初始值高约为130，低频时电机启动初始值低约为30

  if (motord > 0)        analogWrite(DIN1,motord+motorDeadZone),analogWrite(DIN2, 0); //赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if(motord == 0)   analogWrite(DIN1, 0), analogWrite(DIN2, 0);
  else if (motord < 0)   analogWrite(DIN2, -motord+motorDeadZone), analogWrite(DIN1, 0);//高频时电机启动初始值高约为130，低频时电机启动初始值低约为30
}
/**************************************************************************
函数功能：赋值给PWM寄存器 ，重载函数，适用于2或4定向轮车
入口参数：PWM
**************************************************************************/
void Set_PWM(int motorLeft, int motorRight){
  //motorLeft -->  motorb 和 motora
  //motorRight -->  motorc 和 motord
  
//  Serial.println(motorLeft);
//  Serial.println(motorRight);
  if (motorLeft > 0)        analogWrite(AIN2, motorLeft+motorDeadZone), analogWrite(AIN1, 0); //赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if(motorLeft == 0)   analogWrite(AIN2, 0), analogWrite(AIN1, 0);   
  else if (motorLeft < 0)   analogWrite(AIN1, -motorLeft+motorDeadZone), analogWrite(AIN2, 0);//高频时电机启动初始值高约为130，低频时电机启动初始值低约为30

  if (motorLeft > 0)        analogWrite(BIN2,motorLeft+motorDeadZone),analogWrite(BIN1, 0); //赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if(motorLeft == 0)   analogWrite(BIN2, 0),analogWrite(BIN1, 0);
  else if (motorLeft < 0)   analogWrite(BIN1,-motorLeft+motorDeadZone),analogWrite(BIN2, 0);//高频时电机启动初始值高约为130，低频时电机启动初始值低约为30

  if (motorRight > 0)        analogWrite(CIN1,motorRight+motorDeadZone),analogWrite(CIN2,0); //赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if(motorRight == 0)   analogWrite(CIN2, 0), analogWrite(CIN1, 0);
  else if (motorRight < 0)   analogWrite(CIN2, -motorRight+motorDeadZone), analogWrite(CIN1, 0);//高频时电机启动初始值高约为130，低频时电机启动初始值低约为30

  if (motorRight > 0)        analogWrite(DIN1,motorRight+motorDeadZone),analogWrite(DIN2, 0); //赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if(motorRight == 0)   analogWrite(DIN1, 0), analogWrite(DIN2, 0);
  else if (motorRight < 0)   analogWrite(DIN2, -motorRight+motorDeadZone), analogWrite(DIN1, 0);//高频时电机启动初始值高约为130，低频时电机启动初始值低约为30

}

/**************************************************************************
函数功能：增量PI控制器
入口参数：目标速度，编码器测量值
返回  值：电机PWM
根据增量式离散PID公式
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差
e(k-1)代表上一次的偏差  以此类推
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/

int Incremental_PI_A (int Target,int Encoder)
{  float Error;
   static float PWM,LastError;
   Error =Target-Encoder;                                  //计算偏差 Error = 目标值 - 实测值
   PWM+=Velocity_KP*( Error-LastError)+Velocity_KI*Error;   //增量式PI控制器
   PWM = constrain(PWM, -PWMConstrain, PWMConstrain) ;//限幅
   LastError=Error;                                       //保存上一次偏差 
   return PWM;                                           //增量输出
}
int Incremental_PI_B (int Target,int Encoder)
{  float Error; 
   static float PWM,LastError;
   Error=Target-Encoder;                                   //计算偏差
   PWM+=Velocity_KP*(Error-LastError)+Velocity_KI*Error;   //增量式PI控制器
   PWM = constrain(PWM, -PWMConstrain, PWMConstrain) ;//限幅
   LastError=Error;                                        //保存上一次偏差
   return PWM;                                            //增量输出
}
/**************************************************************************/
int Incremental_PI_C (int Target,int Encoder)
{  float Error; 
   static float PWM,LastError;
   Error=Target-Encoder;                                  //计算偏差
   PWM+=Velocity_KP*(Error-LastError)+Velocity_KI*Error;   //增量式PI控制器
   PWM = constrain(PWM, -PWMConstrain, PWMConstrain) ;//限幅
   LastError=Error;                                       //保存上一次偏差
   return PWM;                                           //增量输出
}
/**************************************************************************/
int Incremental_PI_D (int Target,int Encoder)
{  float Error; 
   static float PWM,LastError;
   Error=Target-Encoder;                                  //计算偏差
   PWM+=Velocity_KP*(Error-LastError)+Velocity_KI*Error;   //增量式PI控制器
   PWM = constrain(PWM, -PWMConstrain, PWMConstrain) ;//限幅
   LastError=Error;                                       //保存上一次偏差
   return PWM;                                           //增量输出
}
/**************************************************************************

/*****函数功能：外部中断读取编码器数据，具有二倍频功能 注意外部中断是跳变沿触发********/

void READ_ENCODER_A() {
  if (digitalRead(ENCODER_A) == LOW) {     //如果是下降沿触发的中断
    if (digitalRead(DIRECTION_A) == LOW)      Velocity_1--;  //根据另外一相电平判定方向
    else      Velocity_1++;
  }
  else {     //如果是上升沿触发的中断
    if (digitalRead(DIRECTION_A) == LOW)      Velocity_1++; //根据另外一相电平判定方向
    else     Velocity_1--;
  }
}

/*****函数功能：外部中断读取编码器数据，具有二倍频功能 注意外部中断是跳变沿触发********/
void READ_ENCODER_B() {
  if (digitalRead(ENCODER_B) == LOW) { //如果是下降沿触发的中断
    if (digitalRead(DIRECTION_B) == LOW)      Velocity_2++;//根据另外一相电平判定方向
    else      Velocity_2--;
  }
  else {   //如果是上升沿触发的中断
    if (digitalRead(DIRECTION_B) == LOW)      Velocity_2--; //根据另外一相电平判定方向
    else     Velocity_2++;
  }
}
/*****函数功能：外部中断读取编码器数据，具有二倍频功能 注意外部中断是跳变沿触发********/
void READ_ENCODER_C() {
  if (digitalRead(ENCODER_C) == LOW) { //如果是下降沿触发的中断
    if (digitalRead(DIRECTION_C) == LOW)      Velocity_3++;//根据另外一相电平判定方向
    else      Velocity_3--;
  }
  else {   //如果是上升沿触发的中断
    if (digitalRead(DIRECTION_C) == LOW)      Velocity_3--; //根据另外一相电平判定方向
    else     Velocity_3++;
  }
}
/*****函数功能：外部中断读取编码器数据，具有二倍频功能 注意外部中断是跳变沿触发********/
void READ_ENCODER_D() {
  if (digitalRead(ENCODER_D) == LOW) { //如果是下降沿触发的中断
    if (digitalRead(DIRECTION_D) == LOW)      Velocity_4++;//根据另外一相电平判定方向
    else      Velocity_4--;
  }
  else {   //如果是上升沿触发的中断
    if (digitalRead(DIRECTION_D) == LOW)      Velocity_4--; //根据另外一相电平判定方向
    else     Velocity_4++;
  }
}
/*********函数功能：10ms控制函数 核心代码 作者：igcxl*******/
void control() {
  
  //static int print_Count;
  sei();//全局中断开启
  //获取电机A速度
  if (abs(Velocity_1)<EncoderConstrain)
  {
  Velocity_A = -Velocity_1;    Velocity_1 = 0; //读取编码器数据并根据实际接线做调整、然后清零，这就是通过M法测速（单位时间内的脉冲数）得到速度。
  }
  else
  {
    Velocity_1 = 0;//超出范围的延用上次的速度值、然后清零。
    }
  //获取电机B速度
  if (abs(Velocity_2)<EncoderConstrain)
  {
  Velocity_B =  Velocity_2;    Velocity_2 = 0; //读取编码器数据并根据实际接线做调整、然后清零，这就是通过M法测速（单位时间内的脉冲数）得到速度。
  }
  else
  {
   Velocity_2 = 0; //超出范围的延用上次的速度值、然后清零。
    }
  //获取电机C速度
  if (abs(Velocity_3)<EncoderConstrain)
  {
 Velocity_C = -Velocity_3;    Velocity_3 = 0; //读取编码器数据并根据实际接线做调整、然后清零，这就是通过M法测速（单位时间内的脉冲数）得到速度。
  }
  else
  {
   Velocity_3 = 0; //超出范围的延用上次的速度值、然后清零。
    }
   //获取电机D速度
  if (abs(Velocity_4)<EncoderConstrain)
  {
 Velocity_D = -Velocity_4;    Velocity_4 = 0;//读取编码器数据并根据实际接线做调整、然后清零，这就是通过M法测速（单位时间内的脉冲数）得到速度。
  }
  else
  {
   Velocity_4 = 0; //超出范围的延用上次的速度值、然后清零。
    }
  
  Motora = Incremental_PI_A(0,Velocity_A); //===速度PI控制器
  Motorb = Incremental_PI_B(5,Velocity_B); //===速度PI控制器
  Motorc = Incremental_PI_C(0,Velocity_C); //===速度PI控制器
  Motord = Incremental_PI_D(0,Velocity_D); //===速度PI控制器
  Set_PWM(Motora, Motorb,Motorc,Motord ); //如果不存在异常，使能电机 



}
void setup() {
  // put your setup code here, to run once:
//  int fff = 1;
//  TCCR1B =(TCCR1B & 0xF8) | fff;//调整计数器分频，频率调高至31.374KHZ
//  TCCR3B =(TCCR3B & 0xF8) | fff;//调整计数器分频，频率调高至31.374KHZ
//  TCCR4B =(TCCR4B & 0xF8) | fff;//调整计数器分频，频率调高至31.374KHZ 
//  TCCR5B =(TCCR5B & 0xF8) | fff;//调整计数器分频，频率调高至31.374KHZ
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
  Serial.begin(9600);  
  delay(300);   //延时等待初始化完成
  FlexiTimer2::set(TimerPeriod, control); //10毫秒定时中断函数
  FlexiTimer2::start ();      //中断使能 
  attachInterrupt(0, READ_ENCODER_A, CHANGE);           //开启外部中断 编码器接口A
  attachInterrupt(1, READ_ENCODER_B, CHANGE);           //开启外部中断 编码器接口B
  attachInterrupt(5, READ_ENCODER_C, CHANGE);           //开启外部中断 编码器接口C 
  attachInterrupt(4, READ_ENCODER_D, CHANGE);           //开启外部中断 编码器接口D
  delay(100); 
}

void loop() {
  // put your main code here, to run repeatedly:
//Target_A=0;
// Target_B=0;
// Target_C=0;
// Target_D=0;
//delay(5000);
//Target_A=30;
// Target_B=30;
// Target_C=30;
// Target_D=30;
//delay(5000);
//Set_PWM(200,0,200,0 );
//delay(5000);
//Set_PWM(0,155,0,155 );
//delay(5000);
//Set_PWM(-50,-50,-50,-50 );
//delay(5000);
//Set_PWM(125,125); //Move Forward at full speed
//delay(5000);
//Set_PWM(-10,-20); //Set_PWM(int motorLeft, int motorRight)
//delay(5000);
// Target_A=50;
// Target_B=50;
// Target_C=50;
// Target_D=50;
//delay(5000);
//电机死区测试
//for(int i = 0; i< 255; i++){
//iConstrain = constrain(i, 0, PWMConstrain); //将i限制在0-（255-motorDeadZone）区间，大于返回（255-motorDeadZone） ，小于返回0
//Set_PWM(iConstrain,iConstrain,iConstrain,iConstrain);
//Serial.println(i);
////Serial.println(iConstrain);
//delay(1000);
//}

//  Serial.println(millis());//显示 
//  Serial.println(Motora);//显示 
////  Serial.println(Motorb);//显示
////  Serial.println(Motorc);//显示
////  Serial.println(Motord);//显示
//  Serial.println(Velocity_A);//显示 
////  Serial.println(Velocity_B);//显示
////  Serial.println(Velocity_C);//显示
////  Serial.println(Velocity_D);//显示
//  //Serial.println(iConstrain);
//  delay(200);

 //Serial.println(millis());//显示 
  //Serial.print(Motora);//显示 
  //Serial.print(",");
Serial.print(Motorb);//显示
Serial.print(",");
//Serial.print(Motorc);//显示
//Serial.print(",");
//Serial.print(Motord);//显示
//Serial.print(",");
//Serial.print(Velocity_A);//显示
//Serial.print(",");
Serial.println(Velocity_B);//显示
//Serial.print(",");
//Serial.print(Velocity_C);//显示
//Serial.print(",");
//Serial.println(Velocity_D);//显示
//Serial.print(",");
delay(50);
}
