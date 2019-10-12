/**
 * @file SR04Example.ino 基于 pulseIn函数的超声波测距例程
 * @author igcxl (igcxl@qq.com)
 * @brief 基于 pulseIn函数的超声波测距例程
 * @version 0.2
 * @date 2019-09-21
 * 
 * @copyright Copyright © igcxl.com 2019 All right reserved.
 * 
 */

#define R_moto1 5 //右侧电机控制端1
#define R_moto2 7 //右侧电机控制端2 
#define R_moto3 11 //右侧电机控制端1
#define R_moto4 12 //右侧电机控制端2 
#define L_moto1 6 //左侧电机控制端1
#define L_moto2 8 //左侧电机控制端2 
#define L_moto3 3 //左侧电机控制端1
#define L_moto4 4 //左侧电机控制端2 
//控制端1-控制端2=正值，正转；负值，反转；差值为转速。

const int TRIGGER_PIN = 2;
const int ECHO_PIN = 9;
float distance;
void setup()
{
    Serial.begin(115200);           // 初始化串口通信
    pinMode(TRIGGER_PIN, OUTPUT); // 初始化SR04的引脚
    pinMode(ECHO_PIN, INPUT);     // 要检测引脚上输入的脉冲宽度，需要先设置为输入状态
    Serial.println("Ultrasonic sensor:");
    
  pinMode( R_moto1 , OUTPUT);//定义数字量接口R_moto1为输出量，
  pinMode( R_moto2 , OUTPUT);//定义数字量接口R_moto2为输出量，
  pinMode( R_moto3 , OUTPUT);//定义数字量接口R_moto1为输出量，
  pinMode( R_moto4 , OUTPUT);//定义数字量接口R_moto2为输出量，
  pinMode( L_moto1 , OUTPUT);//定义数字量接口L_moto1为输出量，
  pinMode( L_moto2 , OUTPUT);//定义数字量接口L_moto2为输出量，
  pinMode( L_moto3 , OUTPUT);//定义数字量接口L_moto1为输出量，
  pinMode( L_moto4 , OUTPUT);//定义数字量接口L_moto2为输出量，
}
void range()
{
      // 产生一个10us的高脉冲去触发TrigPin
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);
    // 检测脉冲宽度，并计算出距离
    distance = pulseIn(ECHO_PIN, HIGH) / 58.30;
  }
 void carTurnLeft(){
  
  
  
  }
 void carStop(){
  
      analogWrite(R_moto1,LOW);
    digitalWrite(R_moto2,LOW);   
    analogWrite(R_moto3,LOW);
    digitalWrite(R_moto4,LOW);
    analogWrite(L_moto1,LOW);
    digitalWrite(L_moto2,LOW);
    analogWrite(L_moto3,LOW);
    digitalWrite(L_moto4,LOW); 
  
  }
 void carGo(){
  
  
  
  }
void loop()
{
    range();
    
    if(distance>30){carGo();}
    
    else if(distance>10){carTurnLeft();}
    
    else {carStop();}
    
    Serial.print(distance);
    Serial.print("cm");
    Serial.println();
    delay(50);
}
