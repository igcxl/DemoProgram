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

const int TRIGGER_PIN = 10;
const int ECHO_PIN = 13;
float distance;
void setup()
{
    Serial.begin(9600);           // 初始化串口通信
    pinMode(TRIGGER_PIN, OUTPUT); // 初始化SR04的引脚
    pinMode(ECHO_PIN, INPUT);     // 要检测引脚上输入的脉冲宽度，需要先设置为输入状态
    Serial.println("Ultrasonic sensor:");
}
void loop()
{
    // 产生一个10us的高脉冲去触发TrigPin
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);
    // 检测脉冲宽度，并计算出距离
    distance = pulseIn(ECHO_PIN, HIGH) / 58.00;
    Serial.print(distance);
    Serial.print("cm");
    Serial.println();
    delay(1000);
}
