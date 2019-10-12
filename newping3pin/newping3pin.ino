/*
  作者：Ardui.Co
  效果：超声波测距模块，使用单线连接
  版本：1.0
  更新时间：2017年1月13日
*/
#include <NewPing.h>
#define PING_PIN  13  // 宏定义13号数字端口为触发和接收端口
#define MAX_DISTANCE 200 //宏定义模块的最大测量距离，规格标450cm，但实测一般为200cm
NewPing sonar(PING_PIN, PING_PIN, MAX_DISTANCE); //声明模块的参数，注意Trig和Echo都是同一个端口
void setup() {
 Serial.begin(115200); // 设置串口的波特率
}
void loop() {
 delay(500); // 每次测量的时间间隔，模块规格为40Hz，因此最小为25ms，但一般不少于30ms
 Serial.print("Ping: ");
 Serial.print(sonar.ping_cm()); // 调用库里面的 ping_cm() 方法，直接输出距离
 Serial.println("cm");
}
