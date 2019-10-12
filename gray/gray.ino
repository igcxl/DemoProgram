#define BLACK HIGH // 高电平为黑色
#define WHITE LOW // 低电平为白色
#define RIGHT_GRAY 13 //右侧灰度传感器引脚
#define LEFT_GRAY 10// 左侧灰度传感器引脚
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(RIGHT_GRAY, INPUT);
  pinMode(LEFT_GRAY, INPUT);
   Serial.begin(115200);           // 初始化串口通信
}

// the loop function runs over and over again forever
void loop() {

Serial.print("RIGHT:");
Serial.println(digitalRead(RIGHT_GRAY));
Serial.print("LEFT:");
Serial.println(digitalRead(LEFT_GRAY));
delay(500);
}
