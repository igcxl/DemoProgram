/**
 * @file SoftwareSerial_TTS.ino 软串口和TTS通讯测试程序SG
 * @author igcxl (igcxl@qq.com)
 * @brief  软串口和TTS通讯测试程序
 * @note 硬串口接收到数据时软串口发送GBK编码“你好”，软串口接收到数据时发送到硬串口
 * Not all pins on the Mega and Mega 2560 support change interrupts,
 *so only the following can be used for RX:
 *10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69
 * @version 0.5
 * @date 2019-09-24
 * @copyright Copyright © igcxl.com 2019
 * 
 */

#include <SoftwareSerial.h>
SoftwareSerial mySerial(A14, A15); // (RX, TX)(A8, A9)

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
//  while (!Serial)
//  {
//    ; // wait for serial port to connect. Needed for native USB port only
//  }
  Serial.println("Goodnight moon!");
  delay(100);

  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
     mySerial.write(0xB3); //初
    mySerial.write(0xF5);
    mySerial.write(0xCA); //始
    mySerial.write(0xBC);
    mySerial.write(0xBB); //化
    mySerial.write(0xAF);
 mySerial.println("OK");
}

void loop()
{
  if (mySerial.available())
  {
    Serial.write(mySerial.read());
  }
  if (Serial.available() > 0)
  {
    Serial.println(Serial.available());
    mySerial.write(0xC4); //你
    mySerial.write(0xE3);
    mySerial.write(0xBA); //好
    mySerial.write(0xC3);
    delay(30);
    while (Serial.read() >= 0)
    {
    } //清空串口缓存
  }
}
