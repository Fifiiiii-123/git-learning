/*
MeArm舵机初始化调整程序 v1.0 (MeArm_Servo_Adj.ino)
by 太极创客 (2017-06-02)
http://www.taichi-maker.com

本程序用于太极创客《零基础入门学用Arduino教程 - MeArm篇》项目制作。
在组装MeArm机械臂零部件以前，需要先对舵机进行相应调整，否则机械臂
可能无法正常工作，严重的有可能对MeArm部件及舵机造成难以修复的损坏。
所以请您在组装MeArm以前，请使用本程序对舵机进行初始化调整后再开始
组装MeArm机械臂。

太极创客团队为MeArm机械臂安装和使用制作了视频教程和装配说明书。
您可通过太极创客网站免费下载/收看该教程所有内容。

=================================
  舵机控制线与Arduino连接说明
 -----------------------------
 Arduino Uno R3      MeArm舵机
  引脚编号          PWM控制线

    6   -------  钳子舵机(c)law                
    9   -------  前臂舵机(f)Arm
    10  -------  后臂舵机(r)Arm
    11  -------  底盘舵机(b)ase  
=================================    
如需要获得具体电路连接细节，请查阅太极创客制作的
《零基础入门学用Arduino教程 - MeArm篇》页面。

《零基础入门学用Arduino教程》面向对象是完全没有任何Arduino和编程基础的“小白”，
这套教程通过视频教学，实验演示以及示例程序讲解，为您讲述Arduino的硬件和程序开发
基础知识。

请留意：
本Arduino程序与MeArm官方公布的舵机调整程序有所不同。
太极创客根据我们的教程制作以及对MeArm使用的感受
对舵机组装中的舵机初始化调节进行了相应的调整。

MeArm官方网站网址：MeArm.io
This example code is in the public domain.
*/

#include <Servo.h> 
 
Servo base, rArm, fArm, claw ;  //建立4个舵机对象
 
void setup() 
{ 
  base.attach(9);     // base 伺服舵机连接引脚11 舵机代号'b'
  rArm.attach(8);     // rArm 伺服舵机连接引脚10 舵机代号'r'
  fArm.attach(7);      // fArm 伺服舵机连接引脚9  舵机代号'f'
  claw.attach(6);      // claw 伺服舵机连接引脚6  舵机代号'c'
  Serial.begin(9600);
} 
void loop() 
{ 
  base.write(90); // 将base（底盘）舵机设置为初始位置
  delay(100);
  rArm.write(90); // 将rArm（后臂）舵机设置为初始位置
  delay(100);
  fArm.write(90); // 将fArm（前臂）舵机设置为初始位置
  delay(100);
  claw.write(90); // 将claw（钳子）舵机设置为初始位置
  delay(3000); 
} 

