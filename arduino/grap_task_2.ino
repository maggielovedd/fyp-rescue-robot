#include<ros.h>   //ros/ros.h
#include<geometry_msgs/Point.h>
#include <Servo.h>                // 声明调用Servo.h库
#define  SERVO_NUM  6             //舵机数量
Servo myservo[SERVO_NUM];         //创建六个舵机类

const  byte servo_pin[SERVO_NUM]={10, A2, A3, A0, A1, 7}; //宏定义舵机控制引脚
int pos_0=110;
int pos_1=180;
int pos_2=180;
int pos_3=160;
int del_t=60;
int a=0;

ros::NodeHandle  nh;  // nodehandler nh

void move_or_not(const geometry_msgs::Point &indicator){
 // if(indicator.x == 0){
  //  pass;
 // }
  if(indicator.x == 1){
    severalServoControl(); 
    delay(5000);
    a=1;
  }
  if(indicator.x ==2){
    putdownServoControl();
    delay(5000);
    a=0;
  }
}

ros::Subscriber<geometry_msgs::Point> sub("grab", move_or_not);

void putdownServoControl()
{
 
//int distinguish= myservo[1].read();

if (a==1) 
{
  for(pos_2=90; pos_2>30; pos_2-=1)
  {   
    myservo[2].write(pos_2);
    delay(del_t); 
  }  
  for(pos_3=80; pos_3<160; pos_3+=1)
  {   
    myservo[3].write(pos_3);
    delay(del_t); 
  } 

  for(pos_1=60; pos_1<180; pos_1+=1)
  {
    myservo[1].write(pos_1);
    pos_2=pos_1 *5/4-45;
    myservo[2].write(pos_2);
    delay(del_t); 
  }                 
}
if (a==0)
{
  myservo[0].write(90);
  delay(100);
  myservo[0].write(110);
}
}

void severalServoControl()
{
   
  for(pos_1=180; pos_1>60; pos_1-=1)
  {
    myservo[1].write(pos_1);
    pos_2=pos_1 *5/4-45;
    myservo[2].write(pos_2);
    delay(del_t); 
  }                 
  for(pos_3=160; pos_3>80; pos_3-=1)
  {   
    myservo[3].write(pos_3);
    int rd=myservo[3].read();
    Serial.println(rd);
    delay(del_t); 
  } 
    for(pos_2=30; pos_2<90; pos_2+=1)
  {   
    myservo[2].write(pos_2);
    delay(del_t); 
  }  

  
}    

void setup(){ 
    //put your setup code here, to run once: 
    //ros::init(argc, argv, "grab_listener");
    for(byte i = 0; i < SERVO_NUM; i++ )
    { 
        myservo[i].attach(servo_pin[i]);    // 将10引脚与声明的舵机对象连接起来
    }    
    nh.initNode();
    nh.subscribe(sub);
    myservo[0].write(110);
    myservo[1].write(180);
    myservo[2].write(180);  
    myservo[3].write(160);
    Serial.begin(57600);
    //ros::Subscriber sub = nh.subscribe("grab", 1, move_or_not);
} 


void loop(){ 
    nh.spinOnce();
    //put your main code here, to run repeatedly:
    //while(1);
} 
