#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Arduino.h>
#include <MuVisionSensor.h>


#define SERVOMIN 180*4  //最小电机占空比计数
#define SERVOMAX 620*4  //最大电机占空比计数
#define MU_ADDRESS    0x60


uint8_t devStatus;
Adafruit_PWMServoDriver pwm=Adafruit_PWMServoDriver();
MPU6050 mpu;
MuVisionSensor Mu(MU_ADDRESS);


//前进步态
  char trF[36][8] = {
  61,  68,  54,  61, -26, -39, -13, -26,
  66,  61,  58,  55, -26, -39, -13, -26,
  70,  54,  61,  49, -26, -37, -12, -25,
  73,  46,  65,  43, -24, -35, -11, -23,
  77,  39,  67,  36, -23, -33, -10, -21,
  80,  33,  70,  31, -21, -30,  -8, -19,
  83,  26,  72,  25, -19, -26,  -6, -15,
  85,  19,  74,  18, -16, -20,  -3,  -9,
  86,  16,  76,  14, -13, -12,  -1,   0,
  87,  19,  76,  16,  -9, -11,   3,   2,
  93,  24,  82,  21, -16, -14,  -5,  -2,
  96,  29,  85,  26, -24, -17, -12,  -5,
  95,  34,  84,  30, -28, -20, -17,  -7,
  92,  39,  81,  35, -31, -22, -20,  -9,
  89,  44,  79,  39, -34, -24, -22, -11,
  84,  49,  74,  44, -36, -25, -24, -12,
  79,  54,  70,  48, -38, -26, -26, -13,
  72,  59,  65,  52, -39, -26, -26, -13,
  65,  64,  59,  56, -39, -26, -26, -13,
  60,  68,  54,  60, -38, -26, -26, -13,
  52,  72,  48,  62, -37, -25, -25, -12,
  45,  75,  41,  66, -35, -24, -23, -11,
  37,  79,  35,  69, -32, -22, -20,  -9,
  30,  81,  29,  71, -29, -20, -17,  -7,
  24,  83,  24,  73, -25, -17, -14,  -4,
  19,  85,  18,  75, -19, -14,  -7,  -2,
  16,  87,  14,  76, -10, -11,   1,   1,
  20,  90,  17,  79, -12, -12,   1,   0,
  25,  96,  22,  85, -15, -20,  -2,  -9,
  30,  97,  26,  85, -18, -26,  -5, -15,
  35,  94,  31,  83, -20, -30,  -8, -18,
  40,  91,  36,  80, -23, -33, -10, -21,
  45,  86,  40,  76, -24, -36, -11, -24,
  51,  81,  45,  72, -25, -37, -12, -25,
  55,  76,  49,  68, -26, -38, -13, -26,
  60,  70,  53,  62, -26, -39, -13, -26,
};
   
//左转步态 
  char trL[29][8]  = {
 62,  69,  54,  57, -28, -40, -13, -18,
  63,  60,  59,  55, -28, -40, -13, -18,
  64,  51,  62,  53, -28, -37, -12, -18,
  65,  42,  67,  52, -28, -34, -10, -17,
  67,  33,  70,  50, -27, -30,  -8, -17,
  69,  23,  73,  49, -27, -22,  -5, -17,
  70,  18,  75,  46, -27, -14,  -2, -15,
  72,  17,  76,  45, -27,  -7,   2, -14,
  73,  24,  82,  46, -29, -13,  -5, -14,
  72,  29,  85,  48, -30, -17, -14, -14,
  72,  36,  83,  49, -30, -20, -18, -14,
  70,  42,  79,  51, -31, -22, -22, -15,
  68,  49,  74,  52, -31, -25, -24, -15,
  66,  55,  69,  53, -31, -26, -26, -15,
  65,  61,  62,  55, -31, -26, -26, -15,
  62,  67,  55,  56, -31, -26, -26, -15,
  61,  72,  48,  57, -31, -25, -25, -15,
  59,  77,  40,  58, -31, -25, -23, -15,
  57,  81,  32,  60, -31, -22, -19, -14,
  55,  84,  25,  61, -30, -19, -15, -15,
  51,  87,  18,  61, -28, -16,  -7, -14,
  51,  88,  15,  63, -25, -11,   2, -14,
  52,  96,  20,  63, -27, -19,  -1, -14,
  54,  98,  26,  64, -27, -27,  -5, -17,
  55,  97,  31,  63, -27, -32,  -8, -17,
  56,  93,  37,  61, -27, -36, -10, -17,
  58,  87,  42,  60, -28, -39, -12, -17,
  60,  79,  48,  59, -28, -40, -13, -18,
  61,  70,  53,  58, -28, -40, -13, -18,
};

//右转步态
  char trR[29][8]  = {
  61,  64,  55,  61, -26, -31, -15, -26,
  67,  62,  56,  54, -26, -31, -15, -26,
  72,  60,  57,  47, -25, -31, -15, -24,
  77,  59,  58,  39, -25, -31, -15, -22,
  81,  56,  60,  31, -22, -30, -14, -19,
  84,  54,  61,  24, -19, -30, -15, -14,
  87,  51,  61,  16, -16, -27, -14,  -5,
  88,  51,  63,  15, -11, -25, -14,   2,
  96,  52,  63,  21, -19, -27, -14,  -2,
  98,  54,  64,  26, -27, -27, -17,  -5,
  97,  55,  63,  32, -32, -27, -17,  -8,
  93,  57,  61,  38, -36, -27, -17, -10,
  87,  58,  60,  44, -39, -28, -17, -12,
  79,  60,  59,  49, -40, -28, -18, -13,
  70,  62,  58,  54, -40, -28, -18, -13,
  62,  63,  55,  59, -40, -28, -18, -13,
  52,  64,  54,  62, -37, -28, -18, -12,
  43,  65,  52,  67, -35, -28, -17, -10,
  34,  67,  51,  70, -31, -27, -17,  -8,
  26,  69,  49,  73, -25, -27, -17,  -5,
  18,  70,  46,  75, -15, -27, -15,  -2,
  17,  72,  45,  76,  -7, -27, -14,   2,
  22,  73,  46,  82, -12, -29, -14,  -5,
  28,  72,  48,  85, -16, -30, -14, -14,
  35,  72,  49,  83, -19, -30, -14, -18,
  41,  70,  50,  79, -22, -31, -14, -22,
  48,  68,  52,  74, -25, -31, -15, -24,
  54,  66,  53,  69, -26, -31, -15, -26,
  60,  65,  54,  62, -26, -31, -15, -26,
 };

   char stp[1][8]  = {
   30,30,30,30,30,30,30,30,
 };
 

//校准电机误差
char cal[16]= {0,  0,  0,  0,  0,  0,  0,  0,  3,  4,  -2,  0, 4,  4, 0, 0};

//关节映射表
byte pins[] = {12, 11, 3, 4,
               13, 10, 5, 2,
               14, 9, 6, 1,
               15, 8, 7, 0
              };

//旋转方向
int8_t rotationDirections[] = {1, -1, 1, 1,
                               1, -1, 1, -1,
                               1, -1, -1, 1,
                               -1, 1, 1, -1
                              };
//角度转化为占空比计数时需要的偏移量
int8_t middleShifts[] = {0, 15, 0, 0,
                         -45, -45, -45, -45,
                         45, 45, -45, -45,
                         -75, -75, -75, -75
                        };


 //将角度转化为占空比计数                       
int S2P(int angle,int i){
  float p=0.0; 
  p = SERVOMIN + 880 + float(middleShifts[i] + cal[i]) * 7.04  * rotationDirections[i] + angle * 7.04 * rotationDirections[i];
  return int(p);
}



volatile bool mpuInterrupt = false;     // 表示mpu interrupt 引脚是否被拉高

void dmpDataReady() {
  mpuInterrupt = true;  //表示发生了中断
}


void setup() {
  //配置串口
  Serial.begin(57600);
  //配置电机pwm
  pwm.begin();
  pwm.setPWMFreq(240);

   uint8_t err = 0;
   Wire.begin();
  // initialized MU on the I2C port
  err = Mu.begin(&Wire);

  if (err == MU_OK) {
    Serial.println("MU initialized.");
  } else {
    do {
      Serial.println("fail to initialize MU! Please check protocol "
                     "version or make sure MU is working on the "
                     "correct port with correct mode.");
      delay(5000);
    } while (1);
  }
  // enable vision: traffic card
  Mu.VisionBegin(VISION_TRAFFIC_CARD_DETECT);
}



 
int t=5; 


void loop() {

   // put your main code here, to run repeatedly:
  long time_start = millis();

  // read result
  if (Mu.GetValue(VISION_TRAFFIC_CARD_DETECT, kStatus)) {                   // update vision result and get status, 0: undetected, other: detected
    Serial.println("vision traffic card detected:");
    Serial.print("x = ");
    Serial.println(Mu.GetValue(VISION_TRAFFIC_CARD_DETECT, kXValue));       // get vision result: x axes value
    Serial.print("y = ");
    Serial.println(Mu.GetValue(VISION_TRAFFIC_CARD_DETECT, kYValue));       // get vision result: y axes value
    Serial.print("width = ");
    Serial.println(Mu.GetValue(VISION_TRAFFIC_CARD_DETECT, kWidthValue));   // get vision result: width value
    Serial.print("height = ");
    Serial.println(Mu.GetValue(VISION_TRAFFIC_CARD_DETECT, kHeightValue));  // get vision result: height value
    Serial.print("label = ");
    switch (Mu.GetValue(VISION_TRAFFIC_CARD_DETECT, kLabel)) {              // get vision result: label value
      case MU_TRAFFIC_CARD_FORWARD:
        Serial.println("forward");
        t=0;
        break;
      case MU_TRAFFIC_CARD_LEFT:
        Serial.println("left");
        t=1;
        break;
      case MU_TRAFFIC_CARD_RIGHT:
        Serial.println("right");
        t=2;
        break;
      case MU_TRAFFIC_CARD_TURN_AROUND:
        Serial.println("turn around");
        t=3;
        break;
      case MU_TRAFFIC_CARD_PARK:
        Serial.println("park");
        t=4;
        break;
      default:
        Serial.print("unknow card type: ");
        Serial.println(Mu.GetValue(VISION_TRAFFIC_CARD_DETECT, kLabel));
        break;
    }
  } else {
    Serial.println("vision shape card undetected.");
  }
  Serial.print("fps = ");
  Serial.println(1000/(millis()-time_start));
  Serial.println();


  
      //控制部分
      if(t==0){//前进
    for(int i=0;i<=35;i++){
    for(int j=0;j<=7;j++){
      //根据步态数据设置每个关节的转角
      pwm.setPWM(pins[8+j],0,S2P(   (int(trF[i][j]))  ,8+j  ));
    }
    }
      }
     else if(t==1){//左转
       for(int i=0;i<=28;i++){
    for(int j=0;j<=7;j++){  
      pwm.setPWM(pins[8+j],0,S2P(   (int(trL[i][j]))  ,8+j  ));
    }
    }
      }    
     else if(t==2){//右转
      for(int i=0;i<=28;i++){
    for(int j=0;j<=7;j++){
      pwm.setPWM(pins[8+j],0,S2P(   (int(trR[i][j]))  ,8+j  ));
    }
    }
      }
      else if(t==4){//停下
     for(int i=0;i<=0;i++){
    for(int j=0;j<=7;j++){
      pwm.setPWM(pins[8+j],0,S2P(   (int(stp[i][j]))  ,8+j  ));
    }
    }
      }
}





 
