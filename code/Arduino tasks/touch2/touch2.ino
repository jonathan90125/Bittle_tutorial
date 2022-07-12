#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <MPU6050_6Axis_MotionApps20.h>



#define SERVOMIN 180*4
#define SERVOMAX 620*4
#define PWM_RANGE 620*4-180*4
#define DOF 16
uint8_t devStatus;
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel pixels_10 = Adafruit_NeoPixel(7, 10, NEO_GRB + NEO_KHZ800); 

Adafruit_PWMServoDriver pwm=Adafruit_PWMServoDriver();
MPU6050 mpu;

  char balance[1][8]  = {
  30, 30,  30,  30, 30, 30, 30, 30,
 
 };
 


char cal[16]= {0,  0,  0,  0,  0,  0,  0,  0,  -2,  1,  0,  0, -3,  2, -3,  -8};

byte pins[] = {12, 11, 3, 4,
               13, 10, 5, 2,
               14, 9, 6, 1,
               15, 8, 7, 0
              };

int8_t rotationDirections[] = {1, -1, 1, 1,
                               1, -1, 1, -1,
                               1, -1, -1, 1,
                               -1, 1, 1, -1
                              };

int8_t middleShifts[] = {0, 15, 0, 0,
                         -45, -45, -45, -45,
                         45, 45, -45, -45,
                         -75, -75, -75, -75
                        };


                        
int S2P(int angle,int i){
  float p=0.0; 
  p = SERVOMIN + 880 + float(middleShifts[i] + cal[i]) * 7.04  * rotationDirections[i] + angle * 7.04 * rotationDirections[i];
  return int(p);
}


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


void setup() {
  Serial.begin(57600);
  pwm.begin();
  pwm.setPWMFreq(240);


  mpu.initialize();
    do {   
    devStatus = mpu.dmpInitialize();
    delay(500); 
    mpu.setZAccelOffset(985);
    mpu.setXGyroOffset(39);
    mpu.setYGyroOffset(13);
    mpu.setZGyroOffset(-6);
  
    if (devStatus == 0) {   
      mpu.setDMPEnabled(true);
      attachInterrupt(0, dmpDataReady, RISING);
    
    } 
    
  } while (devStatus);
}




float AR=16384.0;
float GR=131.0;
int16_t ax=0,ay=0,az=0,gx=0,gy=0,gz=0;
float accx=0,accy=0,accz=0;
float sum=0;
float gxx,gyy;

 

void loop() {
  mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);

   gyy=gy/GR;
   Serial.print("sum: ");Serial.println(gyy); 
   if(gyy<0){
     pixels_10.setBrightness(255);
  pixels_10.begin();
    for (int count = 0; count < 7; count++) {
      pixels_10.setPixelColor(count, pixels_10.Color(0xff, 0x00, 0x00));
  }
  pixels_10.show();
  for(int t=1;t<=30 ;t++){
    for( int j=0;j<=7;j++){
        pwm.setPWM(pins[8+j],0,S2P( t,8+j));}
               delay(5);}
  delay(2000);
   for(int t=30;t>=0 ;t--){
    for( int j=0;j<=7;j++){
        pwm.setPWM(pins[8+j],0,S2P( t,8+j));}
               delay(5);}
  delay(2000);
   }
   else{
     pixels_10.setBrightness(255);
  pixels_10.begin();
    for (int count = 0; count < 7; count++) {
      pixels_10.setPixelColor(count, pixels_10.Color(0x00, 0xff, 0x00));
  }
  pixels_10.show();
 
    for( int j=0;j<=7;j++){
        pwm.setPWM(pins[8+j],0,S2P( 0,8+j));}

      }
              

   }
 
       

      
 
