#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
 

#define PACKET_SIZE 42
#define OVERFLOW_THRESHOLD 128
#define SERVOMIN 180*4
#define SERVOMAX 620*4
#define PWM_RANGE 620*4-180*4 
#define HISTORY 2
 


Adafruit_PWMServoDriver pwm=Adafruit_PWMServoDriver();
MPU6050 mpu;


int8_t lag = 0;
float ypr[3]; // [yaw, pitch, roll]存储陀螺仪得到的关于xyz轴的旋转角         
float yprLag[HISTORY][3];
float radPerDeg = M_PI / 180;
float degPerRad = 180 / M_PI;
uint8_t mpuIntStatus;   // 存储mpu中断状态
uint8_t devStatus;      // 判断进行的操作是否成功 (0 = success, !0 = error)
uint16_t packetSize;    // 期望的dmp包尺寸 (默认为 42 bytes)
uint16_t fifoCount;     //目前fifo缓存的bytes数量
uint8_t fifoBuffer[PACKET_SIZE];  
float currentAdjust[8] = {};   //根据目前的姿态每个关节需要调整的角度
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector




//处理陀螺仪数据相关函数
template <typename T> int8_t sign(T val) {
  return (T(0) < val) - (val < T(0));
}

volatile bool mpuInterrupt = false;    
void dmpDataReady() {
  mpuInterrupt = true;
}


unsigned long usedTime = 0;
void getFIFO() {//get FIFO only without further processing
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  mpu.getFIFOBytes(fifoBuffer, packetSize);
  fifoCount -= packetSize;
}


void getYPR() { 
  if (mpuInterrupt || fifoCount >= packetSize)
  {
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount > OVERFLOW_THRESHOLD) { //1024) {
      mpu.resetFIFO();
      lag = (lag - 1 + HISTORY) % HISTORY;
    }
    else if (mpuIntStatus & 0x02) {
      getFIFO();
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      for (byte g = 0; g < 3; g++)
        ypr[g] *= degPerRad;       
      for (byte g =0; g < 3; g++) {
        yprLag[lag][g] = ypr[g];
        ypr[g] = yprLag[(lag - 1 + HISTORY) % HISTORY][g] ;
      }
      lag = (lag + 1) % HISTORY;
    }
  }
}

//初始站立步态
char balance[1][8]  = {
  30, 30,  30,  30, 30, 30, 30, 30,
 };

//电机校准误差
char cal[16]= {0,  0,  0,  0,  0,  0,  0,  0,  -2,  1,  0,  0, -3,  2, -3,  8};

//关节对映表
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
                              
//计算占空比计数需要的偏移量
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


void setup() {
  //I2C部分
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin(); 
  TWBR = 24; 
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
  #endif

  //SERIAL部分
  Serial.begin(115200);
  Serial.setTimeout(10);
  while (!Serial);
  while (Serial.available() && Serial.read());  
  delay(100);


  //MPU部分
  mpu.initialize();
  delay(500);
  do{
    
    devStatus = mpu.dmpInitialize();
    delay(500);
    mpu.setZAccelOffset(985);
    mpu.setXGyroOffset(39);
    mpu.setYGyroOffset(13);
    mpu.setZGyroOffset(-6);
    if (devStatus == 0) {
      mpu.setDMPEnabled(true);
      attachInterrupt(0, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
      packetSize = mpu.dmpGetFIFOPacketSize();
    } 
  } while (devStatus);


  //PWM部分
  pwm.begin();
  pwm.setPWMFreq(240);

  //姿势初始化部分
  for (byte a = 0; a < 8; a++)
  currentAdjust[a] = 0;//将初始调整角度全部置零

    //将机器狗设置为站立姿势
    for(int j=0;j<=7;j++){
      pwm.setPWM(pins[8+j],0,S2P((int(balance[0][j])) ,8+j));
    } 
delay(500);
}


inline int8_t adaptiveCoefficient(byte idx, byte para) {
  return EEPROM.read(160 + idx * 2 + para);//从eeprom读取设置调整角度需要的参数
}


//控制前后平衡的pid参数
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0;  
float Kp = 0.1, Ki = 0.00001, Kd = 0.1;     

//控制左右平衡的pid参数
float error2 = 0, P2 = 0, I2 = 0, D2 = 0, PID_value2 = 0;
float previous_error2 = 0;  
float Kp2 = 0.01, Ki2 = 0.00001, Kd2 = 0.0001;     //kp，ki明显小于上面的kp，ki，因为左右腿之间的距离远小于前后腿，较小的高度差就能导致很大的倾斜角


float RollPitchDeviation[2];
  
void loop() {
  
  getYPR();//获取陀螺仪数据

  for (byte i = 0; i < 2; i++) {
    RollPitchDeviation[i] = ypr[2 - i]; 
    RollPitchDeviation[i] = sign(ypr[2 - i]) * max(fabs(RollPitchDeviation[i]) - 2, 0);//滤除过小的角度，防止调整过于频繁
  }

  
  error= RollPitchDeviation[1];//获取误差
  P = error; //比例
  I = I + error; //积分
  D = error - previous_error; //微分
  PID_value = (Kp * P) + (Ki * I) + (Kd * D); //P,I,D分别乘上比例系数得到结果
  previous_error = error; //更新previous_error
  
  //同上
  error2= RollPitchDeviation[0];
  P2 = error2;
  I2 = I2 + error2;
  D2 = error2 - previous_error2; 
  PID_value2 = (Kp2 * P2) + (Ki2 * I2) + (Kd2 * D2);
  previous_error2 = error2;
  
  for(int i=0;i<=0;i++){
    for(int j=0;j<=7;j++){

      //bool leftQ = (i - 1 ) % 4 > 1 ? true : false;  判断关节左右
      //bool frontQ = i % 4 < 2 ? true : false;   判断关节前后
      //bool upperQ = i / 4 < 3 ? true : false;   判断关节上下
      //当下关节旋转角度是上关节的两倍时，脚部可以刚好在竖直方向上移动
      currentAdjust[j] -=   radPerDeg *adaptiveCoefficient(j+8, 0) *(((j+7) % 4 > 1) ? ( ((j+8) / 4 < 3) ? PID_value2 : 2*PID_value2) : ( ((j+8) / 4 < 3) ? -PID_value2 : -2*PID_value2));
      currentAdjust[j] -=   radPerDeg *adaptiveCoefficient(j+8, 1) *(((j+8) % 4 < 2) ? ( ((j+8) / 4 < 3) ? PID_value : -2*PID_value) : ( ((j+8) / 4 < 3) ? -PID_value : -2*PID_value));

      if(abs(currentAdjust[j])>90)
      {
        for(int num0=0;num0<=7;num0++){
          currentAdjust[num0] =0;
        
        pwm.setPWM(pins[8+num0],0,S2P((int(balance[i][num0]))+currentAdjust[num0],8+num0));}
        delay(3000);
        break;}
      pwm.setPWM(pins[8+j],0,S2P((int(balance[i][j]))+currentAdjust[j],8+j));//初始站立的步态转角再加上处理出来的需要调整的角度
 
    }}
   
 
}
