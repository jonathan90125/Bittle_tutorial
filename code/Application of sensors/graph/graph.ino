#include <Wire.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
 
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);
 
#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2
 
#define LOGO16_GLCD_HEIGHT 16 
#define LOGO16_GLCD_WIDTH  16 
 
 

void setup()        //初始化
{
  Serial.begin(9600);
  delay(500);

  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x64) ，0x3C为I2C协议通讯地址，需根据实际情况更改
}
void loop()
{
  test_SSD1306();     //调用测试函数
}

void test_SSD1306(void)   //测试函数
{
/*-----------------点亮全屏检测屏幕是否有不正常点亮现象-----------------------------*/
  
  display.fillScreen(WHITE);
  display.display();
  delay(2000);

/*------------------------------画点 点坐标(64,32)-------------------------------*/
  display.clearDisplay();   // clears the screen and buffer
  display.drawPixel(64, 32, WHITE);
  display.display();
  delay(2000);

  /*-------------------------- 画线 从(0,0)到(128,64)----------------------------*/
  display.clearDisplay();   // clears the screen and buffer
  display.drawLine(0, 0,128,64, WHITE);
  display.display();
  delay(2000);

/*--------------.画空心矩形  左上角坐标(x0,y0)  右下角坐标(x1,y1)------------------*/
  display.clearDisplay();   // clears the screen and buffer
  display.drawRect(0,0,128,64,WHITE);
  display.display();
  delay(2000);

/*-----------------------实心矩形---------------------------*/
  display.clearDisplay();   // clears the screen and buffer
  display.fillRect(0,0,128,64,WHITE);
  display.display();
  delay(2000);

/*------------------------画空心圆-------------------------*/
  display.clearDisplay();   // clears the screen and buffer
  display.drawCircle(64,32,20,WHITE);
  display.display();
  delay(2000);

/*----------------------画实心圆---------------------------*/
  display.clearDisplay();   // clears the screen and buffer
  display.fillCircle(128,64,20,WHITE);
  display.display();
  delay(2000);

/*---------------------画空心三角形-------------------------*/
  display.clearDisplay();   // clears the screen and buffer
  display.drawTriangle(64,0,0,63,128,63,WHITE);
  display.display();
  delay(2000);
 
/*------------------------画实心三角形-----------------------*/
  display.clearDisplay();   // clears the screen and buffer
  display.fillTriangle(64,0,0,63,128,63,WHITE);
  display.display();
  delay(2000);
 
/*-----------------------空心圆角矩形------------------------*/
  display.clearDisplay();   // clears the screen and buffer
  display.drawRoundRect(0,0,128,64,5,WHITE);
  display.display();
  delay(2000);
 
/*----------------------画实心圆角矩形-----------------------*/
  display.clearDisplay();   // clears the screen and buffer
  display.fillRoundRect(0,0,128,64,5,WHITE);
  display.display();
  delay(2000);

/*------------------------显示英文 数字---------------------*/
  display.clearDisplay();   // clears the screen and buffer
  display.setTextSize(1); //选择字号
  display.setTextColor(WHITE);  //字体颜色
  display.setCursor(0,0);   //起点坐标
  display.println("Hello, Arduino!");
  display.setTextColor(BLACK, WHITE); // 'inverted' text
  display.println(3.141592);
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.print("0x"); display.println(0xDEADBEEF, HEX);
  display.display();
  delay(2000);
  }
 
