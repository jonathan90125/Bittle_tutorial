
int pirPin = 9;

int pirValue;
int sec = 0;

void setup()
{
    pinMode(pirPin, INPUT);
    Serial.begin(9600);
}

void loop()
{
    pirValue = digitalRead(pirPin);
   
   //  用于显示传感器输出状态
     sec += 1;
     Serial.print("Second: ");
     Serial.print(sec);
     Serial.print("PIR value: ");
     Serial.print(pirValue);
     Serial.print('\n');
     delay(1000);
}
 
