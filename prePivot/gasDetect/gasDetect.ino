#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
char auth[] = ""; // the auth code that you got on your gmail
char ssid[] = ""; // username or ssid of your WI-FI
char pass[] = ""; // password of your Wi-Fi
int gasValue = A0; // smoke / gas sensor connected with analog pin A1 of the arduino / mega.
int data = 0;
uint8_t buzzer = D4; 

//Parameters for declaring alert
int CutOff = 300;
unsigned long currentTime;
unsigned long lastTime;
unsigned long delayTime = 5000;
bool LeakDetected = false;
WidgetLED led1(V1);

void setup()
{
  Serial.begin(9600);
  pinMode(buzzer,OUTPUT);
  pinMode(gasValue, INPUT);
  digitalWrite(buzzer,LOW);
  Blynk.begin(auth, ssid, pass);
  currentTime = millis();
  lastTime = currentTime;
}

void loop()
{
  Blynk.run();
  currentTime = millis();
  data = analogRead(gasValue);
  Serial.println(data);
  if ( data > CutOff) //
  {
    digitalWrite(buzzer, HIGH);
    led1.on();

    if (!LeakDetected){
      lastTime = millis();
      LeakDetected = true;
    }
    else{
      if(currentTime-lastTime>delayTime){
        Blynk.notify("ALERT! GAS LEAK DETECTED!");
        LeakDetected = false;
      }  
    }
  }
  else
  {
    digitalWrite(buzzer, LOW);
    led1.off();
    LeakDetected = false;
  }
  
}
