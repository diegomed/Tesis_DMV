#include <Wire.h>
#include <SoftwareSerial.h>
//SoftwareSerial mySerial(8, 9);// software serial #1: RX = digital pin 8, TX = digital pin 9
//SoftwareSerial mySerial(19, 18);// software serial #1: RX = digital pin 19, TX = digital pin 18
float duty1, duty2, duty3, duty4, duty5, duty6;
char buffer[6];

class PWM {

    int ledPin;
    float onTime;
    float offTime;
    int ledState;
    unsigned long previousMillis;

  public:
    PWM(int pin, float f, float delta) {
      ledPin = pin;
      pinMode(ledPin, OUTPUT);

      onTime = (1 / (f * 0.001)) * delta;
      offTime = (1 / (f * 0.001)) * (1 - delta);

      ledState = LOW;
      previousMillis = 0;
    }

    void Update(float f, float delta) {
      onTime = (1 / (f * 0.001)) * delta;
      offTime = (1 / (f * 0.001)) * (1 - delta);
      unsigned long currentMillis = millis();

      if ((ledState == HIGH) && (currentMillis - previousMillis >= onTime)) {
        ledState = LOW;
        previousMillis = currentMillis;
        digitalWrite(ledPin, ledState);
      }
      else if ((ledState == LOW) && (currentMillis - previousMillis >= offTime)) {
        ledState = HIGH;
        previousMillis = currentMillis;
        digitalWrite(ledPin, ledState);
      }
    }
};

PWM Vpwm_derecha1(2, 5, 0);
PWM Vpwm_derecha2(3, 5, 0);
PWM Vpwm_derecha3(4, 5, 0);

PWM Vpwm_izquierda1(5, 5, 0);
PWM Vpwm_izquierda2(6, 5, 0);
PWM Vpwm_izquierda3(7, 5, 0);




void setup() {
  // put your setup code here, to run once:
    Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);
  //Serial1.begin(9600);
  
}

void loop() {
  // put your main code here, to run repeatedly:

 int i = 0;
 int input;
 
 /*
while (Serial.available() > 0) {
    input = Serial.read(); //Leo dato
    
    if (input != '#') {
      buffer[i] = input;
      //Serial.print("duty   ");
      //Serial.println(i);
      
      //Serial.println(buffer[i]);
      buffer[i + 1] = '\0';
      i++;
    } 
 }
 */
 i=0;
// COMUNICACION
Wire.requestFrom(8, 6);    // request 6 bytes from slave device #8
 
  while (Wire.available()) { // slave may send less than requested
   
     char c = Wire.read(); // receive a byte as character
      buffer[i]=c;
      i++;
   //  Serial.print(buffer); 
  }

  
  delay(10);
  duty1= float(buffer[0])/100;
  Vpwm_derecha1.Update(5, duty1);
  
  duty2= float(buffer[1])/100;
  Vpwm_derecha2.Update(5, duty2);
  
  duty3= float(buffer[2])/100;
  Vpwm_derecha3.Update(5, duty3);

  duty4= float(buffer[3])/100;
  Vpwm_izquierda1.Update(5, duty4);
  
  duty5= float( buffer[4])/100;
  Vpwm_izquierda2.Update(5, duty5);
  
  duty6= float(buffer[5])/100;
  Vpwm_izquierda3.Update(5, duty6);

Serial.println(int(buffer[0]));
Serial.println(int(buffer[1]));
Serial.println(int(buffer[2]));
Serial.println(int(buffer[3]));
Serial.println(int(buffer[4]));
Serial.println(int(buffer[5]));

}



