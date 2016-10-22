#include <os48.h>
#include <SD.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>

#define CANT_MUESTRAS 7     // largo del array de muestras para hacer el filtrado
#define I_MUESTRA     3     // muestra del array utilizada para hacer los calculos en ese momento 
#define UMBRAL        10    // Diferencia entre la media y la medida tomada;
#define TIME_SENS     10    // tiempo entre muestras
#define LAR_BUFFER    7

#define VENTANA_SENSOR         0.60   //Ancho que agarra cada sensor en metros
#define DISTANCIA_ENTRE_FILAS   3.5   // distancia entre las filas en metros
#define FACTOR                  600   //factor para calculo de caudal
#define DOSIS                    50   //litros cada 1000 metros cubicos de vegetacion
#define CAUDAL_MAX_2BOQUILLAS     4   // caudal maximo de dos boquillas a una presion especificada 

#define mySerial Serial1

using namespace os48;

//SoftwareSerial mySerial(8,7); //el primer parametro es el RX del arduino, el segundo el TX
Adafruit_GPS GPS(&mySerial);

float duty1, duty2, duty3, duty4, duty5, duty6;
char buffer[6];
int chipSelect = 53;
File myData; //creo objeto de tipo File

String NMEA;

//int trigPin=4; //Sensor Trip pin connected to Arduino pin 13
//int echoPin=2;  //Sensor Echo pin connected to Arduino pin 11
//float pingTime;

class sensor {

  public:
    int i;
    float           dato[CANT_MUESTRAS];
    unsigned long   SensorMillis, CurrentMillis;
    const float     TimeSens = TIME_SENS;
    int             pin4, pin2;
    float           cm, duration;
    float           avg_cm[2];
    float           dif[2];

    sensor(int pin_enable, int pin_outPWM) {
      int n, i;
      pin4 = pin_enable;        // el pin 4 del sensor es el enable
      pin2 = pin_outPWM;        // el pin 2 del sensor es la salida pem en funcion de la distancia
      SensorMillis = 0;
      n = CANT_MUESTRAS;
      i = 0;
      cm = 0.0;
      
      // configuro los pines
      pinMode(pin4, OUTPUT);
      pinMode(pin2, INPUT);
      digitalWrite(pin4, LOW);    // apago el sensor

      while (i < n) {             //Tomo N muestras y las guardo en dato[]
        digitalWrite(pin4, HIGH);
        delayMicroseconds(5);

        duration = pulseIn(pin2, HIGH);
        cm = microsecondsToCentimeters(duration);


        dato[i] = cm;
        i++;

        digitalWrite(pin4, LOW);
        delayMicroseconds(100);   //Tiempo minimo recomendado entre lecturas
      }
    }

    void UpdateSensor() {
      int n, i;
      n = CANT_MUESTRAS;
      CurrentMillis = millis();

      //Serial.print(TimeSens);
      if (CurrentMillis - SensorMillis >= TimeSens) { // Tiempo cada cuanto funciona el sensor

      for (i = 0; i < n - 1; ++i) {             // Primero corre los valores del array dejando libre el ultimo
          dato[i] = dato[i + 1];
        }


        digitalWrite(pin4, HIGH);
        //delayMicroseconds(5);
        duration = pulseIn(pin2, HIGH);
        cm = microsecondsToCentimeters(duration);
        dato[n - 1] = cm;
        digitalWrite(pin4, LOW);

//FILTRADO DE MUESTRAS
        avg_cm[0] =  mean_primera(dato);              //Calculo del promedio en cm
        avg_cm[1] =  mean_ultima(dato);               //Calculo del promedio en cm
        dif[0] = abs(avg_cm[0] - dato[I_MUESTRA]);
        dif[1] = abs(avg_cm[1] - dato[I_MUESTRA]);


        if (dif[0] > UMBRAL) {

          if (dif[1] > UMBRAL) {


            dato[I_MUESTRA] = avg_cm[1] ;
            //Serial.print("descarta  ");
          }
        }
        SensorMillis = millis();
      }
    }

    long microsecondsToCentimeters(long microseconds) {
      // The speed of sound is 340 m/s or 29 microseconds per centimeter.
      // The ping travels out and back, so to find the distance of the
      // object we take half of the distance travelled.
      return microseconds / 29 / 2;
    }

    float mean_primera(float dato[]) {
      int i;
      float mean;

      for (i = 0; i < I_MUESTRA; ++i) {
        mean += dato[i];
      }


      return mean / I_MUESTRA;
    }

    float mean_ultima(float dato[]) {

      int  i;
      float mean;
      int init;
      init = I_MUESTRA + 1;

      for (i = init; i < CANT_MUESTRAS; ++i) {
        mean += dato[i];
      }


      return mean / I_MUESTRA;
    }
};

class TRV {
    public:
    float Calc_TRV;
    float Litros_ha;
    float Caudal;
    float Ancho_Copa;
    int duty_valv;

    public:
    TRV() {}
  
  void Duty_TRV(float Medida, float Velocidad_avance, float Entre_filas) { // Medida del sensor en cm, velocidad de avance en Km y entre filas en m
    
    Medida = Medida / 100;
  
    if (Entre_filas / 2 > Medida) {
  
      Ancho_Copa = Entre_filas / 2 - Medida;
  
      Calc_TRV = (VENTANA_SENSOR * Ancho_Copa * 10000) / DISTANCIA_ENTRE_FILAS;
  
      Litros_ha = (Calc_TRV * DOSIS) / 1000;
  
      Caudal = (Litros_ha * Velocidad_avance * DISTANCIA_ENTRE_FILAS) / 600;
  
      duty_valv = (Caudal / CAUDAL_MAX_2BOQUILLAS) * 100;
  
      if (duty_valv < 35) { // duty mas pequeño al que la valvula trabaja
  
        duty_valv = 35;
      }
    }
    else {
      duty_valv = 0;
    }
  }
};

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

    onTime = (1/(f*0.001))*delta;
    offTime = (1/(f*0.001))*(1-delta);

    ledState = LOW;
    previousMillis = 0;
  }
  
  void Update(float f, float delta) {
    onTime = (1/(f*0.001))*delta;
    offTime = (1/(f*0.001))*(1-delta);
    unsigned long currentMillis = millis();

    if ((ledState == HIGH) && (currentMillis - previousMillis >= onTime)) {
      ledState = LOW;
      previousMillis = currentMillis;
      digitalWrite(ledPin,ledState);
    }
    else if ((ledState == LOW) && (currentMillis - previousMillis >= offTime)) {
      ledState = HIGH;
      previousMillis = currentMillis;
      digitalWrite(ledPin,ledState);
    }
  }
};

PWM pwm13(13,5,0.5);

sensor S_derecha1(2, 3);
sensor S_derecha2(22, 23);
sensor S_derecha3(6, 7);

sensor S_izquierda1(4, 5);
sensor S_izquierda2(10, 11);
sensor S_izquierda3(12, 13);

TRV TreeRowVolume;

Scheduler* scheduler = Scheduler::get(); //Get the instance

//Declare the task pointers as global vars to use them in the task functions.
//Task* task1 = NULL; 
Task* task2 = NULL;
Task* task3 = NULL;
Task* taskP = NULL;

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
  GPS.begin(9600);
//  pinMode(trigPin, OUTPUT);
//  pinMode(echoPin, INPUT);
  pinMode(chipSelect, OUTPUT); //esto hay que hacerlo pare el DataLogger, porque si
  digitalWrite(chipSelect, HIGH);//esto hay que hacerlo para el DataLogger, porque si
  SD.begin(chipSelect); //empiezo la comunicacion con la SD, CS (chip select) esta conectado al pin 53 del arduino
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onRequest(requestEvent); // register event
  delay(100);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  delay(100);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  delay(1500);
  myData = SD.open("PTData.txt", FILE_WRITE);
  delay(100);
  if(myData) {
    myData.print("ancho derecha crudo:");
    myData.print(",");
    myData.print("ancho derecha filtrado:");
    myData.print(",");
    myData.print("Calc_TRV derecha");
    myData.print(",");
    myData.print("Litros_ha derecha");
    myData.print(",");
    myData.print("Caudal derecha");
    myData.print(",");
    myData.print("Ancho_Copa derecha");
    myData.print(",");
    myData.print("duty_valv derecha");
    myData.print(",");
    myData.print("ancho izquierda crudo:");
    myData.print(",");
    myData.print("ancho izquierda filtrado:");
    myData.print(",");
    myData.print("Calc_TRV izquierda");
    myData.print(",");
    myData.print("Litros_ha derecha");
    myData.print(",");
    myData.print("Caudal izquierda");
    myData.print(",");
    myData.print("Ancho_Copa izquierda");
    myData.print(",");
    myData.println("duty_valv izquierda");
    myData.close();
    delay(100);
  }
  Serial.println("Creating tasks...");

//  task1 = scheduler->createTask(&func1, 60); //Creates a task associated to the 'func1' function with 60 bytes of stack memory.
//  task1->clearStackFootprints(); //IMPORTANT to call Task::printMem()
  task2 = scheduler->createTask(&func2, 120);
  task2->clearStackFootprints(); //IMPORTANT to call Task::printMem()
  task3 = scheduler->createTask(&func3, 240);
  task3->clearStackFootprints(); //IMPORTANT to call Task::printMem()
//  taskP = scheduler->createTask(&taskPFunc, 150); //id 3
//  taskP->clearStackFootprints(); //IMPORTANT to call Task::printMem()

  scheduler->setStackOverflowFnc(&fncSO); // <-- Define your custom function here

  Serial.println("Starting...");
  scheduler->start(); //Starts the scheduler. At this point you enter in a multi tasking context.

  //...
  //Nothing will be executed here
}

//void func1()
//{
//  for(;;)
//  {    
//    //pwm13.Update(5,0.5);
//    digitalWrite(13, HIGH);
//    task()->sleep(100);
//    digitalWrite(13, LOW);
//    task()->sleep(100);
//  }
//}

void func2()
{ 
  for(;;)
  {
    while(!GPS.newNMEAreceived()) { 
      char c = GPS.read();
    }
    NMEA = GPS.lastNMEA();
    Serial.println(NMEA);

    GPS.parse(GPS.lastNMEA());
    Serial.print("Velocidad: ");
    Serial.println(GPS.speed);
    Serial.print("Ubicacion: ");
    Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
    Serial.print(", "); 
    Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
  }
}

void func3()
{
  for(;;)
  {    
    float salida;
    float velocidad;
    int i;
    i = 0;
    velocidad = 5.4;

    myData = SD.open("PTData.txt", FILE_WRITE);
    
    S_derecha1.UpdateSensor();
//    Serial.println("ancho derecha crudo: ");
//    Serial.println(S_derecha1.cm);
//    Serial.println("ancho derecha filtrado: ");
//    Serial.println(S_derecha1.dato[I_MUESTRA]);
    if(myData) {
      myData.print(S_derecha1.cm);
      myData.print(",");
    }

    if(myData) {
      myData.print(S_derecha1.dato[I_MUESTRA]);
      myData.print(",");
    }

    TreeRowVolume.Duty_TRV(S_derecha1.dato[I_MUESTRA], velocidad, DISTANCIA_ENTRE_FILAS);
    duty1 = TreeRowVolume.duty_valv;
    buffer[0] = char(duty1);

    if(myData) {
      myData.print(TreeRowVolume.Calc_TRV);
      myData.print(",");
      myData.print(TreeRowVolume.Litros_ha);
      myData.print(",");
      myData.print(TreeRowVolume.Caudal);
      myData.print(",");
      myData.print(TreeRowVolume.Ancho_Copa);
      myData.print(",");
      myData.print(TreeRowVolume.duty_valv);
      myData.print(",");
    }

    S_derecha2.UpdateSensor();
    //tv.writeLCD(S_derecha2.cm);
    duty2  = 50;//Duty_TRV(S_derecha2.dato[I_MUESTRA], velocidad, DISTANCIA_ENTRE_FILAS);
    buffer[1] = char(duty2);
    
    
    S_derecha3.UpdateSensor();
    //tv.writeLCD(S_derecha3.cm);
    duty3  = 60;//S_derecha3.cm / 200;
    buffer[2] = char(duty3);

    S_izquierda1.UpdateSensor();
//    Serial.println("ancho izquierda crudo: ");
//    Serial.println(S_izquierda1.cm);
//    Serial.println("ancho izquierda filtrado: ");
//    Serial.println(S_izquierda1.dato[I_MUESTRA]);

    if(myData) {
      myData.print(S_izquierda1.cm);
      myData.print(",");
    }

    if(myData) {
      myData.print(S_izquierda1.dato[I_MUESTRA]);
      myData.print(",");
    }

    TreeRowVolume.Duty_TRV(S_izquierda1.dato[I_MUESTRA], velocidad, DISTANCIA_ENTRE_FILAS);
    duty4 = TreeRowVolume.duty_valv;
    buffer[3] = char(duty4);

    if(myData) {
      myData.print(TreeRowVolume.Calc_TRV);
      myData.print(",");
      myData.print(TreeRowVolume.Litros_ha);
      myData.print(",");
      myData.print(TreeRowVolume.Caudal);
      myData.print(",");
      myData.print(TreeRowVolume.Ancho_Copa);
      myData.print(",");
      myData.println(TreeRowVolume.duty_valv);
    }

    S_izquierda2.UpdateSensor();
    //tv.writeLCD(S_izquierda2.cm);
    duty5  = 80;//S_izquierda2.cm / 200;
    buffer[4] = char(duty5);
  
  
    S_izquierda3.UpdateSensor();
    //tv.writeLCD(S_izquierda3.cm);
    duty6  = 90;//S_izquierda3.cm / 200;
    buffer[5] = char(duty6);

    if(myData) {
      myData.close();
    }
    
  }
}

void fncSO()
{
  Serial.println("Stack overflow!");
  Serial.print("ID of the task affected: ");
  Serial.println(task()->getId());
  Serial.print("Free stack size: ");
  Serial.println(task()->getUserFreeStackSize()); 
  Serial.flush();
}

void requestEvent() {
  Wire.write(buffer); // respond with message of 6 bytes (se conecta al 4 y 5 del uno con 20 y 21 del mega correspondientemente)
 // Serial.print("enviado");
  // as expected by master
}

//void taskPFunc()
//{
//  for (;;)
//  {
//    task()->sleep(2000);
//
//    OS48_ATOMIC_BLOCK // <-- disable all interrupts to prevent update of internal variables (because the prints consume also CPU time)
//    {
//      Serial.println(F("----------------"));
////      task1->printMem(Serial, true);
//      task2->printMem(Serial, true);
//      task3->printMem(Serial);
//      taskP->printMem(Serial);
//    }
//  }
//}

void loop() {} //Useless now but you have to let it defined.
