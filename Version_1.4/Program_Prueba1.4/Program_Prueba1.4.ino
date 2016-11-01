
#include <SD.h>
//#include <SPI.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>


#define CANT_MUESTRAS 7     // largo del array de muestras para hacer el filtrado
#define I_MUESTRA     3     // muestra del array utilizada para hacer los calculos en ese momento 
#define UMBRAL        10    // Diferencia entre la media y la medida tomada;
#define TIME_SENS     10    // tiempo entre muestras
#define LAR_BUFFER    8

#define VENTANA_SENSOR         0.60   //Ancho que agarra cada sensor en metros
#define DISTANCIA_ENTRE_FILAS   3.5   // distancia entre las filas en metros
#define FACTOR                  600   //factor para calculo de caudal
#define DOSIS                    50   //litros cada 1000 metros cubicos de vegetacion
#define CAUDAL_MAX_2BOQUILLAS     4   // caudal maximo de dos boquillas a una presion especificada 

//SoftwareSerial mySerial(19, 18);// software serial #1: RX = digital pin 19, TX = digital pin 18
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
float duty1, duty2, duty3, duty4, duty5, duty6;
float* rejunte;
char buffer[8];
int chipSelect = 53;
File myData; //creo objeto de tipo File

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


class PantallaLCD {


    float TimeLCD;
    unsigned long previousMillis;

  public:
    PantallaLCD(float Time) {// tiempo cada cuanto se quiere actualizar la pantalla

      TimeLCD = Time;
    }

    void InitLCD() {

      lcd.begin(20, 4);  // initialize the lcd for 16 chars 4 lines, turn on backlight

      for (int i = 0; i < 3; i++) // para que parpadee cuando arranca
      {
        lcd.backlight();
        delay(250);
        lcd.noBacklight();
        delay(250);
      }
      lcd.backlight(); // finish with backlight on


      //NOTE: Cursor Position: (CHAR, LINE) start at 0
      lcd.setCursor(0, 0); //Start at character 4 on line 0
      lcd.print("Tesis DVM");
      delay(1000);
      lcd.setCursor(0, 1);
      lcd.print("Atomizador");
      lcd.setCursor(29, 1);
      lcd.print("Inteligente");
      delay(8000);

      // Wait and then tell user they can start the Serial Monitor and type in characters to
      // Display. (Set Serial Monitor option to "No Line Ending")
      lcd.clear();
      lcd.setCursor(0, 1); //Start at character 0 on line 0
      lcd.print("Iniciando......");
      delay(3000);
      lcd.clear();
    }




    void writeLCD(int horizontal,int vertical,char palabra[], float salida) { //lo que se quiere escribir en la pantalla

      unsigned long currentMillis = millis();

      if ((currentMillis - previousMillis) >= TimeLCD) {
        //lcd.clear();
        lcd.setCursor(horizontal,vertical);
        lcd.print(palabra);
        lcd.setCursor(horizontal, vertical+1);
        lcd.print(salida);
        //lcd.print("cm");
        lcd.setCursor(9, 1);
        previousMillis = currentMillis;

      }

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



      if (duty_valv > 80) { // duty mas pequeño al que la valvula trabaja
  
        duty_valv = 80;
      }
    }
    else {
      duty_valv = 0;
    }
  }
};

sensor S_derecha1(2, 3);
sensor S_derecha2(4, 5);
sensor S_derecha3(6, 7);

sensor S_izquierda1(8, 9);
sensor S_izquierda2(10, 11);
sensor S_izquierda3(12, 13);

TRV TreeRowVolume;

PantallaLCD tv(50);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial2.begin(9600);
  pinMode(chipSelect, OUTPUT); //esto hay que hacerlo pare el DataLogger, porque si
  digitalWrite(chipSelect, HIGH);//esto hay que hacerlo para el DataLogger, porque si
  SD.begin(chipSelect); //empiezo la comunicacion con la SD, CS (chip select) esta conectado al pin 53 del arduino
  Wire.begin(8);                // join i2c bus with address #8
//  Wire.onRequest(requestEvent); // register event
  tv.InitLCD();
  
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
  
}

void loop() {

  float salida;
  float velocidad;
  int i;
  i = 0;
  velocidad = 5.4;
  // put your main code here, to run repeatedly:

  myData = SD.open("PTData.txt", FILE_WRITE);

  S_derecha1.UpdateSensor();
  tv.writeLCD(0,0,"ancho derecha",(S_derecha1.cm));
  Serial.println("ancho derecha crudo: ");
  Serial.println(S_derecha1.cm);
  if(myData) {
    myData.print(S_derecha1.cm);
    myData.print(",");
  }
  
 Serial.println("ancho derecha filtrado: ");
  Serial.println(S_derecha1.dato[I_MUESTRA]);
  if(myData) {
    myData.print(S_derecha1.dato[I_MUESTRA]);
    myData.print(",");
  }
  
  
  TreeRowVolume.Duty_TRV(S_derecha1.dato[I_MUESTRA], velocidad, DISTANCIA_ENTRE_FILAS);
  duty1 = TreeRowVolume.duty_valv;
  buffer[1] = char(duty1);
  
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
 

//  S_derecha2.UpdateSensor();
  //tv.writeLCD(S_derecha2.cm);
  duty2  = 50;//Duty_TRV(S_derecha2.dato[I_MUESTRA], velocidad, DISTANCIA_ENTRE_FILAS);
  buffer[2] = char(duty2);


//  S_derecha3.UpdateSensor();
  //tv.writeLCD(S_derecha3.cm);
  duty3  = 60;//S_derecha3.cm / 200;
  buffer[3] = char(duty3);

  S_izquierda1.UpdateSensor();
  tv.writeLCD(0,2,"ancho izquierda",(S_izquierda1.cm));
  Serial.println("ancho izquierda crudo: ");
  Serial.println(S_izquierda1.cm);
  if(myData) {
    myData.print(S_izquierda1.cm);
    myData.print(",");
  }
 
  Serial.println("ancho izquierda filtrado: ");
  Serial.println(S_izquierda1.dato[I_MUESTRA]);
  if(myData) {
    myData.print(S_izquierda1.dato[I_MUESTRA]);
    myData.print(",");
  }

  TreeRowVolume.Duty_TRV(S_izquierda1.dato[I_MUESTRA], velocidad, DISTANCIA_ENTRE_FILAS);
  duty4 = TreeRowVolume.duty_valv;
  buffer[4] = char(duty4);

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


//  S_izquierda2.UpdateSensor();
  //tv.writeLCD(S_izquierda2.cm);
  duty5  = 80;//S_izquierda2.cm / 200;
  buffer[5] = char(duty5);


//  S_izquierda3.UpdateSensor();
  //tv.writeLCD(S_izquierda3.cm);
  duty6  = 90;//S_izquierda3.cm / 200;
  buffer[6] = char(duty6);
  buffer[0]='*';
  buffer[7] = '#';

  if(myData) {
    myData.close();
  }


  for (i = 0; i < LAR_BUFFER ; ++i) {

    Serial2.write(buffer[i]);
    Serial.println(buffer[i]);

    delayMicroseconds(30);


}
}
  
////INTERRUPCION COMUNICACION
//void requestEvent() {
//  Wire.write(buffer); // respond with message of 6 bytes (se conecta al 4 y 5 del uno con 20 y 21 del mega correspondientemente)
// // Serial.print("enviado");
//  // as expected by master
//}
//







