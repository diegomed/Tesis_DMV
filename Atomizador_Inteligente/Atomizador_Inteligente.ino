
#include <SD.h>

#include <TinyGPS.h>
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

#define ss Serial1



LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
float duty1, duty2, duty3, duty4, duty5, duty6;
float* rejunte;
char buffer[8];
int chipSelect = 53;
File myData; //creo objeto de tipo File
const byte LED = 13;



//************************************************* clase sensor**************************************************
class sensor {

  public:
    int i;
    float           dato[CANT_MUESTRAS];
    unsigned long   SensorMillis, CurrentMillis;
    const float     TimeSens = TIME_SENS;
    int             pin4, pin2;
    float           cm, duration;
    float           avg_cm[4];
    float           dif[2];
    volatile unsigned long   risingMicros;
    volatile unsigned long   fallingMicros;
    volatile unsigned long   pulseTime;

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

        duration = asignNewPulseDuration();//pulseIn(pin2, HIGH,80000);
        cm = microsecondsToCentimeters(duration);

//
//        dato[i] = cm;
//        i++;

 if (cm >= (DISTANCIA_ENTRE_FILAS*100/2)){         // si sobrepasa la medida de la mitad de la entre fila le coloco un cero
        dato[i] = 0;
       
       }

       else{
        dato[i]= DISTANCIA_ENTRE_FILAS*100/2 - cm;
      
       }
        
        
        i++;

        digitalWrite(pin4, LOW);
        delayMicroseconds(100);   //Tiempo minimo recomendado entre lecturas
      }
    }

    void UpdateSensor() {
      int n, i;
      n = CANT_MUESTRAS;
     int dato_aux;
      CurrentMillis = millis();

      //Serial.print(TimeSens);
      if (CurrentMillis - SensorMillis >= TimeSens) { // Tiempo cada cuanto funciona el sensor

      for (i = 0; i < n - 1; ++i) {             // Primero corre los valores del array dejando libre el ultimo
          dato[i] = dato[i + 1];

           //Serial. print(dato[i+1]);

          //Serial. print(" ");
        
        
        }


        digitalWrite(pin4, HIGH);
        //delayMicroseconds(5);
        duration = asignNewPulseDuration();//pulseIn(pin2, HIGH,80000);
        cm = microsecondsToCentimeters(duration);
        //dato[n - 1] = cm;
        
       if (cm >= DISTANCIA_ENTRE_FILAS*100/2){         // si sobrepasa la medida de la mitad de la entre fila le coloco un cero
        dato[n-1] = 0;
       
      
       }

       else{
        dato[n-1]= DISTANCIA_ENTRE_FILAS*100/2 - cm;
       }
        
        digitalWrite(pin4, LOW);


//FILTRADO DE MUESTRAS
        avg_cm[0] =  mean_primera(dato);              //Calculo del promedio en cm
        avg_cm[1] =  mean_ultima(dato); //Calculo del promedio en cm
        avg_cm[2]=   mean_todo(dato);
        avg_cm[3]=   mean_sig3M(dato);
        dif[0] = abs(avg_cm[0] - dato[I_MUESTRA]);
        dif[1] = abs(avg_cm[1] - dato[I_MUESTRA]);


        if (dif[0] > UMBRAL) {

          if (dif[1] > UMBRAL) {


            dato[I_MUESTRA] = avg_cm[2] ;
            //Serial.print("descarta  ");
          }
        }
         dato_aux= dato[I_MUESTRA];

         if (dato_aux==0){
          //Serial.print("entra");

          if (avg_cm[3]>0.0){

         dato[I_MUESTRA]= avg_cm[1];
          
         }
         else {
          dato[I_MUESTRA]=0;
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

    float asignNewPulseDuration() {
      return pulseTime;
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


    float mean_sig3M(float dato[]) {

      int  i;
      float mean;
      int init;
      init = I_MUESTRA + 1;

      for (i = init; i < init+3; ++i) { // toma las siguientes 3 muestras a I_MUESTRA y hace el promedio
        mean += dato[i];
      }


      return mean /3;
    }

    float mean_todo(float dato[]) {

      int  i;
      float mean;
     

      for (i=0; i < CANT_MUESTRAS; ++i) {
        mean += dato[i];
      }


      return mean / CANT_MUESTRAS;
    }
};


//*************************************************************Clase Pantalla*************************************************************** 
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


//********************************************************Clase TRV***************************************************************
class TRV {
    public:
    float Calc_TRV;
    float Litros_ha;
    float Caudal;
    float Ancho_Copa;
    int duty_valv, aux;
 
    public:
    TRV() {}
  
  void Duty_TRV(float Medida, float Velocidad_avance, float Entre_filas) { // Medida del sensor en cm, velocidad de avance en Km y entre filas en m
    
       
  
        Ancho_Copa = Medida/100; //Paso la medida de cm a m
  
        Calc_TRV = (VENTANA_SENSOR * Ancho_Copa * 10000) / DISTANCIA_ENTRE_FILAS;
  
        Litros_ha = (Calc_TRV * DOSIS) / 1000;
  
        Caudal = (Litros_ha * Velocidad_avance * DISTANCIA_ENTRE_FILAS) / 600;
  
        duty_valv = (Caudal / CAUDAL_MAX_2BOQUILLAS) * 100;
  
    if (duty_valv < 35) // duty mas pequeño al que la valvula trabaja
    {
       //Serial.print("medida");
        //Serial.println(Medida);
        aux=Medida;
       if(aux==0)   //Si no ve nada, apaga la valvula
       {
        duty_valv = 0;
       
       }
      else{ 
        duty_valv = 35;
      }
    }
    if (duty_valv > 80) // duty mas grande al que la valvula trabaja
    { 
      duty_valv = 80;
    }
        
//    if(Medida==0)   //Si no ve nada, apaga la valvula
//    {
//      duty_valv = 0;
//    }
  }
};
//****************************************************************Clase GPS*****************************************************
class GPS_UB {

  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  public:
  float velocidad=0;
  double posicion[2];
  TinyGPS gps;
    
    public:
    GPS_UB() {}
    



void UpdateGPS() {
    
    

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (ss.available())
    {
      char c = ss.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);

    velocidad = gps.f_speed_kmph();
   
    if(flat != TinyGPS::GPS_INVALID_F_ANGLE)
      posicion[0]= flat;
     
    if(flon != TinyGPS::GPS_INVALID_F_ANGLE)
      posicion[1]= flon;

  }
  //*********************************** DeBug GPS********************************************************************************
//    Serial.println( );
//    Serial.print("LAT=");
//    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
//    Serial.print(" LON=");
//    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
//    Serial.print(" SPEED=");
//    Serial.print(gps.f_speed_kmph()); // Speed in kilometers per hour (double)
//    Serial.print(" SAT=");
//    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
//    Serial.print(" PREC=");
//    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());

  
  
//  gps.stats(&chars, &sentences, &failed);
//  Serial.print(" CHARS=");
//  Serial.print(chars);
//  Serial.print(" SENTENCES=");
//  Serial.print(sentences);
//  Serial.print(" CSUM ERR=");
//  Serial.println(failed);
// 
//  if (chars == 0)
//    Serial.println("** No characters received from GPS: check wiring **");

//*********************************************************************************************************************************
} 
 
};


//********************************************************Creacion de las Clases***************************************************

sensor S_derecha1(9, 3);//pin enable 2, pin ouput 3  
//sensor S_derecha2(4, 5);
//sensor S_derecha3(6, 7);

sensor S_izquierda1(8, 2);
//sensor S_izquierda2(10, 11);
//sensor S_izquierda3(12, 13);

TRV TreeRowVolume;

PantallaLCD tv(50);


GPS_UB GPS_AT;

void onChangeDerecha ()
{
  if (digitalRead (3) == HIGH){
    S_derecha1.risingMicros = micros();
    //digitalWrite (LED, HIGH);
  }
  else{
    S_derecha1.fallingMicros = micros();
    S_derecha1.pulseTime = S_derecha1.fallingMicros - S_derecha1.risingMicros;
    //digitalWrite (LED, LOW);
  }
}    

void onChangeIzquierda ()
{
  if (digitalRead (2) == HIGH){
    S_izquierda1.risingMicros = micros();
    digitalWrite (LED, HIGH);
  }
  else{
    S_izquierda1.fallingMicros = micros();
    S_izquierda1.pulseTime = S_izquierda1.fallingMicros - S_izquierda1.risingMicros;
    digitalWrite (LED, LOW);
  }
} 


//***************************************************Setup******************************************************************
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial2.begin(9600);

  ss.begin(9600);
  pinMode(chipSelect, OUTPUT); //esto hay que hacerlo pare el DataLogger, porque si
  digitalWrite(chipSelect, HIGH);//esto hay que hacerlo para el DataLogger, porque si
  pinMode (LED, OUTPUT);
  SD.begin(chipSelect); //empiezo la comunicacion con la SD, CS (chip select) esta conectado al pin 53 del arduino



 
  tv.InitLCD();

  digitalWrite (2, HIGH);  // internal pull-up resistor
  digitalWrite (3, HIGH);  // internal pull-up resistor
  attachInterrupt (digitalPinToInterrupt (3), onChangeDerecha, CHANGE);  // attach interrupt handler
  attachInterrupt (digitalPinToInterrupt (2), onChangeIzquierda, CHANGE);  // attach interrupt handler
  
  myData = SD.open("PTData.txt", FILE_WRITE);
  delay(100);
  if(myData) {
    myData.print("ancho D crudo:");
    myData.print(",");
    myData.print("ancho D filtrado:");
    myData.print(",");
    myData.print("Calc_TRV D");
    myData.print(",");
    myData.print("Litros_ha D");
    myData.print(",");
    myData.print("Caudal D");
    myData.print(",");
    myData.print("Ancho_Copa D");
    myData.print(",");
    myData.print("duty_valv D");
    myData.print(",");
    myData.print("ancho I crudo:");
    myData.print(",");
    myData.print("ancho I filtrado:");
    myData.print(",");
    myData.print("Calc_TRV I");
    myData.print(",");
    myData.print("Litros_ha I");
    myData.print(",");
    myData.print("Caudal I");
    myData.print(",");
    myData.print("Ancho_Copa I");
    myData.print(",");
    myData.println("duty_valv I");
    myData.print(",");
    myData.println("velocidad");
    myData.close();
    delay(100);
  }
  
}


//**************************************************************************LOOP***********************************************************
void loop() {

  float salida;
  float velocidad;
  int i, auxvel;
  i = 0;
  velocidad = 5.4;
  // put your main code here, to run repeatedly:
 
 GPS_AT.UpdateGPS();
//************************************************************Debug Gps ********************************************************************
// Serial.print("LAT=");
// Serial.print(GPS_AT.posicion[0],6);
// 
// Serial.print(" LON=");
// Serial.println(GPS_AT.posicion[1],6);
// 
// Serial.print(" SPEED=");
// Serial.println(GPS_AT.velocidad); // Speed in kilometers per hour (double)

 //********************************************************************************************************************************************
 auxvel=GPS_AT.velocidad;

 if (auxvel !=0){
  velocidad=GPS_AT.velocidad;
  //Serial.println("VEL0");
 }
 // Serial.print("VEL");
//   Serial.println(velocidad);

 

 
  myData = SD.open("PTData.txt", FILE_WRITE);

  S_derecha1.UpdateSensor();
  tv.writeLCD(0,0,"ancho D",(S_derecha1.dato[I_MUESTRA]));
  TreeRowVolume.Duty_TRV(S_derecha1.dato[I_MUESTRA], velocidad, DISTANCIA_ENTRE_FILAS);
  duty1 = TreeRowVolume.duty_valv;
  buffer[1] = char(duty1);

  lcd.setCursor(8,0);
  lcd.print("lat ");
  lcd.print(GPS_AT.posicion[0]);
        
  
  if(myData) {
    myData.print(S_derecha1.cm);
    myData.print(",");
  }
  
  if(myData) {
    myData.print(S_derecha1.dato[I_MUESTRA]);
    myData.print(",");
  }

//*****************************************Debug sensor derecha ******************************************************  

//  Serial.println("ancho derecha crudo: ");
//  Serial.println(S_derecha1.cm);
//  Serial.println("ancho derecha filtrado: ");
//  Serial.println(S_derecha1.dato[I_MUESTRA]);
// 
//********************************************************************************************************  
  
  
  
  
  
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
 tv.writeLCD(0,2,"ancho I",S_izquierda1.dato[I_MUESTRA]);
  TreeRowVolume.Duty_TRV(S_izquierda1.dato[I_MUESTRA], velocidad, DISTANCIA_ENTRE_FILAS);
  duty4 = TreeRowVolume.duty_valv;
  buffer[4] = char(duty4);

  
  if(myData) {
    myData.print(S_izquierda1.cm);
    myData.print(",");
  }
  if(myData) {
    myData.print(S_izquierda1.dato[I_MUESTRA]);
    myData.print(",");
  }

//*************************************Debug sensor izquierda ****************************************************************************

//  Serial.println("ancho izquierda crudo: ");
//  Serial.println(S_izquierda1.cm);
//  Serial.println("ancho izquierda filtrado: ");
//  Serial.println(S_izquierda1.dato[I_MUESTRA]);
// 
//*********************************************************************************************************************************************
  
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
    myData.print(",");
    myData.println(velocidad);
 

}

//  S_izquierda2.UpdateSensor();
  //tv.writeLCD(S_izquierda2.cm);
  duty5  = 80;//S_izquierda2.cm / 200;
  buffer[5] = char(duty5);

//  S_izquierda3.UpdateSensor();
  //tv.writeLCD(S_izquierda3.cm);
  duty6  = 90;//S_izquierda3.cm / 200;
  buffer[6] = char (duty6);
  buffer[0]='{';
  buffer[7] = '}';

  lcd.setCursor(8,1);
  lcd.print("lon ");
  lcd.print(GPS_AT.posicion[1]);

  lcd.setCursor(8,2);
  lcd.print("Vel ");
  lcd.print(velocidad);
  

 
 
 
  if(myData) {
    myData.close();
  }

  for (i = 0; i < LAR_BUFFER ; ++i) 
  {
    Serial2.write(buffer[i]);
 //********************************Debug Comunicacion************************************************
 
    //Serial.println(buffer[i]);

 //*************************************************************************************************
    delayMicroseconds(30);
  }
}
  



