//Esclavo

#include <Adafruit_PWMServoDriver.h>
#include "Simple_MPU6050.h"
#include <EEPROM.h>
Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver();

#define pos0 102
#define pos180 512
#define frecuencia 50
#define STEP 4      
#define DIR 5  

int DATO = 0;
String necesario = "";
char tipo;
int valor;
const int initFlexGiroBase = 880;
const int initFlexEstiraH = 970;
const int initFlexPinza = 950;
int posicionNema = 0;
int anguloServo = 0;
int nemaposition;
int servosPinza;
int servosPosition;


void setup() {
  Serial.begin(38400);
  Serial.println("estoy iniciando");
  pinMode(STEP, OUTPUT);  
  pinMode(DIR, OUTPUT); 
  servos.begin();
  servos.setPWMFreq(frecuencia);
  /*Correr esto solo la primera vez cuando nema acaba de ser calibrado*/
  //EEPROM.put(0, 0);
  //inicializaNema();
  //volver a 0 con (1,300)
  //calculaGiro(45);
  setServoPulse(10, 50);
  //el 12 es el de los 2 servos
  setServoPulse(12, 130);
  // el 8 es el del codo
  setServoPulse(8,20);
  //el 14 es de muñeca arriba y abajo */
  setServoPulse(14, 150);
  // el 6 es de rotacion de muñeca
  
  setServoPulse(6,0);
  // el 4 es de la pinza
  setServoPulse(4,90);
}

void loop() {
  if (Serial.available() > 0) {
    necesario = Serial.readString();
    necesario.trim();
    if (necesario != "") {
      tipo = necesario.charAt(0);
      //Es del tipo de la pinza 
      if(tipo == 'P'){
        necesario = necesario.substring(1);
        valor = necesario.toInt();
        servosPinza = map(valor, initFlexPinza, 1023, 0, 180);
        servosPinza = constrain(servosPinza, 0, 180);
        setServoPulse(4, servosPinza);
      }
      //Es de la base
      if(tipo == 'B'){
        necesario = necesario.substring(1);
        valor = necesario.toInt();
        nemaposition = map(valor, initFlexGiroBase,1023, 0, 180);
        nemaposition = constrain(nemaposition, 0, 180);
        calculaMovimiento(nemaposition);
        delay(200);
      }
      //Es del hombro
      if(tipo == 'H'){
        necesario = necesario.substring(1);
        valor = necesario.toInt();
        servosPosition = map(valor, initFlexEstiraH,1000,20, 120);
        servosPosition = constrain(servosPosition, 20, 120);
        calculaGiro(servosPosition);
      }
      //Validacion de rotacion de muñeca
      if(tipo == 'R'){
        necesario = necesario.substring(1);
        valor = necesario.toInt();
        setServoPulse(6, valor); 
      }
      //Validacion de subida y bajada de muñeca
      if(tipo == 'U'){
        necesario = necesario.substring(1);
        valor = necesario.toInt();
        setServoPulse(14, valor); 
      }
    }
    
  }

  delay(200);
}

void setServoPulse(uint8_t n, int angulo) {
  int pulso;
  pulso = map(angulo, 0, 180, pos0, pos180);
  servos.setPWM(n, 0, pulso);
}

void calculaMovimiento(int posN){
  int pasos;
  if(!((posicionNema > posN-20) && (posicionNema < posN+20))){
      if (posicionNema < posN){
    //ir a la derecha
    pasos = posN-posicionNema;
    //guardar posicionNema en memoria
    posicionNema = posN; 
    EEPROM.put(0, posicionNema);
    muevete(1, pasos);
  }
  if(posicionNema > posN){
    pasos = posicionNema-posN;
    posicionNema = posN; 
    EEPROM.put(0, posicionNema);
    muevete(0, pasos);
  } 
    } 
}

void muevete(int direccion, int pasos){
  //derecha
  if(direccion ==1){
    digitalWrite(DIR, HIGH);
  }else{
    //izquierda
    digitalWrite(DIR, LOW);
  }
  for (int x = 0; x < pasos; x++)
  {
    digitalWrite(STEP, HIGH);
    delayMicroseconds(2500);
    digitalWrite(STEP, LOW);
    delayMicroseconds(2500);
  }
} 

void calculaGiro(int angulo){
    setServoPulse(10, angulo);
    setServoPulse(12, 180-angulo);
}
