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

int nema_position = 0;


void setup() {
  Serial.begin(38400);
  pinMode(STEP, OUTPUT);  
  pinMode(DIR, OUTPUT); 
  servos.begin();
  servos.setPWMFreq(frecuencia);
  /*Correr esto solo la primera vez cuando nema acaba de ser calibrado
  EEPROM.put(0, 0);
  inicializaNema();
  volver a 0 con (1,300)*/
  setServoPulse(10, 50); //pair
  setServoPulse(12, 130); //pair
  setServoPulse(8,20); //elbow
  setServoPulse(14, 150); //wrist up-down
  setServoPulse(6,0); //wrist rot
  setServoPulse(4,90); //clamp
}

void loop() {
  if (Serial.available() > 0) {
    String msg_received = Serial.readString();
    msg_received.trim();
    if (msg_received != "") {
      char part = msg_received.charAt(0);
      msg_received = msg_received.substring(1);
      int movement = msg_received.toInt();
      if(part == 'P'){
        setServoPulse(4, movement);
      }
      else if(part == 'B'){
        calculate_nema_mov(movement);
        delay(200);
      }
      else if(part == 'H'){
        both_servos_rotate(movement);
      }
      else if(part == 'R'){
        setServoPulse(6, movement); 
      }
      else if(part == 'U'){
        setServoPulse(14, movement); 
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

void calculate_nema_mov(int posN){
  int steps;
  if(!((nema_position > posN-20) && (nema_position < posN+20))){
      if (nema_position < posN){
        steps = posN-nema_position;
        muevete(1, steps);
  }
  if(nema_position > posN){
        steps = nema_position-posN;
        muevete(0, steps);
  } 
  nema_position = posN; 
  EEPROM.put(0, nema_position);  //save nema position in memory
  
  } 
}

void muevete(int direccion, int steps){
  if(direccion ==1){ //right
    digitalWrite(DIR, HIGH);
  }else{  //left
    digitalWrite(DIR, LOW);
  }
  for (int x = 0; x < steps; x++)
  {
    digitalWrite(STEP, HIGH);
    delayMicroseconds(2500);
    digitalWrite(STEP, LOW);
    delayMicroseconds(2500);
  }
} 

void both_servos_rotate(int angulo){
    setServoPulse(10, angulo);
    setServoPulse(12, 180-angulo);
}
