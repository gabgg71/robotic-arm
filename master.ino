#include <SoftwareSerial.h>
#include "Simple_MPU6050.h"
#define MPU6050_ADDRESS_AD0_LOW     0x68      // direccion I2C con AD0 en LOW o sin conexion
#define MPU6050_ADDRESS_AD0_HIGH    0x69      // direccion I2C con AD0 en HIGH
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW // por defecto AD0 en LOW
    
Simple_MPU6050 mpuMano;
//#define OFFSETSMANO  -8700,    1060,    4122,      13,     -55,      12

#define OFFSETSCODO  -3340,   -2078,    1206,    -225,     117,     -29
#define OFFSETSMANO  -7288,    1100,    1762,      28,     -67,       6
#define spamtimer(t) for (static uint32_t SpamTimer; (uint32_t)(millis() - SpamTimer) >= (t); SpamTimer = millis())
#define printfloatx(Name,Variable,Spaces,Precision,EndTxt) print(Name); {char S[(Spaces + Precision + 3)];Serial.print(F(" ")); Serial.print(dtostrf((float)Variable,Spaces,Precision ,S));}Serial.print(EndTxt);

SoftwareSerial miBT (10,11);

const int flexPinza = A1; 
const int flexBase = A4; 
const int flexHombro = A6; 
int pinzaPasado = 90;
int basePasado =90; 
int hombroPasado=90;
char envio[5];
String algo = "";
int antManoM1 = 0;
int antManoM2 = 0;
int servoManoM1;
int servoManoM2;
void setup()
{
  miBT.begin(38400); 
  pinMode(flexPinza, INPUT);
  pinMode(flexBase, INPUT);
  pinMode(flexHombro, INPUT);
  uint8_t val;
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif    
  Serial.begin(115200); 
  while (!Serial);      
  Serial.println(F("Inicio:"));  
#ifdef OFFSETSCODO              
  Serial.println(F("Usando Offsets predefinidos"));   
  //mpu.SetAddress(MPU6050_ADDRESS_AD0_LOW).load_DMP_Image(OFFSETSCODO);  
  mpuMano.SetAddress(MPU6050_ADDRESS_AD0_HIGH).load_DMP_Image(OFFSETSMANO);
#endif
  //mpu.on_FIFO(mostrar_valores); 
  mpuMano.on_FIFO(mostrar_valores_mano);
  delay(2000);
}

void loop()
{
  int fSensorPinza = analogRead(flexPinza);
  int fSensorBase = analogRead(flexBase);
  int fSensorHombro = analogRead(flexHombro);
  if(!((pinzaPasado  > fSensorPinza-10) && (pinzaPasado  < fSensorPinza+10))){
    algo = "P"+String(fSensorPinza);
    for(int i =0; i< 5; i++){
      envio[i]= algo.charAt(i);
    }
    pinzaPasado = fSensorPinza;
    miBT.write(envio, sizeof(envio)); Serial.println("envio de pinza");
  }
  //validacion de base
  if(!((basePasado > fSensorBase-20) && (basePasado < fSensorBase+20))){
      algo = "B"+String(fSensorBase);
      for(int i =0; i< 5; i++){
        envio[i]= algo.charAt(i);
      }
      basePasado = fSensorBase;
      miBT.write(envio, sizeof(envio)); Serial.println("envio de base"); 
      } 
  
  //validacion del hombro
  if(!((hombroPasado  > fSensorHombro-10) && (hombroPasado < fSensorHombro+10))){
    algo = "H"+String(fSensorHombro);
    for(int i =0; i< 5; i++){
      envio[i]= algo.charAt(i);
    }
    hombroPasado = fSensorHombro;
    miBT.write(envio, sizeof(envio)); Serial.println("envio hombro");
  }

  if(!((hombroPasado  > fSensorHombro-10) && (hombroPasado < fSensorHombro+10))){
    algo = "A"+String(fSensorHombro);
    for(int i =0; i< 5; i++){
      envio[i]= algo.charAt(i);
    }
    hombroPasado = fSensorHombro;
    miBT.write(envio, sizeof(envio)); Serial.println("envio hombro");
  }
 

  mpuMano.dmp_read_fifo();
  
  delay(1200);

}

void mostrar_valores_mano (int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp) {  
  uint8_t SpamDelay = 1000;    
  Quaternion q;         
  VectorFloat gravity;  
  float ypr[3] = { 0, 0, 0 };
  float xyz[3] = { 0, 0, 0 };     
  spamtimer(SpamDelay) {     
    mpuMano.GetQuaternion(&q, quat); 
    mpuMano.GetGravity(&gravity, &q);
    mpuMano.GetYawPitchRoll(ypr, &q, &gravity);
    mpuMano.ConvertToDegrees(ypr, xyz); 
    
    //arriba y abajo 
    servoManoM1 = map(xyz[1], -90,90, 0, 180);
    if(!((servoManoM1 > antManoM1-15) && (servoManoM1 < antManoM1+15))){
      servoManoM1 = constrain(servoManoM1, 0, 180);
      //Enviar dato 
      algo = "U"+String(servoManoM1);
      for(int i =0; i< 5; i++){
        envio[i]= algo.charAt(i);
      }
      antManoM1 = servoManoM1;
      miBT.write(envio, sizeof(envio)); 
      Serial.print("envio muñeca arriba abajo");
      Serial.println(xyz[1]);
    }
    
    //movimiento izq y derecha
    servoManoM2 = map(xyz[2], -90,90, 0, 180);
    if(!((servoManoM2 > antManoM2-15) && (servoManoM2 < antManoM2+15))){
      //Serial.println(xyz[2]);
      servoManoM2 = constrain(servoManoM2, 0, 180);
      algo = "R"+String(servoManoM2);
      for(int i =0; i< 5; i++){
        envio[i]= algo.charAt(i);
      }
      antManoM2 = servoManoM2;
      miBT.write(envio, sizeof(envio)); 
      Serial.print("envio muñeca rotacion (x)");
      Serial.println(xyz[2]);
    }
  }
}
