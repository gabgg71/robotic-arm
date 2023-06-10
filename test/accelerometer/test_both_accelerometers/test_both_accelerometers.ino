#include <SoftwareSerial.h>
#include "Simple_MPU6050.h"
#define MPU6050_ADDRESS_AD0_LOW     0x68      // hand
#define MPU6050_ADDRESS_AD0_HIGH    0x69      // elbow
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW 
    
Simple_MPU6050 mpu_hand;
Simple_MPU6050 mpu_elbow;

#define OFFSETSMANO  -1728,    -822,    1600,     129,      93,     -15
#define OFFSETSCODO  -954,   -3190,     770,     145,    -221,       5

#define spamtimer(t) for (static uint32_t SpamTimer; (uint32_t)(millis() - SpamTimer) >= (t); SpamTimer = millis())
#define printfloatx(Name,Variable,Spaces,Precision,EndTxt) print(Name); {char S[(Spaces + Precision + 3)];Serial.print(F(" ")); Serial.print(dtostrf((float)Variable,Spaces,Precision ,S));}Serial.print(EndTxt);


int last_pitch = 0;
int last_rotation = 0;
int last_elbow = 0;

void setup()
{
  uint8_t val;
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif    
  Serial.begin(115200); 
  while (!Serial);      
  Serial.println(F("Start:"));  
#ifdef OFFSETSCODO              
  Serial.println(F("Using predefined offsets"));   
  mpu_elbow.SetAddress(MPU6050_ADDRESS_AD0_LOW).load_DMP_Image(OFFSETSCODO);  
  mpu_hand.SetAddress(MPU6050_ADDRESS_AD0_HIGH).load_DMP_Image(OFFSETSMANO);
#endif
  mpu_elbow.on_FIFO(show_values_elbow); 
  mpu_hand.on_FIFO(show_values_hand);
  delay(2000);
}

void loop()
{
  mpu_hand.dmp_read_fifo(); 
  mpu_elbow.dmp_read_fifo(); 
}

//
void show_values_elbow(int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp) {	
  uint8_t SpamDelay = 100;	
  Quaternion q;					
  VectorFloat gravity;				
  float ypr[3] = { 0, 0, 0 };	
  float xyz[3] = { 0, 0, 0 };	
  spamtimer(SpamDelay) {			
    mpu_elbow.GetQuaternion(&q, quat);
    mpu_elbow.GetGravity(&gravity, &q);	
    mpu_elbow.GetYawPitchRoll(ypr, &q, &gravity);	
    mpu_elbow.ConvertToDegrees(ypr, xyz);	
    // just interested in the sensor movement in roll.	
    if(xyz[2] > last_elbow+10 || xyz[2] < last_elbow-10){
      last_elbow = xyz[2];
      Serial.println("Significant elbow movement up/down: " + String(last_elbow));
    }		
  }
}

void show_values_hand(int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp) {  
  // just interested in the sensor movement in pitch (up-down) and roll (rotation).	
  uint8_t SpamDelay = 1000;    
  Quaternion q;         
  VectorFloat gravity;  
  float ypr[3] = { 0, 0, 0 };
  float xyz[3] = { 0, 0, 0 };     
  spamtimer(SpamDelay) {     
    mpu_hand.GetQuaternion(&q, quat); 
    mpu_hand.GetGravity(&gravity, &q);
    mpu_hand.GetYawPitchRoll(ypr, &q, &gravity);
    mpu_hand.ConvertToDegrees(ypr, xyz); 
    if(xyz[1] > last_pitch+20 || xyz[1] < last_pitch-20){
      last_pitch = xyz[1];
      Serial.println("Significant movement hand up/down: " + String(last_pitch));
    }
    if(xyz[2] > last_rotation+10 || xyz[2] < last_rotation-10){
      last_rotation = xyz[2];
      Serial.println("Significant hand rotation: " + String(last_rotation));
  }}
}