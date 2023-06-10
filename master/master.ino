#include <SoftwareSerial.h>
#include "Simple_MPU6050.h"
#define MPU6050_ADDRESS_AD0_LOW     0x68    
#define MPU6050_ADDRESS_AD0_HIGH    0x69    
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW 
    
Simple_MPU6050 mpu_hand;
Simple_MPU6050 mpu_elbow;


#define OFFSETSCODO  -3340,   -2078,    1206,    -225,     117,     -29
#define OFFSETSMANO  -7288,    1100,    1762,      28,     -67,       6
#define spamtimer(t) for (static uint32_t SpamTimer; (uint32_t)(millis() - SpamTimer) >= (t); SpamTimer = millis())
#define printfloatx(Name,Variable,Spaces,Precision,EndTxt) print(Name); {char S[(Spaces + Precision + 3)];Serial.print(F(" ")); Serial.print(dtostrf((float)Variable,Spaces,Precision ,S));}Serial.print(EndTxt);

SoftwareSerial miBT (10,11);

const int flex_clamp = A1; 
const int flex_base = A4; 
const int flex_shoulder = A6; 


const int start_base_flx = 880;
const int start_shoulder_flx = 970;
const int start_clamp_flx = 950;

int last_clamp = 90;
int last_base =90; 
int last_shoulder=90;
int last_pitch = 90;
int last_rotation = 90;
int last_elbow = 90;


char message[5];
String info = "";

void setup()
{
  miBT.begin(38400); 

  pinMode(flex_clamp, INPUT);
  pinMode(flex_base, INPUT);
  pinMode(flex_shoulder, INPUT);

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
#ifdef OFFSETSMANO              
  mpu_hand.SetAddress(MPU6050_ADDRESS_AD0_HIGH).load_DMP_Image(OFFSETSMANO);
#ifdef OFFSETSCODO              
  mpu_elbow.SetAddress(MPU6050_ADDRESS_AD0_LOW).load_DMP_Image(OFFSETSCODO);  
#endif
  mpu_elbow.on_FIFO(show_values_elbow); 
  mpu_hand.on_FIFO(show_values_hand);
  delay(2000);
}

void loop()
{
  int data_clamp = analogRead(flex_clamp);
  int data_base = analogRead(flex_base);
  int data_shoulder = analogRead(flex_shoulder);
 

  data_clamp = map(data_clamp, start_clamp_flx,1023, 0, 180);   
  if(data_clamp > last_clamp+10 || data_clamp < last_clamp-10){
    last_clamp = constrain(data_clamp, 0, 180);
    send_data("P"+String(last_clamp));
  }

  data_base = map(data_base, start_base_flx,1023, 0, 180);
  if(data_base > last_base+20 || data_base < last_base-20){
    last_base = constrain(data_base, 0, 180);
    send_data("B"+String(last_base));
   } 

  data_shoulder = map(data_shoulder, start_shoulder_flx,1023, 20,120);
  if(data_shoulder > last_shoulder+10 || data_shoulder < last_shoulder-10){
    last_shoulder = constrain(data_shoulder, 20,120);
    send_data("H"+String(last_shoulder));
  }

  mpu_hand.dmp_read_fifo();
  // mpu_elbow.dmp_read_fifo(); 
  delay(1200);
}

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
    int data_elbow = map(xyz[1], -90,90, 0, 180);
    if(xyz[2] > last_elbow+15 || xyz[2] < last_elbow-15){
      last_elbow = constrain(data_elbow, 0, 180);
      send_data("E"+String(last_elbow));
      Serial.println("Significant elbow movement up/down: " + String(last_elbow));
    }		
  }
}


void show_values_hand(int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp) {  
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

    int data_pitch = map(xyz[1], -90,90, 0, 180);
    if(data_pitch > last_pitch+15 || data_pitch < last_pitch-15){
      last_pitch = constrain(data_pitch, 0, 180);
      send_data("U"+String(last_pitch));
      Serial.println("Significant movement hand up/down: " + String(last_pitch));
    }
    
    int data_rotation = map(xyz[2], -90,90, 0, 180);
    if(data_rotation > last_rotation+15 || data_rotation < last_rotation-15){
      last_rotation = constrain(data_rotation, 0, 180);
      send_data("R"+String(last_rotation));
      Serial.println("Significant hand rotation: " + String(last_rotation));
    }
  }
}

void send_data(String info){
    for(int i =0; i< 5; i++){
      message[i]= info.charAt(i);
    }
    miBT.write(message, sizeof(message));
}


