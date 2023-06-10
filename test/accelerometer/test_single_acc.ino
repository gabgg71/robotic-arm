#include "Simple_MPU6050.h"				
#define MPU6050_ADDRESS_AD0_LOW     0x68			
#define MPU6050_ADDRESS_AD0_HIGH    0x69			
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_HIGH	

Simple_MPU6050 mpu;			

#define spamtimer(t) for (static uint32_t SpamTimer; (uint32_t)(millis() - SpamTimer) >= (t); SpamTimer = millis())

#define printfloatx(Name,Variable,Spaces,Precision,EndTxt) print(Name); {char S[(Spaces + Precision + 3)];Serial.print(F(" ")); Serial.print(dtostrf((float)Variable,Spaces,Precision ,S));}Serial.print(EndTxt);

void show_values (int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp) {	
  uint8_t SpamDelay = 100;	
  Quaternion q;					
  VectorFloat gravity;				
  float ypr[3] = { 0, 0, 0 };	
  float xyz[3] = { 0, 0, 0 };	
  spamtimer(SpamDelay) {			
    mpu.GetQuaternion(&q, quat);
    mpu.GetGravity(&gravity, &q);	
    mpu.GetYawPitchRoll(ypr, &q, &gravity);	
    mpu.ConvertToDegrees(ypr, xyz);		
    Serial.printfloatx(F("Yaw")  , xyz[0], 9, 4, F(",   "));  
    Serial.printfloatx(F("Pitch"), xyz[1], 9, 4, F(",   "));  
    Serial.printfloatx(F("Roll") , xyz[2], 9, 4, F(",   "));  
    Serial.println();				
  }
}

void setup() {
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
#ifdef OFFSETS								
  Serial.println(F("Predefined offsets"));		
  mpu.SetAddress(MPU6050_ADDRESS_AD0_LOW).load_DMP_Image(OFFSETS);	

#else										
  Serial.println(F(" There is not any established offset, we will create new ones\n"	
                   " Place the sensor on a flat surface and wait a few seconds.\n"
                   " \t\tPress any key and ENTER."));
  while (Serial.available() && Serial.read());		
  while (!Serial.available());   		           
  while (Serial.available() && Serial.read()); 	
  mpu.SetAddress(MPU6050_ADDRESS_AD0_HIGH).CalibrateMPU().load_DMP_Image();
#endif
  mpu.on_FIFO(show_values);	
}

void loop() {
  mpu.dmp_read_fifo();		
}				